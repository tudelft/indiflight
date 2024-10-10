#!/usr/bin/env python3

# Python bindings for the MOCKUP target interface functions
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This file is part of Indiflight.
#
# Indiflight is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# Indiflight is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.
#
# If not, see <https://www.gnu.org/licenses/>.


import ctypes as ct
import numpy as np
import os

# these "enum" classes are a big limitation currently, as they need to be 
# updated manually if things are appended/inserted in Indiflight
# similar problem with the configurator on a few things.. but yeah.. this is what it will be
class flightModeFlags():
    ANGLE_MODE         = (1 << 0)
    HORIZON_MODE       = (1 << 1)
    MAG_MODE           = (1 << 2)
    HEADFREE_MODE      = (1 << 6)
    PASSTHRU_MODE      = (1 << 8)
    FAILSAFE_MODE      = (1 << 10)
    GPS_RESCUE_MODE    = (1 << 11)
    VELOCITY_MODE      = (1 << 12)
    POSITION_MODE      = (1 << 13)
    CATAPULT_MODE      = (1 << 14)
    LEARNER_MODE       = (1 << 15)
    PID_MODE           = (1 << 16)
    NN_MODE            = (1 << 17)
    OFFBOARD_POSE_MODE = (1 << 18)

class flightLogDisarmReason():
    DISARM_REASON_SWITCH = 4
    DISARM_REASON_SYSTEM = 255

class boxId():
    # not for flight modes
    BOXARM = (1 << 0)
    BOXPREARM = (1 << 34)
    BOXTHROWTOARM = (1 << 35)

hid_codes = {
    'a': 0x04, 'b': 0x05, 'c': 0x06, 'd': 0x07, 'e': 0x08,
    'f': 0x09, 'g': 0x0A, 'h': 0x0B, 'i': 0x0C, 'j': 0x0D,
    'k': 0x0E, 'l': 0x0F, 'm': 0x10, 'n': 0x11, 'o': 0x12,
    'p': 0x13, 'q': 0x14, 'r': 0x15, 's': 0x16, 't': 0x17,
    'u': 0x18, 'v': 0x19, 'w': 0x1A, 'x': 0x1B, 'y': 0x1C,
    'z': 0x1D, '0': 0x27, '1': 0x1E, '2': 0x1F, '3': 0x20,
    '4': 0x21, '5': 0x22, '6': 0x23, '7': 0x24, '8': 0x25,
    '9': 0x26, ' ': 0x2C, '\n': 0x28, '\b': 0x2A, '\t': 0x2B,
    'ESC': 0x29, 'LEFT': 0x50, 'UP': 0x48, 'RIGHT': 0x4D, 'DOWN': 0x50
}

# ctypes stuff
float_ptr = np.ctypeslib.ndpointer(dtype=ct.c_float, ndim=1)
timeUs_t = ct.c_uint32

class IndiflightSITLMockup():
    def __init__(self, libfile, N=4):
        self.libfile = libfile
        self.N = N

        # arrays
        self.gyro = np.zeros(3, dtype=ct.c_float)
        self.acc = np.zeros(3, dtype=ct.c_float)
        self.motorOmega = np.zeros(self.N, dtype=ct.c_float)
        self.motorCommands = np.zeros(self.N, dtype=ct.c_float)

        self.pos = np.zeros(3, dtype=ct.c_float)
        self.vel = np.zeros(3, dtype=ct.c_float)
        self.quat = np.zeros(4, dtype=ct.c_float)

        self.offboardPos = np.zeros(3, dtype=ct.c_float)
        self.offboardQuat = np.zeros(4, dtype=ct.c_float)

        self.posSp = np.zeros(3, dtype=ct.c_float)
        self.yawSp = 0.

        try:
            os.remove("./eeprom.bin")
        except OSError:
            pass

        self._loadLibAndInit()

    def __del__(self):
        #print("IndiflightSITLMockup destroyed: ensure disarmed to flush blackbox logs... ", end="")
        self.disarm( reason = flightLogDisarmReason.DISARM_REASON_SYSTEM )
        #print("done.")

#%% loading and config
    def _loadLibAndInit(self):
        self.lib = ct.CDLL( self.libfile )
        self.lib.init()

        # argtypes
        self.lib.setImu.argtypes = [float_ptr, float_ptr]
        self.lib.setMotorSpeed.argtypes = [float_ptr, ct.c_int]
        self.lib.setMocap.argtypes = [float_ptr, float_ptr, float_ptr]
        self.lib.setOffboardPose.argtypes = [float_ptr, float_ptr]
        self.lib.setPosSetpoint.argtypes = [float_ptr, ct.c_float]
        self.lib.getMotorOutputCommands.argtypes = [float_ptr, ct.c_int]
        self.lib.tick.argtypes = [timeUs_t]
        self.lib.processCharacterInteractive.argtypes = [ct.c_char]
        self.lib.processKey.argtypes = [ct.c_uint8]
        self.lib.disarm.argtypes = [ct.c_uint8]

        # most important states
        self.armingFlags = self.getVariableReference(ct.c_uint8, "armingFlags")
        self.flightModeFlags = self.getVariableReference(ct.c_uint32, "flightModeFlags")
        # misnomer. this is the boxes, not the flight modes.
        # also, this actually is a packed struct of multiple uint32.
        # ain't nobody got time for that. we support first 64 rc boxes
        self.rcModeActivationMask = self.getVariableReference(ct.c_uint64, "rcModeActivationMask") 

        # disable logging by default
        self.setLogging( False )

    def getVariableReference(self, type, variable):
        return type.in_dll(self.lib, variable)

    def load_profile(self, profile_txt):
        # upload profile, must not end with batch end\n save\n
        with open(profile_txt, 'r') as file:
            while True:
                char = file.read(1)
                if not char:
                    break
                self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        #todo: what if file already contains this? then we crash, i think
        for char in "batch end\nsave\n":
            self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        # realod lib
        self._loadLibAndInit()

    def issueCliCommand(self, string):
        if "save" in string:
            # save requires reload of the library and re-calling init()
            raise ValueError("cannot issue save command in this function. call saveConfig()")

        for char in string:
            self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        self.lib.processCharacterInteractive(bytes("\n", 'utf-8'))

        # todo: pipe clioutput to printf for MOCKUP/(SITL) target

    def saveConfig(self):
        self.disarm()
        for char in "\nsave\n":
            self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        self._loadLibAndInit()

#%% sending and receiving sensor data / flight control outputs
    def sendImu(self, gyro, acc):
        self.gyro[:] = gyro
        self.acc[:] = acc
        self.lib.setImu(self.gyro, self.acc)

    def sendMotorSpeeds(self, omega):
        n = len(omega)
        if n > self.N:
            raise TypeError("too many elements in omega. Must be at most self.N")

        self.motorOmega[:n] = omega
        self.lib.setMotorSpeed(self.motorOmega, n)

    def sendMocap(self, pos, vel, quat):
        self.pos[:] = pos
        self.vel[:] = vel
        self.quat[:] = quat
        self.lib.setMocap(self.pos, self.vel, self.quat)

    def sendOffboardPose(self, pos, quat):
        self.offboardPos[:] = pos
        self.offboardQuat[:] = quat
        self.lib.setMocap(self.offboardPos, self.offboardQuat)

    def sendPositionSetpoint(self, pos, yaw):
        self.posSp[:] = pos
        self.yawSp = yaw
        self.lib.setPosSetpoint(self.posSp, self.yawSp)

    def getMotorCommands(self):
        self.lib.getMotorOutputCommands(self.motorCommands, self.N)
        return np.array(self.motorCommands, dtype=float)

    def sendKeyboard(self, key):
        if key in hid_codes.keys():
            self.lib.processKey(hid_codes[key])
        else:
            raise ValueError(f"The key {key} is not present in the hid_codes.")

#%% managing modes and states
    def enableFlightMode(self, mode):
        self.flightModeFlags.value |= mode

    def disableFlightMode(self, mode):
        self.flightModeFlags.value &= ~mode

    def getCurrentFlightModes(self):
        raise NotImplementedError("ToDo")

    def enableRxBox(self, box):
        self.rcModeActivationMask.value |= box

    def disableRxBox(self, box):
        self.rcModeActivationMask.value &= ~box

    def getCurrentRxBoxes(self):
        raise NotImplementedError("ToDo")

    def setLogging(self, enabled):
        self.issueCliCommand(f"set blackbox_device = {'SITL' if enabled else 'NONE'}")
        print()

    def arm(self):
        self.armingFlags.value = 1

    def disarm(self, reason = flightLogDisarmReason.DISARM_REASON_SWITCH):
        self.lib.disarm(reason) # also marks blackbox finished
        self.lib.blackboxUpdate()
        self.lib.blackboxUpdate() # call twice to flush all blackbox state changes
        self.armingFlags.value = 0 # make extra sure

#%% finally: tick the inner loop controls
    def tick(self, dtUs):
        # ToDo: timing of IMU and EKF stuff is now really hardcoded
        self.lib.tick( dtUs )


if __name__=="__main__":
    # init the Mockup. This deletes any existing "./eeprom.bin" and generates one with default settings
    mockup = IndiflightSITLMockup( "./obj/main/indiflight_MOCKUP.so" )

    # load settings from a profile exported from the configurator ontop of this 
    # default "./eeprom.bin". On exit, the "./eeprom.bin" is overwritten 
    # with this combined result.
    mockup.load_profile( "./src/utils/SIL.txt" )

    # set flight modes and rc switch positions
    mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | 
                            flightModeFlags.POSITION_MODE)
    #mockup.enableRxBox(boxId.BOXTHROWTOARM)

    # enable logging
    mockup.setLogging( True )

    # send a position setpoint
    mockup.sendPositionSetpoint( [0., 1., -1.], np.pi )

    # arm
    mockup.arm()

    for i in range( 20000 ):
        # send (some) data 
        mockup.sendImu( [0., 1., 0.], [0., 0., -9.81] )
        mockup.sendMocap( [0., 1., 0.], [0., 0., 0.], [1., 0., 0., 0.] )
        mockup.sendMotorSpeeds( [0., 0., 0., 100.] )

        # run system
        mockup.tick( 500 ) # 2000 Hz PID Frequency

        # get (some) outputs
        cmd = mockup.getMotorCommands()
        if not i % 2000:
            print(cmd)

    mockup.disarm() # not strictly needed, just makes the logfile terminate nicer
