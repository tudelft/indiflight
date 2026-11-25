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
    # flightModeFlags_e in src/main/fc/runtime_config.h. Written and read only
    # through IndiflightSITLMockup._setFlightModes()/_getFlightModes() (C side:
    # mockupSetFlightModes() in src/platform/SIMULATOR/mockup.c), so moving the
    # indiflight modes into their own word only changes these values and that helper.
    ANGLE_MODE       = (1 << 0)
    HORIZON_MODE     = (1 << 1)
    MAG_MODE         = (1 << 2)
    ALT_HOLD_MODE    = (1 << 3)
    POS_HOLD_MODE    = (1 << 5)
    HEADFREE_MODE    = (1 << 6)
    CHIRP_MODE       = (1 << 7)
    PASSTHRU_MODE    = (1 << 8)
    FAILSAFE_MODE    = (1 << 10)
    GPS_RESCUE_MODE  = (1 << 11)
    AUTOPILOT_MODE   = (1 << 12)
    # bits 16.. are the firmware's separate indiflightModeFlags word (see mockupSetFlightModes)
    INDI_MODE        = (1 << 16)   # INDI controller instead of PID
    POSITION_MODE    = (1 << 17)   # indiflight position control (with INDI_MODE)
    WAYPOINT_MODE    = (1 << 18)
    # not in this port: CATAPULT_MODE, LEARNER_MODE, PID_MODE (PID is the default), NN_MODE

class flightLogDisarmReason():
    DISARM_REASON_SWITCH = 4
    DISARM_REASON_SYSTEM = 255

class boxId():
    # RX boxes by configurator name (msp_box.c), see enableRxBox()
    BOXARM = "ARM"
    BOXPREARM = "PREARM"
    BOXINDI = "INDI"
    BOXPOSCTL = "POSITION"
    BOXWAYPOINTS = "WAYPOINTS"
    # not in this port: BOXTHROWTOARM

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
    def __init__(self, libfile, Nr=4, Ns=0):
        self.libfile = libfile
        self.Nr = Nr
        self.Ns = Ns
        self.lib = None
        self.lib_handle = None

        # arrays
        self.gyro = np.zeros(3, dtype=ct.c_float)
        self.acc = np.zeros(3, dtype=ct.c_float)
        self.motor_normalized_cmds = np.zeros(self.Nr, dtype=ct.c_float)
        self.motorOmega            = np.zeros(self.Nr, dtype=ct.c_float)
        self.servo_normalized_cmds = np.zeros(self.Ns, dtype=ct.c_float)
        self.servoFeedback         = np.zeros(self.Ns, dtype=ct.c_float)

        self.pos = np.zeros(3, dtype=ct.c_float)
        self.vel = np.zeros(3, dtype=ct.c_float)
        self.quat = np.zeros(4, dtype=ct.c_float)

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
        if self.lib_handle is not None:
            # close sockets/files of this instance; dlclose does not, so the
            # next init() would fail to bind the TCP serial ports
            self.lib.mockupShutdown()

            del self.armingFlags
            del self.lib

            dlclose = ct.CDLL(None).dlclose
            dlerror = ct.CDLL(None).dlerror
            dlclose.argtypes = [ct.c_void_p]
            dlclose.restype = ct.c_int
            dlerror.restype = ct.c_char_p

            if dlclose(self.lib_handle) != 0:
                print(f"Closing lib failed: {dlerror()}")

        self.lib = ct.CDLL( self.libfile )
        self.lib_handle = self.lib._handle
        self.lib.init()

        # argtypes
        self.lib.setImu.argtypes = [float_ptr, float_ptr]
        self.lib.setMotorSpeed.argtypes = [float_ptr, ct.c_int]
        self.lib.setServoAngle.argtypes = [float_ptr, ct.c_int]
        self.lib.setMocap.argtypes = [float_ptr, float_ptr, float_ptr]
        self.lib.setPosSetpoint.argtypes = [float_ptr, ct.c_float]
        self.lib.getMotorOutputCommands.argtypes = [float_ptr, ct.c_int]
        self.lib.getServoOutputCommands.argtypes = [float_ptr, ct.c_int]
        self.lib.tick.argtypes = []
        self.lib.mockupCliChar.argtypes = [ct.c_char]
        self.lib.processKey.argtypes = [ct.c_uint8]
        self.lib.disarm.argtypes = [ct.c_uint8]
        self.lib.mockupSetFlightModes.argtypes = [ct.c_uint32, ct.c_bool]
        self.lib.mockupGetFlightModes.restype = ct.c_uint32
        self.lib.mockupSetRxBoxByName.argtypes = [ct.c_char_p, ct.c_bool]
        self.lib.mockupSetRxBoxByName.restype = ct.c_bool
        self.lib.mockupArm.argtypes = [ct.c_bool]

        # most important state
        self.armingFlags = self.getVariableReference(ct.c_uint8, "armingFlags")

    def getVariableReference(self, type, variable):
        return type.in_dll(self.lib, variable)

    def load_profile(self, profile_txt):
        # upload profile, must not end with batch end\n save\n
        with open(profile_txt, 'r') as file:
            while True:
                char = file.read(1)
                if not char:
                    break
                self.lib.mockupCliChar(bytes(char, 'utf-8'))

        #todo: what if file already contains this? then we crash, i think
        for char in "batch end\nsave\n":
            self.lib.mockupCliChar(bytes(char, 'utf-8'))

        ## realod lib
        self._loadLibAndInit()

    def issueCliCommand(self, string):
        if "save" in string:
            # save requires reload of the library and re-calling init()
            raise ValueError("cannot issue save command in this function. call saveConfig()")

        for char in string:
            self.lib.mockupCliChar(bytes(char, 'utf-8'))

        self.lib.mockupCliChar(bytes("\n", 'utf-8'))

        # todo: pipe clioutput to printf for MOCKUP/(SITL) target

    def saveConfig(self):
        self.disarm()
        for char in "\nsave\n":
            self.lib.mockupCliChar(bytes(char, 'utf-8'))

        self._loadLibAndInit()

#%% sending and receiving sensor data / flight control outputs
    def sendImu(self, gyro, acc):
        self.gyro[:] = gyro
        self.acc[:] = acc
        self.lib.setImu(self.gyro, self.acc)

    def sendMotorSpeeds(self, omega): # rad/s
        n = len(omega)
        if n > self.Nr:
            raise TypeError(f"too many elements in omega. Must be at most self.Nr: {self.Nr}")

        self.motorOmega[:n] = omega
        self.lib.setMotorSpeed(self.motorOmega, n)

    def sendServoAngles(self, angles): # rad
        n = len(angles)
        if n > self.Ns:
            raise TypeError(f"too many elements in omega. Must be at most self.Ns: {self.Ns}")

        self.servoFeedback[:n] = angles
        self.lib.setServoAngle(self.servoFeedback, n)

    def sendMocap(self, pos, vel, quat):
        self.pos[:] = pos
        self.vel[:] = vel
        self.quat[:] = quat
        self.lib.setMocap(self.pos, self.vel, self.quat)

    def sendPositionSetpoint(self, pos, yaw):
        self.posSp[:] = pos
        self.yawSp = yaw
        self.lib.setPosSetpoint(self.posSp, self.yawSp)

    def getMotorCommands(self):
        self.lib.getMotorOutputCommands(self.motor_normalized_cmds, self.Nr)
        return np.array(self.motor_normalized_cmds, dtype=float)

    def getServoCommands(self):
        self.lib.getServoOutputCommands(self.servo_normalized_cmds, self.Ns)
        return np.array(self.servo_normalized_cmds, dtype=float)

    def sendKeyboard(self, key):
        if key in hid_codes.keys():
            self.lib.processKey(hid_codes[key])
        else:
            raise ValueError(f"The key {key} is not present in the hid_codes.")

#%% managing modes and states
    # the one place that touches the flight-mode word(s), see flightModeFlags
    def _setFlightModes(self, mask, enable):
        self.lib.mockupSetFlightModes(mask, enable)

    def _getFlightModes(self):
        return self.lib.mockupGetFlightModes()

    def enableFlightMode(self, mode):
        self._setFlightModes(mode, True)

    def disableFlightMode(self, mode):
        self._setFlightModes(mode, False)

    def getCurrentFlightModes(self):
        return self._getFlightModes()

    def enableRxBox(self, box):
        if not self.lib.mockupSetRxBoxByName(box.encode(), True):
            raise ValueError(f"unknown RX box {box}")

    def disableRxBox(self, box):
        if not self.lib.mockupSetRxBoxByName(box.encode(), False):
            raise ValueError(f"unknown RX box {box}")

    def getCurrentRxBoxes(self):
        raise NotImplementedError("ToDo")

    def setLogging(self, enabled):
        self.issueCliCommand(f"set blackbox_device = {'VIRTUAL' if enabled else 'NONE'}")
        print()

    def arm(self):
        self.lib.mockupArm(True)

    def disarm(self, reason = flightLogDisarmReason.DISARM_REASON_SWITCH):
        self.lib.disarm(reason) # also marks blackbox finished
        self.lib.mockupBlackboxUpdate()
        self.lib.mockupBlackboxUpdate() # call twice to flush all blackbox state changes
        self.armingFlags.value = 0 # make extra sure

#%% finally: tick the inner loop controls
    def tick(self):
        # ToDo: timing of IMU and EKF stuff is now really hardcoded
        self.lib.tick()


if __name__=="__main__":
    # init the Mockup. This deletes any existing "./eeprom.bin" and generates one with default settings
    mockup = IndiflightSITLMockup( "./obj/main/indiflight_MOCKUP.so" )

    # load settings from a profile exported from the configurator ontop of this 
    # default "./eeprom.bin". On exit, the "./eeprom.bin" is overwritten 
    # with this combined result.
    mockup.load_profile( "./src/utils/SIL.txt" )

    # set flight modes and rc switch positions
    mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | 
                            flightModeFlags.POSITION_MODE | flightModeFlags.INDI_MODE)
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
