#!/usr/bin/env python3

import ctypes as ct
import os
import numpy as np

# ctypes stuff
float_ptr = np.ctypeslib.ndpointer(dtype=ct.c_float, ndim=1)
timeUs_t = ct.c_uint32

class IndiflightSITLMockup(object):
    def __init__(self, libfile, profile_txt=None, N=4):
        self.libfile = libfile
        self.N = N

        # delete eeprom and init to create a clean eeprom
        os.remove('eeprom.bin')
        self._loadLibAndInit()

        if profile_txt is not None:
            self._load_profile(profile_txt)

        # arrays
        self.gyro = np.zeros(3, dtype=ct.c_float)
        self.acc = np.zeros(3, dtype=ct.c_float)
        self.motorOmega = np.zeros(self.N, dtype=ct.c_float)
        self.motorCommands = np.zeros(self.N, dtype=ct.c_float)

        self.pos = np.zeros(3, dtype=ct.c_float)
        self.vel = np.zeros(3, dtype=ct.c_float)
        self.quat = np.zeros(self.N, dtype=ct.c_float)

        self.posSp = np.zeros(3, dtype=ct.c_float)
        self.yawSp = 0.

        # argtypes
        self.lib.setImu.argtypes = [float_ptr, float_ptr]
        self.lib.setMotorSpeed.argtypes = [float_ptr]
        self.lib.setMocap.argtypes = [float_ptr, float_ptr, float_ptr]
        self.lib.setPosSetpoint.argtypes = [float_ptr, ct.c_float]
        self.lib.getMotorOutputCommands.argtypes = [float_ptr, ct.c_int]
        self.lib.tick.argtypes = [timeUs_t]

    def _load_profile(self, profile_txt):
        # upload profile, must not end with batch end\n save\n
        self.lib.processCharacterInteractive.argtypes = [ct.c_char]
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

    def sendImu(self, gyro, acc):
        self.gyro[:] = gyro
        self.acc[:] = acc
        self.lib.setImu(self.gyro, self.acc)

    def sendMotorSpeeds(self, omega):
        n = len(omega)
        if n > self.N:
            raise TypeError("too many elements in omega. Must be at most self.N")

        self.motorOmega[:n] = omega
        self.lib.setMotorSpeed(self.motorOmega)

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
        self.lib.getMotorOutputCommands(self.motorCommands, self.N)
        return np.array(self.motorCommands, dtype=float)

    def tick(self, currentTimeUs):
        self.lib.tick( currentTimeUs )

    def _loadLibAndInit(self):
        self.lib = ct.CDLL( self.libfile )
        self.lib.init()

    def getVariableReference(self, type, variable):
        return type.in_dll(self.lib, variable)

    def issueCliCommand(self, string):
        if "save" in string:
            # save requires reload of the library and re-calling init()
            raise ValueError("cannot issue save command in this function. call saveConfig()")

        for char in string:
            self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        # todo: pipe clioutput to printf for MOCKUP/(SITL) target

    def saveConfig(self):
        for char in "\nsave\n":
            self.lib.processCharacterInteractive(bytes(char, 'utf-8'))

        self._loadLibAndInit()


if __name__=="__main__":
    mockup = IndiflightSITLMockup(
        "./obj/main/indiflight_MOCKUP.so",
        "./src/utils/BTFL_cli_20240314_MATEKH743_CineRat_HoverAtt_NN.txt",
        )

    # get references to a few global variables

    # send data 
    mockup.sendImu( [0., 1., 0.], [0., 0., -9.81] )
    mockup.sendMocap( [0., 1., 0.], [0., 0., 0.], [1., 0., 0., 0.] )
    mockup.sendMotorSpeeds( [0., 0., 0., 100.] )

    # send setpoint
    mockup.sendPositionSetpoint( [0., 1., -1.], np.pi )

    # set armed and correct flight modes
    armingFlags = mockup.getVariableReference(ct.c_int, "armingFlags")
    armingFlags.value = 1 # armed
    flightModeFlags = mockup.getVariableReference(ct.c_int, "flightModeFlags")
    flightModeFlags.value = (1 << 0) | (1 << 13)  # ANGLE_MODE and POSITION_MODE

    # run system
    currentTimeUs = 0 # flight controller time in microseconds
    mockup.tick( currentTimeUs )

    # get outputs
    cmd = mockup.getMotorCommands()
    print(cmd)
