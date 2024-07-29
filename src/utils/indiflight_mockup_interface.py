#!/usr/bin/env python3

import ctypes as ct
import numpy as np

# these "enum" classes are a big limitation currently, as they need to be 
# updated manually if things are appended/inserted in Indiflight
# similar problem with the configurator on a few things.. but yeah.. this is what it will be
class flightModeFlags():
    ANGLE_MODE       = (1 << 0)
    HORIZON_MODE     = (1 << 1)
    MAG_MODE         = (1 << 2)
    HEADFREE_MODE    = (1 << 6)
    PASSTHRU_MODE    = (1 << 8)
    FAILSAFE_MODE    = (1 << 10)
    GPS_RESCUE_MODE  = (1 << 11)
    VELOCITY_MODE    = (1 << 12)
    POSITION_MODE    = (1 << 13)
    CATAPULT_MODE    = (1 << 14)
    LEARNER_MODE     = (1 << 15)
    PID_MODE         = (1 << 16)
    NN_MODE          = (1 << 17)

class flightLogDisarmReason():
    DISARM_REASON_SWITCH = 4
    DISARM_REASON_SYSTEM = 255

class boxId():
    # not for flight modes
    BOXARM = (1 << 0)
    BOXPREARM = (1 << 34)
    BOXTHROWTOARM = (1 << 35)

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
        self.quat = np.zeros(self.N, dtype=ct.c_float)

        self.posSp = np.zeros(3, dtype=ct.c_float)
        self.yawSp = 0.

        self._loadLibAndInit()

    def __del__(self):
        print("IndiflightSITLMockup destroyed: ensure disarmed to flush blackbox logs... ", end="")
        self.disarm( reason = flightLogDisarmReason.DISARM_REASON_SYSTEM )
        print("done.")

#%% loading and config
    def _loadLibAndInit(self):
        self.lib = ct.CDLL( self.libfile )
        self.lib.init()

        # argtypes
        self.lib.setImu.argtypes = [float_ptr, float_ptr]
        self.lib.setMotorSpeed.argtypes = [float_ptr]
        self.lib.setMocap.argtypes = [float_ptr, float_ptr, float_ptr]
        self.lib.setPosSetpoint.argtypes = [float_ptr, ct.c_float]
        self.lib.getMotorOutputCommands.argtypes = [float_ptr, ct.c_int]
        self.lib.tick.argtypes = [timeUs_t]
        self.lib.processCharacterInteractive.argtypes = [ct.c_char]
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
    # init the Mockup with the "./eeprom.bin", if it exists. If not, generate one with default settings
    mockup = IndiflightSITLMockup( "./obj/main/indiflight_MOCKUP.so" )

    # load settings from a profile exported from the configurator ontop of the 
    # "./eeprom.bin", if it exists. On exit, the "./eeprom.bin" is overwritten 
    # with this combined result.
    #mockup.load_profile( "./src/utils/BTFL_cli_20240314_MATEKH743_CineRat_HoverAtt_NN.txt" )

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
