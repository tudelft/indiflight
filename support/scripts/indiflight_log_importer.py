#!/usr/bin/env python3
import numpy as np
import pandas as pd
from orangebox import Parser as BFLParser

from argparse import ArgumentParser
from matplotlib import pyplot as plt

class IndiflightLog(object):
    UNIT_FLOAT_TO_SIGNED16VB = ((127 << 6) - 1)
    RADIANS_TO_DEGREES = 57.2957796
    RADIANS_TO_DECADEGREES = 1e-1 * RADIANS_TO_DEGREES
    RADIANS_TO_DECIDEGREES = 10 * RADIANS_TO_DEGREES
    RADIANS_TO_HUNDRESOFRADIANS = 0.01
    METER_TO_MM = 1000.
    METER_TO_CM = 100.
    RAD_TO_MRAD = 1000.
    ACC_TO_MSS = 9.81 / 2048.
    PERCENT = 100.
    DSHOT_MIN = 158.
    DSHOT_MAX = 2048.

    @staticmethod
    def modeToText(bits):
        # rc_modes.h:boxId_e, but only the first 32 bits i guess, because
        # blackbox.c:1200 only memcpys 4 bytes
        def single(bit):
            match(bit):
                case 0: return "ARM"
                case 1: return "ANGLE"
                case 2: return "HORIZON"
                case 3: return "MAG"
                case 4: return "HEADFREE"
                case 5: return "PASSTHRU"
                case 6: return "FAILSAFE"
                case 7: return "GPSRESCUE"
                case 8: return "VELCTL"
                case 9: return "POSCTL"
                case 10: return "CATAPULT"
                case 11: return "LEARNAFTERCATAPULT"
                case 12: return "ANTIGRAVITY"
                case 13: return "HEADADJ"
                case 14: return "CAMSTAB"
                case 15: return "BEEPERON"
                case 16: return "LEDLOW"
                case 17: return "CALIB"
                case 18: return "OSD"
                case 19: return "TELEMETRY"
                case 20: return "SERVO1"
                case 21: return "SERVO2"
                case 22: return "SERVO3"
                case 23: return "BLACKBOX"
                case 24: return "AIRMODE"
                case 25: return "3D"
                case 26: return "FPVANGLEMIX"
                case 27: return "BLACKBOXERASE"
                case 28: return "CAMERA1"
                case 29: return "CAMERA2"
                case 30: return "CAMERA3"
                case 31: return "FLIPOVERAFTERCRASH"
                case 32: return "PREARM"
                case 33: return "THROWTOARM"
                case 34: return "BEEPGPSCOUNT"
                case 35: return "VTXPITMODE"
                case 36: return "PARALYZE"
                case 37: return "USER1"
                case 38: return "USER2"
                case 39: return "USER3"
                case 40: return "USER4"
                case 41: return "PIDAUDIO"
                case 42: return "ACROTRAINER"
                case 43: return "VTXCONTROLDISABLE"
                case 44: return "LAUNCHCONTROL"
                case 45: return "MSPOVERRIDE"
                case 46: return "STICKCOMMANDDISABLE"
                case 47: return "BEEPERMUTE"
                case 48: return "READY"

        try:
            return [single(bit) for bit in bits]
        except TypeError:
            return single(bits)

    def __init__(self, filename, timeRange=None):
        # import raw data using orangebox parser
        bfl = BFLParser.load(filename)

        if bfl.reader.log_count > 1:
            raise NotImplementedError("logParser not implemented for multiple\
                                       logs per BFL or BBL file. Use bbsplit\
                                       cmd line util")

        # dump data rows into pandas frame
        self.raw = pd.DataFrame([frame.data for frame in bfl.frames()],
                               columns=bfl.field_names )
        self.raw.set_index('loopIteration', inplace=True)

        # get parameters
        self.parameters = bfl.headers

        # crop to time range and apply scaling
        self.data = self._processData(timeRange)

        # parse rc box mode change events (use LogData.modeToText to decode)
        self.flags = self._convertModeFlagsToEvents()

    def _processData(self, timeRange):
        # crop relevant time range out of raw, and adjust time
        t0 = self.raw['time'].iloc[0]
        if timeRange is not None:
            data = self.raw[ ( (self.raw['time'] - t0) > timeRange[0]*1e3 )
                        & ( (self.raw['time'] - t0) <= timeRange[1]*1e3) ].copy(deep=True)
        else:
            data = self.raw.copy(deep=True)

        # manage time in s, ms and us
        data['time'] -= t0
        data.rename(columns={'time':'timeUs'}, inplace=True)
        timeUs = data['timeUs'].to_numpy()
        data['timeMs'] = 1e-3 * timeUs
        data['timeS'] = 1e-6 * timeUs

        # get time differences
        dtime = np.zeros_like(timeUs, dtype=float)
        dtime[:-1] = np.diff(timeUs) # todo: protect against small values?
        dtime[-1] = dtime[-2] # estimate last element...
        data['dtimeUs'] = dtime.astype(int);
        data['dtimeMs'] = 1e-3 * dtime;
        data['dtimeS'] = 1e-6 * dtime;

        # adjust column units
        for col in data.columns:
            if col == 'loopIteration':
                data[col] = data[col].astype(int)
            elif col == 'rcCommand[3]':
                data[col] -= 1000.
                data[col] /= 1000.
            elif col.startswith('rcCommand'):
                data[col] /= 500.
            elif col.startswith('gyro'):
                data[col] /= self.RADIANS_TO_DEGREES
                if col.startswith('gyroADC') and ( ("[1]" in col) or ("[2]" in col) ):
                    data[col] *= -1.
            elif col.startswith('accSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('acc'):
                data[col] *= self.ACC_TO_MSS
                if ("[1]" in col) or ("[2]" in col):
                    data[col] *= -1.
            elif col.startswith('motor'):
                data[col] -= self.DSHOT_MIN
                data[col] /= (self.DSHOT_MAX - self.DSHOT_MIN)
            elif col.startswith('quat'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('alpha'):
                data[col] /= self.RADIANS_TO_DECADEGREES
            elif col.startswith('spfSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('dv'):
                data[col] /= 10.
            elif col.startswith('u['):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('u_state'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('omega_dot'):
                data[col] *= 100
            elif col.startswith('omega'):
                data[col] /= 1.
            elif col.startswith('pos') or col.startswith('extPos') or col.startswith('ekf_pos'):
                data[col] /= self.METER_TO_MM
            elif col.startswith('vel') or col.startswith('extVel') or col.startswith('ekf_vel'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('extAtt') or col.startswith('ekf_att'):
                data[col] /= 1000.
            elif col.startswith('ekf_acc_b'):
                data[col] /= 1000.
            elif col.startswith('ekf_gyro_b'):
                data[col] /= self.RADIANS_TO_DEGREES
            elif (col == "flightModeFlags") or (col == "stateFlags")\
                    or (col == "failsafePhase") or (col == "rxSignalReceived")\
                    or (col == "rxFlightChannelValid"):
                data[col] = data[col].astype(int)

        return data

    def _convertModeFlagsToEvents(self):
        # parse changes in flightModeFlags column into its own dataframe
        flags = []
        lastFlags = 0
        for index, row in self.data.iterrows():
            # for some reason, flightModeFlags in the logs doesnt correspond to
            # flightModeFlags in betaflight, but to rcModeActivationMask...
            currentFlags = int(row['flightModeFlags'])
            e, d = IndiflightLog._getModeChanges(currentFlags, lastFlags)
            lastFlags = currentFlags

            if (len(e) + len(d)) > 0:
                flags.append({'loopIteration': index, 
                                'timeUs': int(row['timeUs']),
                                "enable": e, 
                                "disable": d})

        df = pd.DataFrame(flags)
        df.set_index('loopIteration', inplace=True)
        return df

    @staticmethod
    def _getModeChanges(new, old=0):
        # find if bits have been turned on (enabled) or turned off (enabled) 
        # between two int32
        enabled = []
        disabled = []
        for i in range(32):
            bitSel = (1 << i)
            if (new & bitSel) and not (old & bitSel):
                enabled.append(i)
            elif not (new & bitSel) and (old & bitSel):
                disabled.append(i)

        return enabled, disabled

if __name__=="__main__":
    # script to demonstrate how to use it
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(0, 3_500), type=int, help="integer time range to consider in ms since start of datafile")

    args = parser.parse_args()

    # import data
    log = IndiflightLog(args.datafile, args.range)

    # plot some overview stuff
    f, axs = plt.subplots(3, 4, figsize=(12,9), sharex='all', sharey='row')
    for i in range(4):
        line1, = axs[0, i].plot(log.data['timeMs'], log.data[f'omega[{i}]'], label=f'filtered rpm omega[{i}]')

        yyax = axs[0, i].twinx()
        line2, = yyax.plot(log.data['timeMs'], log.data[f'u[{i}]'], label=f'command u[{i}]', color='orange')
        yyax.tick_params('y', colors='orange')
        yyax.set_ylim(bottom=0)

        lines = [line1, line2]
        labels = [line.get_label() for line in lines]

        axs[0, i].legend(lines, labels)
        axs[0, i].set_ylim(bottom=0)
        axs[0, i].grid()
        if (i==0):
            axs[0, i].set_ylabel("Motor command/output")

    for i in range(4):
        axs[1, i].plot(log.data['timeMs'], log.data[f'omega_dot[{i}]'], label=f'onboard drpm omega_dot[{i}]')
        axs[1, i].legend()
        axs[1, i].grid()
        if (i==0):
            axs[1, i].set_ylabel("Motor acceleration")

    for i in range(3):
        axs[2, i].plot(log.data['timeMs'], log.data[f'alpha[{i}]'], label=f'onboard angular accel alpha[{i}]')
        axs[2, i].legend()
        axs[2, i].grid()
        axs[2, i].set_xlabel("Time [ms]")
        if (i==0):
            axs[2, i].set_ylabel("Angular acceleration")

    axs[2, 3].get_shared_y_axes().remove(axs[2, 3])
    axs[2, 3].plot(log.data['timeMs'], log.data[f'accUnfiltered[{i}]'], label=f'unfiltered z accel acc[2]')
    axs[2, 3].legend()
    axs[2, 3].grid()
    axs[2, 3].set_xlabel("Time [ms]")

    f.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95)

    # Maximize the window on Linux
    mgr = plt.get_current_fig_manager()
    mgr.resize(1920, 1080)

    f.show()
    plt.show()



