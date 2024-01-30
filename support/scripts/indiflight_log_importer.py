#!/usr/bin/env python3
import numpy as np
import pandas as pd
from orangebox import Parser as BFLParser

from argparse import ArgumentParser
import logging
import pickle
import os
from matplotlib import pyplot as plt
from hashlib import md5

class IndiflightLog(object):
    UNIT_FLOAT_TO_SIGNED16VB = ((127 << 6) - 1)
    RADIANS_TO_DEGREES = 57.2957796
    RADIANS_TO_DECADEGREES = 1e-1 * RADIANS_TO_DEGREES
    RADIANS_TO_DECIDEGREES = 10 * RADIANS_TO_DEGREES
    RADIANS_TO_HUNDRESOFRADIANS = 0.01
    METER_TO_MM = 1000.
    METER_TO_CM = 100.
    RAD_TO_MRAD = 1000.
    ONE_G = 9.80665
    PERCENT = 100.
    DSHOT_MIN = 158.
    DSHOT_MAX = 2048.
    CACHE_NAME = "indiflight_logs"

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

    def __init__(self, filename, timeRange=None, useCache=False, clearCache=False):
        if clearCache:
            self._clearCache()

        if useCache:
            self.raw, self.parameters = self._tryCacheLoad(filename)

        if not useCache or self.raw is None or self.parameters is None:
            # import raw data using orangebox parser
            logging.info("Parsing logfile")
            bfl = BFLParser.load(filename)

            if bfl.reader.log_count > 1:
                raise NotImplementedError("logParser not implemented for multiple\
                                           logs per BFL or BBL file. Use bbsplit\
                                           cmd line util")

            # dump data rows into pandas frame. # TODO: only import until range?
            logging.info("Importing into dataframe")
            self.raw = pd.DataFrame([frame.data for frame in bfl.frames()],
                                   columns=bfl.field_names )
            self.raw.set_index('loopIteration', inplace=True)

            # get parameters
            self.parameters = bfl.headers

            # pickle, if requested
            if useCache:
                self._storeCache()

        # crop to time range and apply scaling
        logging.info("Apply scaling and crop to range")
        self.data = self._processData(timeRange)

        # parse rc box mode change events (use LogData.modeToText to decode)
        logging.info("Convert flight modes to events")
        self.flags = self._convertModeFlagsToEvents()
        logging.info("Done")

    def _clearCache(self):
        from platformdirs import user_cache_dir

        cachedir = user_cache_dir(self.CACHE_NAME, self.CACHE_NAME)

        logging.info(f"Clearing cache at {cachedir}")
        try:
            for file_name in os.listdir(cachedir):
                os.remove(os.path.join(cachedir, file_name))
        except FileNotFoundError:
            pass

    def _tryCacheLoad(self, filename):
        from platformdirs import user_cache_dir

        cachedir = user_cache_dir(self.CACHE_NAME, self.CACHE_NAME)
        if not os.path.exists(cachedir):
            os.makedirs(cachedir)

        BUF_SIZE = 65536 # chunksize

        # idea: filename is md5 digest of both the log and this file. That 
        #       should prevent reading any stale/corrupt cache
        hash = md5()
        with open(filename, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                hash.update(data)
        with open(__file__, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                hash.update(data)

        cacheFileStem=hash.hexdigest()

        # Join the folder path with the filename to get the complete file path
        self.cacheFilePath = os.path.join(cachedir, cacheFileStem+'.pkl')
        if os.path.exists(self.cacheFilePath):
            logging.info("Using cached pickle instead of reading BFL log")
            logging.debug(f"Cache pickle location: {self.cacheFilePath}")
            with open(self.cacheFilePath, 'rb') as f:
                raw, parameters = pickle.load(f)
            return raw, parameters
        else:
            logging.info("No cached pickle found")
            return None, None

    def _storeCache(self):
        # to get here, tryCacheLoad has to have been called
        logging.info("Caching raw logs to pickle")
        logging.debug(f"Cache pickle location: {self.cacheFilePath}")
        with open(self.cacheFilePath, 'wb') as f:
            pickle.dump((self.raw, self.parameters), f)

    def _processData(self, timeRange):
        # crop relevant time range out of raw, and adjust time
        t0 = self.raw['time'].iloc[0]
        if timeRange is not None:
            data = self.raw[ ( (self.raw['time'] - t0) > timeRange[0]*1e3 )
                        & ( (self.raw['time'] - t0) <= timeRange[1]*1e3) ].copy(deep=True)
        else:
            data = self.raw.copy(deep=True)

        self.N = len(data)

        # manage time in s, ms and us
        data['time'] -= t0
        data.rename(columns={'time':'timeUs'}, inplace=True)
        timeUs = data['timeUs'].to_numpy()
        data['timeMs'] = 1e-3 * timeUs
        data['timeS'] = 1e-6 * timeUs

        # adjust column units
        highRes = 10. if self.parameters['blackbox_high_resolution'] else 1.
        for col in data.columns:
            if col == 'loopIteration':
                data[col] = data[col].astype(int)
            elif col == 'rcCommand[3]':
                data[col] -= 1000. * highRes
                data[col] /= 1000. * highRes
            elif col.startswith('rcCommand'):
                data[col] /= 500. * highRes
            elif col.startswith('gyro'):
                data[col] /= self.RADIANS_TO_DEGREES * highRes
                if col.startswith('gyroADC') and ( ("[1]" in col) or ("[2]" in col) ):
                    data[col] *= -1.
            elif col.startswith('accSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('acc'):
                data[col] *= self.ONE_G / self.parameters['acc_1G']
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
                data[col].replace("", 0, inplace=True)
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

    def resetTime(self):
        self.data['timeS'] -= self.data['timeS'].iloc[0]
        self.data['timeMs'] -= self.data['timeMs'].iloc[0]
        self.data['timeUs'] -= self.data['timeUs'].iloc[0]

    def plot(self):
        # plot some overview stuff
        f, axs = plt.subplots(3, 4, figsize=(12,9), sharex='all', sharey='row')
        axs[2, 3]._shared_axes['y'].remove(axs[2, 3])

        for i in range(4):
            line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'onboard rpm omega[{i}]')

            yyax = axs[0, i].twinx()
            line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'command u[{i}]', color='orange')
            yyax.tick_params('y', colors='orange')
            yyax.set_ylim(bottom=-0.1, top=1.1)

            lines = [line1, line2]
            labels = [line.get_label() for line in lines]

            axs[0, i].legend(lines, labels)
            axs[0, i].set_ylim(bottom=-0.1, top=1.1)
            axs[0, i].grid()
            if (i==0):
                axs[0, i].set_ylabel("Motor command/output")

        for i in range(4):
            axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'onboard drpm omega_dot[{i}]')
            axs[1, i].legend()
            axs[1, i].grid()
            if (i==0):
                axs[1, i].set_ylabel("Motor acceleration")

        for i in range(3):
            axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'onboard angular accel alpha[{i}]')
            axs[2, i].legend()
            axs[2, i].grid()
            axs[2, i].set_xlabel("Time [ms]")
            if (i==0):
                axs[2, i].set_ylabel("Angular acceleration")

        axs[2, 3].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'onboard z accel acc[2]')
        axs[2, 3].legend()
        axs[2, 3].grid()
        axs[2, 3].set_xlabel("Time [ms]")

        f.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95)

        # Maximize the window on Linux
        mgr = plt.get_current_fig_manager()
        mgr.resize(1920, 1080)

        f.show()

    def compare(self, other, other_offset=0, self_name='A', other_name='B'):
        a = self_name
        b = other_name

        f, axs = plt.subplots(3, 4, figsize=(12,9), sharex='all', sharey='row')
        axs[2, 3]._shared_axes['y'].remove(axs[2, 3])

        otherTimeMs = other.data['timeMs'] - other_offset;

        for i in range(4):
            line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'{a}: onboard rpm omega[{i}]')
            line1b, = axs[0, i].plot(otherTimeMs, other.data[f'omega[{i}]'], linestyle='--', label=f'{b}: onboard rpm omega[{i}]')

            yyax = axs[0, i].twinx()
            line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'{a}: command u[{i}]', color='green')
            line2b, = yyax.plot(otherTimeMs, other.data[f'u[{i}]'], linestyle='--', label=f'{b}: command u[{i}]', color='black')
            yyax.tick_params('y', colors='green')
            yyax.set_ylim(bottom=0)

            lines = [line1, line1b, line2, line2b]
            labels = [line.get_label() for line in lines]

            axs[0, i].legend(lines, labels)
            axs[0, i].set_ylim(bottom=0)
            axs[0, i].grid()
            if (i==0):
                axs[0, i].set_ylabel("Motor command/output")

        for i in range(4):
            axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'{a}: onboard drpm omega_dot[{i}]')
            axs[1, i].plot(otherTimeMs, other.data[f'omega_dot[{i}]'], linestyle='--', label=f'{b}: onboard drpm omega_dot[{i}]')
            axs[1, i].legend()
            axs[1, i].grid()
            if (i==0):
                axs[1, i].set_ylabel("Motor acceleration")

        for i in range(3):
            axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'{a}: onboard angular accel alpha[{i}]')
            axs[2, i].plot(otherTimeMs, other.data[f'alpha[{i}]'], linestyle='--', label=f'{b}: onboard angular accel alpha[{i}]')
            axs[2, i].legend()
            axs[2, i].grid()
            axs[2, i].set_xlabel("Time [ms]")
            if (i==0):
                axs[2, i].set_ylabel("Angular acceleration")

        axs[2, 3].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'{a}: onboard z accel acc[2]')
        axs[2, 3].plot(otherTimeMs, other.data[f'accSmooth[{i}]'], linestyle='--', label=f'{b}: onboard z accel acc[2]')
        axs[2, 3].legend()
        axs[2, 3].grid()
        axs[2, 3].set_xlabel("Time [ms]")

        f.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95)

        # Maximize the window on Linux
        mgr = plt.get_current_fig_manager()
        mgr.resize(1920, 1080)

        f.show()

if __name__=="__main__":
    # script to demonstrate how to use it
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("-v", required=False, action='count', default=0, help="verbosity (can be given up to 3 times)")
    parser.add_argument("--range","-r", required=False, nargs=2, type=int, help="integer time range to consider in ms since start of datafile")
    parser.add_argument("--no-cache","-n", required=False, action='store_true', default=False, help="Do not load from or store to raw data cache")
    parser.add_argument("--clear-cache","-c", required=False, action='store_true', default=False, help="Clear raw data cache")

    args = parser.parse_args()
    verbosity = [logging.ERROR, logging.WARNING, logging.INFO, logging.DEBUG]
    logging.basicConfig(format='%(asctime)s -- %(name)s %(levelname)s: %(message)s', level=verbosity[min(args.v, 3)])

    # import data
    log = IndiflightLog(args.datafile, args.range, not args.no_cache, args.clear_cache)

    # plot some stuff
    #log.plot()
    #plt.show()
