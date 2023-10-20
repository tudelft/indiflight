"""
To be used as module. Opens and processes INFL blackbox csv data
"""

import pandas as pd
import numpy as np

class INFLImuLogProcessor():
    def __init__(self, filename):
        self.filename = filename

        # central differences kernel
        self.Kernel = [-1., 0., 1.]
        self.K = len(self.Kernel)
        assert(self.K%2) # differentiation kernel must be odd-length
        self.offset = int((self.K - 1) / 2) # offset of resulting values in convolution

        self.timeRanges = []
        self.imuFRD = pd.DataFrame(columns=["time", "gyroX", "gyroY", "gyroZ",
                                            "accX", "accY", "accZ",
                                            "alphaX", "alphaY", "alphaZ"],
                                            dtype=float) #FIXME time is not float but int
        self.N = 0

        self.raw = self._importCSV()
        self.startTime = self.raw.time[0]

    def _importCSV(self):
        # Import csv with two quirks:
        # 1. first (line_num - 1) lines are unstructured parameters/additional info
        # 2. for some reason there are extra appended columns in the data sometimes.
        with open(self.filename) as f:
            lines = f.readlines()
            line_num = -1
            for k,line in enumerate(lines):
                if line.find( "loopIteration" ) != -1:
                    line_num = k
                    col_num = len(line.split(','))
                    break

        if line_num == -1:
            raise ValueError("Cannot detect start of data in .csv")

        return pd.read_csv(self.filename,
                           skiprows=line_num,
                           header=0,
                           usecols=range(col_num))

    def add_range(self, start, end):
        """crop data range and perform differentiation of gyro"""
        startLog = self.startTime + start*1e6
        endLog = self.startTime + end*1e6

        # crop range
        part = self.raw[(self.raw.time > startLog) & (self.raw.time < endLog)]

        # get time differentials between samples (allowing from dropped 
        # samples/non-regular time stamps). But drop duplicate or non-monotonous
        # time-stamps
        deltas = np.convolve(part.time, np.flip(self.Kernel), mode='valid') * 1e-6
        dropIdx = np.where(deltas < 1e-4)[0] # 100us limit
        deltas = np.delete(deltas, dropIdx)

        dropIdx += self.offset
        part = part.drop(part.index[dropIdx])

        # check if enough datapoints left
        L = len(part)
        if L < self.K:
            # not enough datapoints left in range
            return

        # coordinate transform acc and gyro
        imu = pd.DataFrame(columns=self.imuFRD.columns)
        imu.time = part.time
        imu.gyroX = np.pi/180. * part['gyroADC[0]']
        imu.gyroY = -np.pi/180. * part['gyroADC[1]']
        imu.gyroZ = -np.pi/180. * part['gyroADC[2]']
        imu.accX = 9.81 / 2048 * part['accSmooth[0]'] # TODO: make this work for all accel types by looking at the scaling parameter in the logfile
        imu.accY = -9.81 / 2048 * part['accSmooth[1]']
        imu.accZ = -9.81 / 2048 * part['accSmooth[2]']

        # do differentiation and drop boundaries
        alphaX = np.convolve(imu.gyroX, np.flip(self.Kernel), mode='valid') / deltas
        alphaY = np.convolve(imu.gyroY, np.flip(self.Kernel), mode='valid') / deltas
        alphaZ = np.convolve(imu.gyroZ, np.flip(self.Kernel), mode='valid') / deltas
        imu.drop(imu.index[:self.offset], inplace=True)
        imu.drop(imu.index[-self.offset:], inplace=True)
        imu.alphaX = alphaX
        imu.alphaY = alphaY
        imu.alphaZ = alphaZ

        # add to df and update length and ranges
        self.imuFRD = pd.concat( (self.imuFRD, imu) )
        self.N = len(self.imuFRD)

        self.timeRanges.append( (startLog, endLog) )