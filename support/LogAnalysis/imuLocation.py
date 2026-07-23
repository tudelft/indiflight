import numpy as np
from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal

#ranges = [(27850, 28350), (31470, 32000), (41550, 42150)]
#location = "/mnt/data/WorkData/BlackboxLogs/MIRROR_Meteor_1/005_acc_offset_throws.bbl"

ranges = [(6718, 7318), (8675, 8675+550), (10788, 10788+600), (12779, 12799+600)]
location = "/mnt/data/WorkData/BlackboxLogs/2025-03-18/throws/btfl_003.bbl"


#%% data importing

# filter parameters
fc = 50. # Hz. tau = 1/(2*pi*fc) if first order
order = 2 # 1 --> simple first order. 2 and up --> butterworth

# fill arrays
gyroFilt = np.empty((0, 3))
dgyroFilt = np.empty((0, 3))
accFilt = np.empty((0, 3))
for j, r in enumerate(ranges):
    log = IndiflightLog(location, timeRange=r, resetTime=True)

    gyro = Signal(log.data["timeS"], log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]] )
    acc  = Signal(log.data["timeS"], log.data[[f"accADCafterRpm[{i}]" for i in range(3)]] )

    gyroFilt = np.concatenate( (gyroFilt, gyro.filter('lowpass', order, fc).y) )
    dgyroFilt = np.concatenate( (dgyroFilt, gyro.filter('lowpass', order, fc).dot().y) )
    accFilt = np.concatenate( (accFilt, acc.filter('lowpass', order, fc).y) )

#%% setting up system and solving

# build regressor matrix
A = np.empty((gyroFilt.shape[0], 3, 3))
for i, (w, dw, a) in enumerate(zip(gyroFilt, dgyroFilt, accFilt)):
    wx, wy, wz = w
    dwx, dwy, dwz = dw
    A[i] = np.array([
        [-(wy*wy + wz*wz),    wx*wy - dwz   ,    wx*wz + dwy   ],
        [  wx*wy + dwz   ,  -(wx*wx + wz*wz),    wy*wz - dwx   ],
        [  wx*wz - dwy   ,    wy*wz + dwx   ,  -(wx*wx + wy*wy)],
        ])

# stack regressors, outputs, and solve
xhat, residuals, rank, s = np.linalg.lstsq(A.reshape(-1, 3),
                                           accFilt.reshape(-1),
                                           rcond=None)

#%% output final result
print(f"x [mm]: {(xhat*1e3).round(2)}")
