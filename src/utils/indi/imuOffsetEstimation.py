import pandas as pd
import numpy as np
from scipy.optimize import least_squares

raw=pd.read_csv("LOG00209_posModeCrazySpinRecovery.BFL.csv", skiprows=140, header=0, usecols=range(105))

timeStart = raw.time.iloc[0]
timeRange = (timeStart + 33.378e6, timeStart + 33.378e6 + 0.205e6)
raw = raw[(raw.time > timeRange[0]) & (raw.time < timeRange[1])]

gyro = np.array([raw['gyroADC[0]'].to_numpy(),
    -raw['gyroADC[1]'].to_numpy(),
    -raw['gyroADC[2]'].to_numpy(),
]) * np.pi / 180.

alpha = np.array([raw['alpha[0]'].to_numpy(),
    raw['alpha[1]'].to_numpy(),
    raw['alpha[2]'].to_numpy(),
]) * np.pi / 180.

acc = np.array([raw['accSmooth[0]'].to_numpy(),
    -raw['accSmooth[1]'].to_numpy(),
    -raw['accSmooth[2]'].to_numpy(),
]) / 2048 * 9.81

def getA(gyro, alpha, acc):
    w_x, w_y, w_z = gyro
    a_x, a_y, a_z = alpha
    f_x, f_y, f_z = acc
    return np.array([
        [-w_y**2 - w_z**2,  w_x*w_y - a_z,  w_x*w_z + a_y],
        [ w_x*w_y + a_z, -w_x**2 - w_z**2,  w_y*w_z - a_x],
        [ w_x*w_z - a_y,  w_y*w_z + a_x, -w_x**2 - w_y**2]
        ])

def getb(gyro, alpha, acc):
    w_x, w_y, w_z = gyro
    a_x, a_y, a_z = alpha
    f_x, f_y, f_z = acc
    return np.array([f_x, f_y, f_z])

L = len(raw)
A = np.zeros((3*L, 3))
b = np.zeros((3*L))
for i in range(L):
    A[(3*i):(3*(i+1)), :] = getA(gyro[:, i], alpha[:, i], acc[:, i])
    b[(3*i):(3*(i+1))] = getb(gyro[:, i], alpha[:, i], acc[:, i])

xs = np.linalg.pinv(A) @ b

