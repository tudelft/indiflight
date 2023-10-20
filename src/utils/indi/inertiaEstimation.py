#!/usr/bin/env python
"""
Estimate (unscaled) inertia matrix from gyro data of free-tumbling
"""

import pandas as pd
import numpy as np
from itertools import chain
from argparse import ArgumentParser

# functions. TODO: turn this into a class?
def importINFLCsv(filename):
    # Import csv with two quirks:
    # 1. first (line_num - 1) lines are unstructured parameters/additional info
    # 2. for some reason there are extra appended columns in the data sometimes.
    with open(filename) as f:
        lines = f.readlines()
        line_num = -1
        for k,line in enumerate(lines):
            if line.find( "loopIteration" ) != -1:
                line_num = k
                col_num = len(line.split(','))
                break

    if line_num == -1:
        raise ValueError("Cannot detect start of data in .csv")

    raw = pd.read_csv(args.file, skiprows=line_num, header=0, usecols=range(col_num))
    return raw

def processData(df, timeRange):
    """crop data range and perform differentiation of gyro"""

    # crop range
    part = df[(df.time > timeRange[0]) & (df.time < timeRange[1])]

    # prepare differentiation kernel and time deltas
    Kernel = np.array([-1., 0., 1.]) # central differences
    K = len(Kernel)
    assert(K%2) # differentiation kernel must be odd-length
    offset = int((K - 1) / 2) # offset of resulting values in convolution

    # get time differentials between samples (allowing from dropped 
    # samples/non-regular time stamps). But drop duplicate or non-monotonous
    # time-stamps
    deltas = np.convolve(part.time, np.flip(Kernel), mode='valid') * 1e-6
    dropIdx = np.where(deltas < 1e-4)[0] # 100us limit
    deltas = np.delete(deltas, dropIdx)

    dropIdx += offset
    part = part.drop(part.index[dropIdx])

    # check if enough datapoints left
    L = len(part)
    if L < K:
        # not enough datapoints left in range
        return np.zeros((0,3)), np.zeros((0,3)), np.array((0,3)), L

    # get gyro and acceleration, and flip their coord-sys to FRD
    gyro = np.pi/180 * part[['gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]']].to_numpy(dtype=float)
    gyro[:, 1] *= -1.
    gyro[:, 2] *= -1.
    acc = part[['accSmooth[0]', 'accSmooth[1]', 'accSmooth[2]']].to_numpy(dtype=float)
    acc[:, 1] *= -1.
    acc[:, 2] *= -1.

    # finally, get derivative of gyro
    #alpha = np.pi/180 * part[['alpha[0]', 'alpha[1]', 'alpha[2]']].to_numpy(dtype=float)
    alphaX = np.convolve(gyro[:, 0], np.flip(Kernel), mode='valid') / deltas
    alphaY = np.convolve(gyro[:, 1], np.flip(Kernel), mode='valid') / deltas
    alphaZ = np.convolve(gyro[:, 2], np.flip(Kernel), mode='valid') / deltas
    alpha = np.array((alphaX, alphaY, alphaZ)).T

    # drop boundaries to have same length as alpha
    boundaryIdx = list(chain(range(offset), range(L-offset, L)))
    gyro = np.delete(gyro, boundaryIdx, axis=0)
    acc = np.delete(acc, boundaryIdx, axis=0)

    return gyro, alpha, acc, L - K + 1

def getA(gyro, alpha):
    w_x, w_y, w_z = gyro
    a_x, a_y, a_z = alpha
    # see inertiaEquations.py on how this is computed
    return np.array([
        [      a_x, -w_y*w_z,  w_y*w_z, -w_x*w_z + a_y,  w_x*w_y + a_z,  w_y**2 - w_z**2],
        [  w_x*w_z,      a_y, -w_x*w_z,  w_y*w_z + a_x, -w_x**2 + w_z**2, -w_x*w_y + a_z],
        [ -w_x*w_y,  w_x*w_y,      a_z,  w_x**2 - w_y**2, -w_y*w_z + a_x,  w_x*w_z + a_y],
        ])

def x2I(x, Ixx):
    Iyy = x[0]
    Izz = x[1]
    Ixy = x[2]
    Ixz = x[3]
    Iyz = x[4]
    return Ixx * np.matrix([
        [1.,  Ixy, Ixz],
        [Ixy, Iyy, Iyz],
        [Ixz, Iyz, Izz],
    ])

def I2x(I):
    Ixx = I[0,0]
    Iyy = I[1,1] / Ixx
    Izz = I[2,2] / Ixx
    Ixy = I[0,1] / Ixx
    Ixz = I[0,2] / Ixx
    Iyz = I[1,2] / Ixx
    return np.array([Iyy, Izz, Ixy, Ixz, Iyz])


#%% start of script
if __name__=="__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("file", help="csv file")
    parser.add_argument("-r", "--range", required=True, nargs=2, action="append", type=float, metavar=("START", "END"), help="Free-tumbling time range(s) in the data to consider (seconds). Can be passed multiple times")
    parser.add_argument("-p", "--principal", action="store_true", help="Assume IMU is aligned with principal axes so products of inertia are forced 0")
    args = parser.parse_args()

    raw = importINFLCsv(args.file)

    # process time ranges
    timeStart = raw.time.iloc[0]
    try:
        timeRanges = [(timeStart + x[0]*1e6, timeStart + x[1]*1e6) for x in args.range]
    except TypeError:
        timeRanges = [(timeStart + args.range[0]*1e6, timeStart + args.range[1]*1e6)]


    # iterate over tiem ranges to assemble A and b
    # assume Ixx == 1, so A is 5 columns, because the first column moves into b
    A = np.empty((0, 5))
    b = np.empty((0,))
    for timeRange in timeRanges:
        gyro, alpha, acc, L = processData(raw, timeRange)
        if L == 0:
            continue

        Asub = np.empty((L*3, 5))
        bsub = np.empty((L*3,))

        for i in range(L):
            Asingle = getA(gyro[i, :], alpha[i, :])
            Asub[(3*i):(3*(i+1)), :] =  Asingle[:, 1:]
            bsub[(3*i):(3*(i+1))]    = -Asingle[:, 0] * 1.

        A = np.concatenate((A, Asub), axis=0)
        b = np.concatenate((b, bsub))


    # solve system
    xs = np.zeros((5,))
    if args.principal:
        # solve reduced system assuming products of inertia are 0
        Ared = A[:, :2]
        C = np.linalg.inv(Ared.T @ Ared)

        xs[:2] = C[:2, :2] @ (Ared.T @ b)
    else:
        # solve full system
        C = np.linalg.inv(A.T @ A)
        xs = C @ (A.T @ b)


    #Iest = np.array([
    #    [0.00072, 0., 0.],
    #    [0., 0.00077, 0.],
    #    [0., 0., 0.00089],
    #    ])
    #
    #print("Old measurement")
    #print(Iest)
    #print("Old measurement -- Ixx normalized")
    #print(Iest/Iest[0,0])
    #print()

    print(f"Logfile: {args.file}. Time range(s): {args.range}")
    print(x2I(xs, 1.))
