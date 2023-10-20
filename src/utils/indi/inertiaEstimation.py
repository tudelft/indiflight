#!/usr/bin/env python
"""
Estimate (unscaled) inertia matrix from gyro data of free-tumbling
"""

import numpy as np
from argparse import ArgumentParser

from logImport import INFLImuLogProcessor

# functions
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
    parser.add_argument("-c", "--cov", action="store_true", help="Also output parameter variance-covariance matrix")
    args = parser.parse_args()

    log = INFLImuLogProcessor(args.file)

    # process time ranges
    try:
        for range in args.range:
            log.add_range(range[0], range[1])
    except TypeError:
        log.add_range(args.range[0], args.range[1])

    # iterate over time ranges to assemble A and b
    # assume Ixx == 1, so A is 5 columns, because the first column moves into b
    A = np.empty((3*log.N, 5))
    b = np.empty((3*log.N,))

    for i, (index, row) in enumerate(log.imuFRD.iterrows()):
        gyro = [row.gyroX, row.gyroY, row.gyroZ]
        alpha = [row.alphaX, row.alphaY, row.alphaZ]
        Asingle = getA(gyro, alpha)

        A[(3*i):(3*(i+1)), :] =  Asingle[:, 1:]
        b[(3*i):(3*(i+1))]    = -Asingle[:, 0] * 1.

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

    if args.cov:
        print()
        print(f"Covariance [Iyy, Izz{', Ixy, Ixz, Iyz]' if not args.principal else ']'}")
        print(C)
