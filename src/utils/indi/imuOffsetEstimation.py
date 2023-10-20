#!/usr/bin/env python
"""
Estimate CoG location with respect the IMU from IMU data of free-tumbling
"""

import numpy as np
from argparse import ArgumentParser

from logImport import INFLImuLogProcessor

# functions
def getA(gyro, alpha, acc):
    w_x, w_y, w_z = gyro
    a_x, a_y, a_z = alpha
    #f_x, f_y, f_z = acc
    # see imuOffsetEquations.py on how this is computed
    return np.array([
        [-w_y**2 - w_z**2,  w_x*w_y - a_z,  w_x*w_z + a_y],
        [ w_x*w_y + a_z, -w_x**2 - w_z**2,  w_y*w_z - a_x],
        [ w_x*w_z - a_y,  w_y*w_z + a_x, -w_x**2 - w_y**2]
        ])

def getb(gyro, alpha, acc):
    #w_x, w_y, w_z = gyro
    #a_x, a_y, a_z = alpha
    f_x, f_y, f_z = acc
    return np.array([f_x, f_y, f_z])

#%% start of script
if __name__=="__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("file", help="csv file")
    parser.add_argument("-r", "--range", required=True, nargs=2, action="append", type=float, metavar=("START", "END"), help="Free-tumbling time range(s) in the data to consider (seconds). Can be passed multiple times")
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
    A = np.empty( (3*log.N, 3) )
    b = np.empty( (3*log.N,) )

    for i, (index, row) in enumerate(log.imuFRD.iterrows()):
        gyro = [row.gyroX, row.gyroY, row.gyroZ]
        alpha = [row.alphaX, row.alphaY, row.alphaZ]
        acc = [row.accX, row.accY, row.accZ]
        Asingle = getA(gyro, alpha, acc)
        bsingle = getb(gyro, alpha, acc)

        A[(3*i):(3*(i+1)), :] = Asingle
        b[(3*i):(3*(i+1))]    = bsingle

    # solve system
    C = np.linalg.inv(A.T @ A)
    xs = C @ (A.T @ b)

    print(f"Logfile: {args.file}. Time range(s): {args.range}")
    print(f"Vector from IMU to CoG in FRD body coordinates:")
    print(-xs)

    if args.cov:
        print()
        print(f"Covariance:")
        print(C)
