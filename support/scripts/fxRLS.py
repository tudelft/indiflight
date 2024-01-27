#!/usr/bin/env python3
from argparse import ArgumentParser
from matplotlib import pyplot as plt
import numpy as np

from estimators import RLS, Signal
from indiflight_log_importer import IndiflightLog

if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    N_ACT = 4

    # fitting parameters
    fc = 15. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth
    gamma = 1e0
    forgetting = 0.995 # todo: dependent on sampling rate?

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    u     = Signal(log.data['timeS'], log.data[[f'u[{i}]' for i in range(N_ACT)]])
    omega = Signal(log.data['timeS'], log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]])
    gyro  = Signal(log.data['timeS'], log.data[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
    acc   = Signal(log.data['timeS'], log.data[[f'accUnfiltered[{i}]' for i in range(3)]])

    #omegaOnboard = log.data[[f'omega[{i}]' for i in range(3)]]
    #gyroOnboard = log.data[[f'gyroADC[{i}]' for i in range(3)]]
    #alphaOnboard = log.data[[f'alpha[{i}]' for i in range(3)]]
    #accOnboard = log.data[[f'accSmooth[{i}]' for i in range(3)]]

    # filter unfiltered data
    uFilt = u.filter('lowpass', order, fc)
    uFilt.setSignal(np.clip(uFilt.y, 0., 1.)) # can happen on order > 1
    uDiffFilt = uFilt.diff()

    omegaFilt = omega.filter('lowpass', order, fc)
    omegaDiffFilt = omegaFilt.diff()
    omegaDotFilt = omegaFilt.dot()
    omegaDotDiffFilt = omegaDotFilt.diff()

    alpha = gyro.dot()
    alphaFilt = alpha.filter('lowpass', order, fc)
    alphaDiffFilt = alphaFilt.diff()

    accFilt = acc.filter('lowpass', order, fc)
    accDiffFilt = accFilt.diff()

    # axes
    axisNames = ['Roll', 'Pitch', 'Yaw',
                 'f_x', 'f_y', 'f_z']
    axisSymbols = [ '$\Delta\dot p$', '$\Delta\dot q$', '$\Delta\dot r$',
                    '$\Delta f_x$', '$\Delta f_y$', '$\Delta f_z$']
    axisEstimators = []
    for axis in range(6):
        est = RLS(2*N_ACT, gamma, forgetting)
        for i in range(len(log.data)):
            a = np.array([[ # todo: for loop somehow
                2 * omegaFilt.y[i, 0] * omegaDiffFilt.y[i, 0],
                2 * omegaFilt.y[i, 1] * omegaDiffFilt.y[i, 1],
                2 * omegaFilt.y[i, 2] * omegaDiffFilt.y[i, 2],
                2 * omegaFilt.y[i, 3] * omegaDiffFilt.y[i, 3],
                omegaDotDiffFilt.y[i, 0],
                omegaDotDiffFilt.y[i, 1],
                omegaDotDiffFilt.y[i, 2],
                omegaDotDiffFilt.y[i, 3],
            ]]).T
            y = alphaDiffFilt.y[i, axis] if axis < 3 else accDiffFilt.y[i, axis-3]
            est.newSample(a, y)

        axisEstimators.append(est)

        f = est.plotParameters(
            configs=[
                {'indices':[0, 1, 2, 3], 'regNames': [f"$2\omega_{i}\Delta\omega_{i}$" for i in range(4)]}, 
                {'indices':[4, 5, 6, 7], 'regNames': [f"$\Delta\dot\omega_{i}$" for i in range(4)]}],
            time=log.data['timeMs'],
            title=f"{axisNames[axis]} Axis",
            yLabel=f"{axisSymbols[axis]}",
            )
        f.show()
