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

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    u     = Signal(log.data['timeS'], log.data[[f'u[{i}]' for i in range(N_ACT)]])
    omega = Signal(log.data['timeS'], log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]])
    gyro  = Signal(log.data['timeS'], log.data[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
    acc   = Signal(log.data['timeS'], log.data[[f'accUnfiltered[{i}]' for i in range(3)]])

    omegaOnboard = log.data[[f'omega[{i}]' for i in range(3)]]
    gyroOnboard = log.data[[f'gyroADC[{i}]' for i in range(3)]]
    alphaOnboard = log.data[[f'alpha[{i}]' for i in range(3)]]
    accOnboard = log.data[[f'accSmooth[{i}]' for i in range(3)]]

    # filter unfiltered data
    fc = 1. # Hz
    order = 1

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
    accDiff = acc.diff()

    # do estimation
    gamma = 1e7
    forgetting = 0.99 # todo: dependent on sampling rate?
    est = RLS(2*N_ACT, gamma, forgetting)

    axis = 0
    #for index, row in log.data.iterrows():
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
        y = alphaDiffFilt.y[i, axis]

        #a = np.array([[ # todo: for loop somehow
        #    uDiffFilt[0, i],
        #    uDiffFilt[1, i],
        #    uDiffFilt[2, i],
        #    uDiffFilt[3, i],
        #    omegaDotDiffFilt[0, i],
        #    omegaDotDiffFilt[1, i],
        #    omegaDotDiffFilt[2, i],
        #    omegaDotDiffFilt[3, i],
        #]]).T
        #y = alphaDiffFilt[axis, i]

        est.newSample(a, y)

    f, axs = plt.subplots(8, 1, figsize=(10,12), sharex='all', sharey='row')
    x_hist_np = np.array(est.x_hist)
    axs[0].plot(log.data['timeMs'], x_hist_np[:, 0])
    axs[1].plot(log.data['timeMs'], x_hist_np[:, 1])
    axs[2].plot(log.data['timeMs'], x_hist_np[:, 2])
    axs[3].plot(log.data['timeMs'], x_hist_np[:, 3])
    axs[4].plot(log.data['timeMs'], x_hist_np[:, 4])
    axs[5].plot(log.data['timeMs'], x_hist_np[:, 5])
    axs[6].plot(log.data['timeMs'], x_hist_np[:, 6])
    axs[7].plot(log.data['timeMs'], x_hist_np[:, 7])

    f2, axs2 = plt.subplots(10, 1, figsize=(10,12), sharex='all', sharey='row')
    a_hist_np = np.array(est.a_hist)
    axs2[0].plot(log.data['timeMs'], a_hist_np[:, 0])
    axs2[1].plot(log.data['timeMs'], a_hist_np[:, 1])
    axs2[2].plot(log.data['timeMs'], a_hist_np[:, 2])
    axs2[3].plot(log.data['timeMs'], a_hist_np[:, 3])
    axs2[4].plot(log.data['timeMs'], a_hist_np[:, 4])
    axs2[5].plot(log.data['timeMs'], a_hist_np[:, 5])
    axs2[6].plot(log.data['timeMs'], a_hist_np[:, 6])
    axs2[7].plot(log.data['timeMs'], a_hist_np[:, 7])

    axs2[8].plot(log.data['timeMs'], alphaFilt.y)
    axs2[9].plot(log.data['timeMs'], est.y_hist)

    f.show()
    f2.show()
    plt.show()