#!/usr/bin/env python3

from argparse import ArgumentParser
from logParser import LogData
from estimators import RLS
from matplotlib import pyplot as plt
import numpy as np

from scipy.signal import butter, lfilter, lfilter_zi

if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    N_ACT = 4

    # load data into numpy
    log = LogData(args.datafile, args.range)
    u = log.data[[f'u[{i}]' for i in range(N_ACT)]].to_numpy().T
    omega = log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]].to_numpy().T
    alphaFilt = log.data[[f'alpha[{i}]' for i in range(3)]].to_numpy().T
    alphaDiffFilt = np.diff(alphaFilt, axis=1, prepend=alphaFilt[:, 0][:, np.newaxis])

    # filter unfiltered data
    fs = 1000.
    fc = 35.
    order = 1
    b, a = butter(order, fc, fs=fs)

    uFilt,_ = lfilter(b, a, u, zi=u[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    uFilt = np.clip(uFilt, 0., 1.) # can happen on order > 1
    uDiffFilt = np.diff(uFilt, axis=1, prepend=uFilt[:, 0][:, np.newaxis])

    omegaFilt,_ = lfilter(b, a, omega, zi=omega[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    omegaDiffFilt = np.diff(omegaFilt, axis=1, prepend=omegaFilt[:, 0][:, np.newaxis])
    omegaDotFilt = omegaDiffFilt / log.data['dtimeS'].to_numpy()
    omegaDotDiffFilt = np.diff(omegaDotFilt, axis=1, prepend=omegaDotFilt[:, 0][:, np.newaxis])

    # do estimation
    gamma = 1e7
    forgetting = 0.99 # todo: dependent on sampling rate?
    est = RLS(2*N_ACT, gamma, forgetting)

    axis = 0
    #for index, row in log.data.iterrows():
    for i in range(len(log.data)):
        #a = np.array([[ # todo: for loop somehow
        #    2 * omegaFilt[0, i] * omegaDiffFilt[0, i],
        #    2 * omegaFilt[1, i] * omegaDiffFilt[1, i],
        #    2 * omegaFilt[2, i] * omegaDiffFilt[2, i],
        #    2 * omegaFilt[3, i] * omegaDiffFilt[3, i],
        #    omegaDotDiffFilt[0, i],
        #    omegaDotDiffFilt[1, i],
        #    omegaDotDiffFilt[2, i],
        #    omegaDotDiffFilt[3, i],
        #]]).T
        #y = alphaDiffFilt[axis, i]

        a = np.array([[ # todo: for loop somehow
            omegaFilt[0, i] * omegaFilt[0, i],
            omegaFilt[1, i] * omegaFilt[1, i],
            omegaFilt[2, i] * omegaFilt[2, i],
            omegaFilt[3, i] * omegaFilt[3, i],
            omegaDotFilt[0, i],
            omegaDotFilt[1, i],
            omegaDotFilt[2, i],
            omegaDotFilt[3, i],
        ]]).T
        y = alphaFilt[axis, i]

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

    axs2[8].plot(log.data['timeMs'], alphaFilt.T)
    axs2[9].plot(log.data['timeMs'], est.y_hist)

    f.show()
    f2.show()
    plt.show()