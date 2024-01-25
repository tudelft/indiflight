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

    log = LogData(args.datafile, args.range)

    fs = 1000.
    fc = 35.
    order = 1
    b, a = butter(order, fc, fs=fs)

    N_ACT = 4
    u = log.data[[f'u[{i}]' for i in range(N_ACT)]].to_numpy().T
    omega = log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]].to_numpy().T
    omegaDot = np.diff(omega, axis=1, prepend=omega[:, 0][:, np.newaxis])
    omegaDot /= log.data['dtimeS'].to_numpy()

    uFilt,_ = lfilter(b, a, u, zi=u[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    uFilt = np.clip(uFilt, 0., 1.) # can happen on order > 1

    omegaFilt,_ = lfilter(b, a, omega, zi=omega[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    omegaDotFilt,_ = lfilter(b, a, omegaDot, zi=omegaDot[:, 0][:,np.newaxis]*lfilter_zi(b, a))

    gamma = 1e7
    forgetting = 0.999
    est = RLS(N_ACT, gamma, forgetting)

    motor = 0
    delay = 0
    for i in range(len(log.data)-delay):
        a = np.array([[
            uFilt[motor,i],
            np.sqrt(uFilt[motor,i]), 
            1.,
            -omegaDotFilt[motor,i+delay],
        ]]).T
        y = omegaFilt[motor,i]

        est.newSample(a, y)

    # recover data
    wm = est.x[0] + est.x[1]
    lam = est.x[0] / (est.x[0] + est.x[1])
    k = lam + 0.07*np.sin(lam * np.pi)
    w0 = est.x[2]
    tau = est.x[3]

    # plot
    usample = np.linspace(0, 1, 101)
    fk, axk = plt.subplots(1,1, figsize=(6, 4))
    axk.plot(usample, usample*(k*usample + (1-k)))

    f, axs = plt.subplots(4, 1, figsize=(10,12), sharex='all', sharey='row')
    axs[0].plot(log.data['timeMs'], omegaDot.T)
    axs[0].plot(log.data['timeMs'], omegaDotFilt.T)
    axs[1].plot(log.data['timeMs'], u.T)
    axs[1].plot(log.data['timeMs'], uFilt.T)
    axs[2].plot(log.data['timeMs'], omega.T)
    axs[2].plot(log.data['timeMs'], wm*(lam*u[motor] + (1-lam)*np.sqrt(u[motor])) + w0 - tau*omegaDot[motor])
    axs[3].plot(log.data['timeMs'], omegaFilt.T)
    axs[3].plot(log.data['timeMs'], wm*(lam*uFilt[motor] + (1-lam)*np.sqrt(uFilt[motor])) + w0 - tau*omegaDotFilt[motor])

    f.show()

    f, axs = plt.subplots(4, 1, figsize=(10,12), sharex='all', sharey='row')
    x_hist_np = np.array(est.x_hist)
    axs[0].plot(log.data['timeMs'], x_hist_np[:, 0])
    axs[1].plot(log.data['timeMs'], x_hist_np[:, 1])
    axs[2].plot(log.data['timeMs'], x_hist_np[:, 2])
    axs[3].plot(log.data['timeMs'], x_hist_np[:, 3])

    plt.show()

