#!/usr/bin/env python3

from argparse import ArgumentParser
from logParser import LogData
from matplotlib import pyplot as plt

from scipy.signal import butter, lfilter, lfilter_zi

import numpy as np

class RLS(object):
    def __init__(self, n, gamma=1e8, forgetting=0.995):
        self.x = np.zeros((n,1))
        self.P = gamma * np.eye(len(self.x))
        self.forgetting = forgetting
        self.k_hist = []
        self.e_hist = []
        self.x_hist = []
        self.P_hist = []

    def newData(self, a, y, disable=None):
        # regressors a and observation y
        k = ( self.P @ a ) / ( self.forgetting + a.T @ self.P @ a )
        self.P = (1/self.forgetting) * (self.P - k @ a.T @ self.P)

        e = y - self.x.T @ a
        if (disable==None):
            self.x += k * e
        else:
            self.x[~disable] += k[~disable] * e

        self.k_hist.append(k)
        self.e_hist.append(e)
        self.x_hist.append(self.x)
        self.P_hist.append(self.P)


if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")

    args = parser.parse_args()

    log = LogData(args.datafile, args.range)

    fs = 1000.
    fc = 40.
    order = 1
    b, a = butter(order, fc, fs=fs)

    N_ACT = 4
    u = log.data[[f'u[{i}]' for i in range(N_ACT)]].to_numpy().T
    omega = log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]].to_numpy().T
    omegaDot = np.diff(omega, axis=1, append=omega[:, -1][:, np.newaxis])
    omegaDot /= log.data['dtimeS'].to_numpy()

    uFilt,_ = lfilter(b, a, u, zi=u[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    uFilt = np.clip(uFilt, 0., 1.) # can happen on order > 1

    omegaFilt,_ = lfilter(b, a, omega, zi=omega[:, 0][:,np.newaxis]*lfilter_zi(b, a))
    omegaDotFilt,_ = lfilter(b, a, omegaDot, zi=omegaDot[:, 0][:,np.newaxis]*lfilter_zi(b, a))

    x0 = np.array([[0., 0., 0., 0.]]).T
    gamma = 1e7
    P0 = gamma * np.eye(len(x0))

    x = x0
    P = P0
    forgetting = 0.999
    omegaDotScale = 1. #1e-5

    motor = 0

    delay = 0

    for i in range(len(log.data)-delay):
        a = np.array([[ uFilt[motor,i], np.sqrt(uFilt[motor,i]), 1., -omegaDotScale*omegaDotFilt[motor,i+delay] ]]).T
        y = omegaFilt[motor,i]
        #if (uFilt[motor, i] - uFilt[motor, i-1]) > 0.0:
        #    continue
        k = ( P @ a ) / ( forgetting + a.T @ P @ a )
        e = y - x.T @ a
        x += k * e
        #x[:2] += k[:2] * e
        P = (1/forgetting) * (P - k @ a.T @ P)

    # recover data
    wm = x[0] + x[1]
    lam = x[0] / (x[0] + x[1])
    k = lam + 0.07*np.sin(lam * np.pi)
    w0 = x[2]
    tau = x[3] / omegaDotScale

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


