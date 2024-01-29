#!/usr/bin/env python3
from argparse import ArgumentParser
import numpy as np

from estimators import RLS, Signal, imuOffsetCorrection
from indiflight_log_importer import IndiflightLog

if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    N_ACT = 4

    # fitting parameters
    fc = 10. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth
    gamma = 1e0
    forgetting = 0.995 # todo: dependent on sampling rate?

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    u     = Signal(log.data['timeS'], log.data[[f'u[{i}]' for i in range(N_ACT)]])
    omega = Signal(log.data['timeS'], log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]])
    gyro  = Signal(log.data['timeS'], log.data[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
    acc   = Signal(log.data['timeS'], log.data[[f'accUnfiltered[{i}]' for i in range(3)]])
    r = np.array([-0.01275887, -0.01445745, 0.01290286]) # TODO: get this automatically from imuLocationRLS
    #r = np.zeros((3,))
    accCorrected = Signal(log.data['timeS'],
                          imuOffsetCorrection(acc.y, gyro.y, gyro.dot().y, r))

    # filter unfiltered data
    uFilt = u.filter('lowpass', order, fc)
    uFilt.setSignal(np.clip(uFilt.y, 0., 1.)) # can happen on order > 1

    omegaFilt = omega.filter('lowpass', order, fc)
    gyroFilt = gyro.filter('lowpass', order, fc)
    accFilt = acc.filter('lowpass', order, fc)
    accCorrectedFilt = accCorrected.filter('lowpass', order, fc)

    # axes
    axisNames = ['$f_x$', '$f_y$', '$f_z$',
                 'Roll', 'Pitch', 'Yaw']
    axisSymbols = ['$\Delta f_x$', '$\Delta f_y$', '$\Delta f_z$',
                   '$\Delta\dot p$', '$\Delta\dot q$', '$\Delta\dot r$']
    axisEstimators = []
    axisSelect = list(range(6)) # or [2] to get only accZ for instance 
    for axis in axisSelect:
        if axis < 3:
            # specific force
            est = RLS(N_ACT, 1, gamma=gamma, forgetting=forgetting)

            regressors = 2 * omegaFilt.y * omegaFilt.diff().y
            ys = accFilt.diff().y[:, axis]
        else:
            # dgyro, we also need rotor rate
            est = RLS(2*N_ACT, 1, gamma=gamma, forgetting=forgetting)

            regressorsForce = 2 * omegaFilt.y * omegaFilt.diff().y
            regressorsGyro  = omegaFilt.dot().diff().y
            regressors = np.concatenate((regressorsForce, regressorsGyro), axis=1)
            ys = gyroFilt.dot().diff().y[:, axis-3]

        for r, y in zip(regressors, ys):
            est.newSample(r, y)

        regressorNames = [f"$2\omega_{i}\Delta\omega_{i}$" for i in range(4)]
        parGroups = [[0,1,2,3]]
        if axis >= 3:
            regressorNames += [f"$\Delta\dot\omega_{i}$" for i in range(4)]
            parGroups.append([4,5,6,7])

        est.setName(f"{axisNames[axis]} Effectiveness Estimation")
        #est.setParameterNames( NOT SET )
        est.setRegressorNames([regressorNames])
        est.setOutputNames([f"{axisSymbols[axis]}"])

        axisEstimators.append(est)

        f = est.plotParameters(
            parGroups=parGroups,
            #yGroup=None, # defualt is fine
            timeMs=log.data['timeMs'],
            sharey=True,
            zoomy=True,
            )
        f.show()
