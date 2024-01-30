#!/usr/bin/env python3
from argparse import ArgumentParser
import logging
import numpy as np

from estimators import RLS, Signal
from indiflight_log_importer import IndiflightLog

if __name__=="__main__":
    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=logging.INFO,
        )

    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    N_ACT = 4

    # fitting parameters
    fc = 40. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth
    gamma = 1e7
    forgetting = 0.995 # todo: dependent on sampling rate?

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    u     = Signal(log.data['timeS'], log.data[[f'u[{i}]' for i in range(N_ACT)]])
    omega = Signal(log.data['timeS'], log.data[[f'omegaUnfiltered[{i}]' for i in range(N_ACT)]])

    # filter unfiltered data
    uFilt = u.filter('lowpass', order, fc)
    uFilt.setSignal(np.clip(uFilt.y, 0., 1.)) # can happen on order > 1

    omegaFilt = omega.filter('lowpass', order, fc)

    motors = []
    for motor in range(N_ACT):
        est = RLS(N_ACT, 1, gamma=gamma, forgetting=forgetting)
        for i in range(len(log.data)):
            a = np.array([[
                uFilt.y[i, motor],
                np.sqrt(uFilt.y[i, motor]), 
                1.,
                -omegaFilt.dot().y[i, motor],
            ]])
            y = omegaFilt.y[i, motor]
            est.newSample(a, y)

        motorDict = {}
        motorDict['est'] = est

        # recover data
        motorDict['wm'] = est.x[0] + est.x[1]
        motorDict['lam'] = est.x[0] / (est.x[0] + est.x[1])
        motorDict['k'] = motorDict['lam'] + 0.07*np.sin(motorDict['lam'] * np.pi)
        motorDict['w0'] = est.x[2]
        motorDict['tau'] = est.x[3]

        motors.append(motorDict)

        est.setName(f"Motor {motor}")
        est.setParameterNames(["$a$", "$b$", "$\omega_0$", "$\\tau$"])
        est.setRegressorNames([["$u$", "$\sqrt{u}$", "unity", "$-\dot\omega$"]])
        est.setOutputNames([f"$\omega_{motor}$"])

        f = est.plotParameters(
            parGroups=[[0,1], [2], [3]],
            #yGroup=None, #default is fine
            timeMs=log.data['timeMs'],
            sharey=False,
            zoomy=False,
            )
        f.show()
