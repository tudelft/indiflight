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

    # fitting parameters
    fc = 30. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth
    gamma = 1e0
    forgetting = 1.0 # todo: dependent on sampling rate?

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    log.resetTime()

    #gyro = Signal(log.data["timeS"], log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]] )
    #acc  = Signal(log.data["timeS"], log.data[[f"accUnfiltered[{i}]" for i in range(3)]] )
    gyro = Signal(log.data["timeS"], log.data[[f"gyroADC[{i}]" for i in range(3)]] )
    acc  = Signal(log.data["timeS"], log.data[[f"accSmooth[{i}]" for i in range(3)]] )

    gyroFilt = gyro.filter('lowpass', order, fc)
    accFilt  = acc.filter('lowpass', order, fc)
    gyroFilt = gyro
    accFilt  = acc

    est = RLS(3, 3, gamma=gamma, forgetting=forgetting)
    for i in range(len(log.data)):
        wx, wy, wz    = gyroFilt.y[i]
        dwx, dwy, dwz = gyroFilt.dot().y[i]
        A = np.array([
            [-(wy*wy + wz*wz),    wx*wy - dwz   ,    wx*wz + dwy   ],
            [  wx*wy + dwz   ,  -(wx*wx + wz*wz),    wy*wz - dwx   ],
            [  wx*wz - dwy   ,    wy*wz + dwx   ,  -(wx*wx + wy*wy)],
            ])
        # equally weighing all 3 axes
        est.newSample(A, accFilt.y[i])

    est.setName("IMU offset estimator")
    est.setParameterNames(["$d_x$", "$d_y$", "$d_z$"])
    # est.setRegressorNames( NOT SET )
    est.setOutputNames(["$f_x$", "$f_y$", "$f_z$"])

    f = est.plotParameters(
        parGroups=[[0], [1], [2]],
        yGroups=[[0], [1], [2]],
        timeMs=log.data['timeMs'],
        sharey=True,
    )
    f.show()
