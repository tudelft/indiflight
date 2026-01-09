# PyNDIflight simulation definition for a multirotor.
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.


import numpy as np
from tqdm import tqdm
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, ArgumentTypeError
import threading
import os
import sys

from PyNDIflight.crafts import Tailsitter, TailsitterPhi, IMU
from PyNDIflight.interfaces import Mocap, IndiflightHIL, IndiflightSITLWrapper, visApp, visData
from PyNDIflight.sim import Sim

SUPPORTED_BAUDS = [
#57600, 115200, # unlikely to work 
500000, 576000, # maybe works
921600, # shown to work
#1000000, 1500000, 2000000, 3000000, # theoretically possible, untested
]

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    # required arguments for docker to work
    parser.add_argument("--sil", required=False, type=str, metavar="LIBRARY", default=None, help="Load INDIflight interface as shared library.")
    parser.add_argument("--sil-profile-txt", required=False, type=str, metavar="PROFILE.txt", help="Import these profile settings into the SIL")
    # further arguments
    parser.add_argument("--hil", required=False, type=str, metavar="DEVICE", help="Use INDIflight hardware interface. Use either --hil or --sil")
    parser.add_argument("--hil-baud", required=False, choices=SUPPORTED_BAUDS, type=int, default=921600, help="HIL baudrate ")
    parser.add_argument("--mocap", required=False, nargs=2, metavar=("IP", "PORT"), help="Stream mocap UDP packets to this IP/hostname and port")
    parser.add_argument("--no-vis", required=False, action="store_true", help="Do not launch visualization webserver")
    parser.add_argument("--no-real-time", required=False, action="store_true", help="Run as fast as possible")
    parser.add_argument("--no-sil-log", required=False, action="store_true", help="DO NOT write Indiflight logs into ./logs-mockup")  # enabled by default now
    parser.add_argument("--catapult", required=False, action="store_true", help="Use catapult (sil-only)")
    parser.add_argument("--throw", required=False, action="store_true", help="Use throwing")
    parser.add_argument("--learn", required=False, action="store_true", help="Learn after throw/catapult (sil-only)")
    args = parser.parse_args()

    if args.sil and args.hil is not None:
        raise ArgumentTypeError("Cannot have both --hil and --sil")

    if args.hil and args.no_real_time:
        raise ArgumentTypeError("--hil must not be used with --no-real-time")

    if not args.sil and args.catapult:
        raise ArgumentTypeError("--catapult only works for SIL")

    if not args.sil and args.learn:
        raise ArgumentTypeError("--learn only works for SIL")

    if args.catapult and args.throw:
        raise ArgumentTypeError("--catapult and --throw cannot be selected at the same time")

    if args.learn and not args.catapult and not args.throw:
        raise ArgumentTypeError("--learn requires either --catapult or --throw")

    if args.sil:
        if not os.path.isfile(args.sil):
            raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")

        import os
        sys.path.append(os.path.join((os.path.dirname(args.sil)),
                                     "../../src/utils"))
        from indiflight_mockup_interface import flightModeFlags, boxId

    if args.mocap is not None:
        args.mocap_host = args.mocap[0]
        try:
            args.mocap_port = int(args.mocap[1])
        except ValueError:
            raise ArgumentTypeError ("PORT must be integer")


    #%% Generate craft
    # tail = Tailsitter()
    # tail.setInertia(m=0.5, I=np.diag([6e-3, 2e-3, 6.5e-3]))
    # tail.setRotor(0, X=[-0.01, -0.13, -0.07], k=1.e-6, cm=-0.005, wmax=3000., tau=0.03, kESC=0.5, I=2e-6) # RR
    # tail.setRotor(1, X=[-0.01, +0.13, -0.07], k=1.e-6, cm=+0.005, wmax=3000., tau=0.03, kESC=0.5, I=2e-6) # FR

    tail = TailsitterPhi()
    tail.setInertia(m=0.564, I=np.diag([5.73e-3, 1.35e-3, 5.43e-3]))

    # with d0 only for yaw
    # tail.setRotor(0, X=[5.831e-03, -1.269e-01, -7.000e-02], ax=[2.661e-02, 0.000e+00, -9.996e-01], k=1.238e-6, cm=-6.639e-3, wmax=3000., tau=0.03, kESC=0.5, I=3.338e-6) # RR
    # tail.setRotor(1, X=[5.831e-03, +1.269e-01, -7.000e-02], ax=[2.661e-02, 0.000e+00, -9.996e-01], k=1.238e-6, cm=+6.639e-3, wmax=3000., tau=0.03, kESC=0.5, I=3.338e-6) # FR

    # phi theory coefficients
    # phi = 5.024e-03
    # Phi = np.array([
    #     [+2.735e-01,          0, +3.644e-02,          0, -1.951e-04,          0],
    #     [         0, +3.104e-02,          0, +1.414e-03,          0, -4.235e-03],
    #     [+3.644e-02,          0, +3.924e-02,          0, +8.971e-05,          0],
    #     [         0, +1.414e-03,          0, +8.162e-04,          0, +7.459e-04],
    #     [-1.951e-04,          0, +8.971e-05,          0, +8.788e-04,          0],
    #     [         0, -4.235e-03,          0, +7.459e-04,          0, +3.201e-03],
    # ], dtype=np.float32)

    # # elevon contribution
    # d0 = np.array([-1.042e-01, +1.022e-01], dtype=np.float32)
    # cd = np.array([-6.258e-07,          0,          0,          0, -2.381e-08, -7.286e-08], dtype=np.float32)
    # cdd = np.array([         0,          0,          0,          0, -1.958e-03,          0], dtype=np.float32)
    # cddd = np.array([         0,          0,          0,          0,          0,          0], dtype=np.float32)

    # with d0 for all, but cmww=0, seems to be better
    phi = 3.297e-1
    Phi = np.array([
        [+3.106e-01,          0, +4.148e-02,          0, -1.613e-04,          0],
        [         0, +3.603e-02,          0, +1.355e-03,          0, -3.538e-03],
        [+4.148e-02,          0, +4.465e-02,          0, +1.951e-04,          0],
        [         0, +1.355e-03,          0, +8.290e-04,          0, +7.494e-04],
        [-1.613e-04,          0, +1.951e-04,          0, +4.093e-04,          0],
        [         0, -3.538e-03,          0, +7.494e-04,          0, +2.790e-03],
    ], dtype=np.float32)

    cd = np.array([-7.032e-07,          0,          0,          0, -1.835e-08, -6.047e-08], dtype=np.float32)
    cdd = np.array([         0,          0,          0,          0, -1.412e-03,          0], dtype=np.float32)
    cddd = np.array([         0,          0,          0,          0, -2.201e-05,          0], dtype=np.float32)
    d0 = np.array([-3.618e-01, -1.540e-01], dtype=np.float32)

    tail.setPhiModel(phi, Phi)
    tail.setElevonModel(cd, cdd, cddd, d0)

    tail.setRotor(0, X=[1.104e-02, -1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=1.413e-06, cm=-6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # RR
    tail.setRotor(1, X=[1.104e-02, +1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=1.413e-06, cm=+6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # FR


    inertia_ratios = np.array([(tail.I[2,2] - tail.I[1,1]) / tail.I[0,0],
                               (tail.I[0,0] - tail.I[2,2]) / tail.I[1,1],
                               (tail.I[1,1] - tail.I[0,0]) / tail.I[2,2]])
    print(f"Craft inertia ratios: {inertia_ratios}") # [ 0.74999994 -0.25000003 -0.6153846]


    #%% craft interfaces
    imu = IMU(tail, r=[-3.580e-02, -3.827e-03, +5.630e-03], qBody=[0.707, 0., 0.707, 0.], accStd=0.08, gyroStd=0.08)
    # imu = IMU(tail, r=[-3.580e-02, -3.827e-03, +5.630e-03], qBody=[0.707, 0., 0.707, 0.], accStd=0.0, gyroStd=0.0)
    # imu = IMU(tail, r=[0, 0, 0], qBody=[0.707, 0., 0.707, 0.], accStd=0.0, gyroStd=0.0)

    mocap = Mocap(tail, args.mocap_host, args.mocap_port) if args.mocap else None
    hil = IndiflightHIL(tail, imu, device=args.hil, baud=args.hil_baud) if args.hil else None
    sil = IndiflightSITLWrapper(tail, imu, args.sil, Nr=tail.Nr, Ns=tail.Ns) if args.sil else None


    #%% indiflight configuration, if software in the loop
    if sil is not None:
        sil.mockup.load_profile( args.sil_profile_txt ) if args.sil_profile_txt else None
        sil.mockup.setLogging( not args.no_sil_log )
        sil.mockup.initUros()

        sil.sendMocap()
        sil.mockup.sendPositionSetpoint( [0., 0., -2.], 0. )
        sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE)

        if args.catapult:
            sil.mockup.enableFlightMode(flightModeFlags.CATAPULT_MODE)
        elif args.throw:
            sil.mockup.enableRxBox(boxId.BOXTHROWTOARM)
            sil.mockup.enableRxBox(boxId.BOXARM)


    #%% initial conditions
    tail.setPose(x=[0, -3.5, -0.1], q=[0.707, 0., 0., 0.707])
    # tail.setPose(x=[0., 0., -0.1], q=[0.707, 0., 0.707, 0.])
    # tail.setPose(x=[0., 0., -20.1], q=[0, -0.707, 0., 0.707])
    # tail.setPose(x=[0., 0., -0.1], q=[0, 0, 0, 1.])
    tail.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

    sim = Sim(tail, imu, mocap, hil, sil)

    if not args.no_vis:
        print("\n\n##########################################\n")
        print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    if args.throw:
        tail.throw(height=6.,
                 #wB=[1., -1., 2.], # approx body rotation in rad/s
                 #vHorz=[3., 0.], # final speed in x-y-plane in m/s
                 wB=[-0., -4., 0.], # approx body rotation in rad/s
                 vHorz=[0., 2.], # final speed in x-y-plane in m/s
                 #wB=[2., -4., 3.], # approx body rotation in rad/s
                 #vHorz=[0., 0.], # final speed in x-y-plane in m/s
                 at_time=6.5)

    #%% run loop
    dt = 0.000125 # 8kHz
    T = 30. # seconds
    dt_rt = None if args.no_real_time else 1*dt
    armed = False
    start_trajectory = False
    heading = False
    speedup = False
    for i in tqdm(range(int(T / dt)), target_looptime=dt_rt):
        if args.learn and sim.t > 3.0:
            sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE) if sil else None
            sil.mockup.sendPositionSetpoint( [0., 0., -2.], 0. )

        if not args.throw and sim.t > 2.5 and not armed:
            sil.mockup.arm() if sil else None
            armed = True

        if not start_trajectory and sim.t > 12. and sil is not None:
            # start trajectory tracking at 8*0.5 = 4m/s target speed
            sil.mockup.sendKeyboard('1')
            # sil.mockup.sendPositionSetpoint( [4., 0., -1.5], 0. )
            if sim.t > 14.:
                sil.mockup.sendKeyboard('h')
                for _ in range(6):
                    sil.mockup.sendKeyboard('3')
                start_trajectory = True

        # if not heading and sim.t > 17. and sil is not None:
        #     sil.mockup.sendKeyboard('h')
        #     heading = True

        # if not speedup and sim.t > 20. and sil is not None:
        #     speedup = True
        #     for _ in range(6):
        #         sil.mockup.sendKeyboard('3')
        #     # test recovery mode
        #     # sil.sendMocap = lambda *args: None

        # if sim.t > 12. and sil is not None:
        #     # sil.mockup.sendPositionSetpoint( [20., 0., -1.5], 0. )
        #     sil.mockup.sendPositionSetpoint( [2., 2., -1.5], 0. )
        # if sim.t > 10. and sil is not None:
        #     sil.mockup.sendPositionSetpoint( [-1., 0., -2.], 0. )
        # if sim.t > 10.6 and sil is not None:
        #     sil.mockup.sendPositionSetpoint( [1., 0., -2.], 0. )

        sim.tick(dt)
