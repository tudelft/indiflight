# PyNDIflight simulation for the wind direction estimator (flight/wind_estimator.c):
# (throw and learn, or) take off to 0, 0, -4 in a constant wind, hover, then spin
# on station with the trajectory tracker's spin mode (keyboard 9, then 2/3), in
# one or both yaw directions. With --analyse, the log is decoded afterwards and
# the estimate compared with the true wind direction.
#
# Example (after `make TARGET=MOCKUP PROFILE=DarkO_ThrowToHover YES=y`):
#   python support/simulation/windSpinTailsitterSim.py \
#       --sil obj/main/indiflight_MOCKUP.so \
#       --sil-profile-txt configs/profiles/DarkO_ThrowToHover.txt \
#       --throw --no-vis --no-real-time --wind 0 3 0 --analyse
#
# Copyright 2024, 2026 Till Blaha (Delft University of Technology)
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
    parser.add_argument("--wind", required=False, nargs=3, type=float, default=[0., 3., 0.], metavar=("N", "E", "D"), help="Wind velocity in NED, m/s (where the air moves TO)")
    parser.add_argument("--spin-period", required=False, type=float, default=10., help="Seconds per revolution (rounded to steps of 100/n s, i.e. n key presses)")
    parser.add_argument("--spin-revs", required=False, type=float, default=1.5, help="Revolutions per spin direction")
    parser.add_argument("--spin-dirs", required=False, choices=["pos", "neg", "both"], default="both", help="Yaw direction(s): pos, neg, or pos then neg")
    parser.add_argument("--hover-time", required=False, type=float, default=10., help="Seconds after launch before spinning")
    parser.add_argument("--analyse", required=False, action="store_true", help="Decode the log afterwards and print the wind estimate (needs indiflight_log_tools, sil only)")
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
    tail.setWind(args.wind)
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
        sil.mockup.sendPositionSetpoint( [0., 0., -4.], 0. )
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
    t_launch = 6.5 if args.throw else 2.5 # throw time / arming time
    SPIN_KEY_RATE = 2.*np.pi/100. # rad/s per press of '3'/'2' (TT_SPIN_RATE_STEP*0.5)
    spin_presses = max(1, int(round(2.*np.pi/args.spin_period / SPIN_KEY_RATE)))
    spin_rate = spin_presses * SPIN_KEY_RATE
    spin_dirs = {"pos": [+1], "neg": [-1], "both": [+1, -1]}[args.spin_dirs]
    spin_time = args.spin_revs * 2.*np.pi / spin_rate # per direction
    dt = 0.000125 # 8kHz
    t_spin = t_launch + args.hover_time
    T = t_spin + len(spin_dirs) * spin_time + 2. # s
    dt_rt = None if args.no_real_time else 1*dt
    armed = False
    spin_phase = -1 # index into spin_dirs, -1: not started
    stopped = False
    w = np.asarray(args.wind)
    wind_from = np.degrees(np.arctan2(-w[1], -w[0])) % 360.
    print(f"Wind {w} m/s, from {wind_from:.1f} deg; spin {2.*np.pi/spin_rate:.1f} s/rev, {args.spin_revs} revs per direction {spin_dirs}")
    for i in tqdm(range(int(T / dt)), target_looptime=dt_rt):
        if args.learn and sim.t > 3.0:
            sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE) if sil else None

        if not args.throw and sim.t > 2.5 and not armed:
            sil.mockup.arm() if sil else None
            armed = True

        if sil is not None:
            phase = int((sim.t - t_spin) // spin_time) if sim.t > t_spin else -1
            if (phase != spin_phase) and (0 <= phase < len(spin_dirs)):
                d = spin_dirs[phase]
                prev = spin_dirs[phase-1] if phase > 0 else 0
                if phase == 0:
                    sil.mockup.sendKeyboard('9') # spin mode at the current position, resets the estimator
                print(f"\nt={sim.t:.1f}s: spin {'+' if d > 0 else '-'}")
                for _ in range(spin_presses * abs(d - prev)):
                    sil.mockup.sendKeyboard('3' if d > prev else '2')
                spin_phase = phase
            if not stopped and phase >= len(spin_dirs):
                print(f"\nt={sim.t:.1f}s: spin done")
                sil.mockup.sendKeyboard('4') # stop spin, the estimator holds its estimate
                stopped = True

        sim.tick(dt)

    #%% analysis
    if args.analyse and sil is not None and not args.no_sil_log:
        import glob
        sil.mockup.disarm() # flushes the log
        from indiflight_log_tools import IndiflightLog
        logfile = sorted(glob.glob("logs-mockup/LOG*.BFL"))[-1]
        d = IndiflightLog(logfile).data
        r = d.iloc[-1]
        state = int(r["wind_state"])
        est = np.degrees(r["wind_dir"]) % 360.
        err = (est - wind_from + 180.) % 360. - 180.
        print(f"\n{logfile}: wind_state {state} (valid +: {bool(state & 2)}, -: {bool(state & 4)})")
        print(f"  wind from: true {wind_from:.1f} deg, estimated {est:.1f} +- {np.degrees(r['wind_dir_std']):.1f} deg (error {err:+.1f})")
        print(f"  tilt {np.degrees(np.arcsin(min(1., np.hypot(r['wind_tilt[0]'], r['wind_tilt[1]'])))):.1f} deg, "
              f"body bias fwd {r['wind_bias[0]']:+.3f} right {r['wind_bias[1]']:+.3f}, "
              f"swept +{np.degrees(r['wind_swept[0]']):.0f} / -{np.degrees(r['wind_swept[1]']):.0f} deg")
