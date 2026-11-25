# PyNDIflight simulation definition for a hexarotor (HexDemo_post_fix.txt).
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

from scipy.spatial.transform import Rotation

from PyNDIflight.crafts import MultiRotor, IMU
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
    # Hexarotor matching configs/profiles/HexDemo_post_fix.txt.
    #
    # The firmware only knows the *specific* effectiveness G1 (per unit motor
    # command, i.e. per Tmax), so the absolute mass/size below is a plausible
    # choice and everything else is derived such that the sim reproduces the
    # profile's G1/G2 (within the ~1.5% noise of the learned profile values):
    #
    #   indi_act_g1_fz    / 100  =  -Tmax / m                     [N/kg]
    #   indi_act_g1_roll  / 10   =  -y_i * Tmax / Ixx             [rad/s/s]
    #   indi_act_g1_pitch / 10   =  +x_i * Tmax / Iyy             [rad/s/s]
    #   indi_act_g1_yaw   / 10   =  +cm_i * Tmax / Izz            [rad/s/s]
    #   indi_act_g2_yaw   * 1e-5 =  Irotor / Izz                  [-]
    #
    # Geometry from the signs of g1_roll/g1_pitch (FRD body frame):
    # motor 0 points forward and the indices go counter-clockwise seen from
    # above. Spin directions alternate (sign of g1_yaw, cm<0 is CW from above):
    #
    #             x (nose)
    #             ^
    #             0 (CW)
    #    1 (CCW)  |  5 (CCW)
    #             +-----> y (right)
    #    2 (CW)      4 (CW)
    #             3 (CCW)
    #
    # Note that the MOCKUP target does not apply motor_output_reordering, so
    # rotor i here is INDI actuator i of the profile.
    G1_FZ    = 7.00                      # N/kg,   -indi_act_g1_fz / 100
    G1_ROLL  = 495.5 / np.sin(np.pi/3)   # rad/s/s, roll effectiveness of a motor at lever arm R; |g1_roll| / 10 of the 60deg arms is R*sin(60deg)
    G1_PITCH = 573.7                     # rad/s/s, average of |g1_pitch| / 10 (half-arms scaled by 2)
    G1_YAW   = 20.67                     # rad/s/s, |g1_yaw| / 10
    G2_YAW   = 1e-4                      # -,       |g2_yaw| * 1e-5
    OMEGA_MAX = 37000. / 60. * 2.*np.pi  # rad/s,   indi_act_max_rpm (also hardcoded in the learner via LEARNER_HARDCODE_MAXRPM)
    TAU = 0.025                          # s,       indi_act_time_constant_ms
    K_ESC = 0.5                          # -,       indi_act_nonlinearity / 100

    m = 0.6                              # kg, assumed
    R = 0.1                              # m,  assumed arm length (center to motor)
    Tmax = G1_FZ * m                     # N per motor
    Ixx = Tmax * R / G1_ROLL
    Iyy = Tmax * R / G1_PITCH
    Izz = Ixx + Iyy                      # thin-plate assumption
    cm = G1_YAW * Izz / Tmax             # m, drag-torque-to-thrust ratio
    Irotor = G2_YAW * Izz
    k = Tmax / OMEGA_MAX**2

    # NOTE: the profile's indi_act_hover_rpm (21600) is not consistent with
    # g1_fz = -700 and max_rpm = 37000: with these numbers the sim hovers at
    # ~17900 rpm. Hovering at 21600 rpm would require G1_FZ ~= 4.8 N/kg. The
    # hover rpm only seeds the INDI linearization (rpm feedback is enabled),
    # so the sim follows G1 instead.

    mc = MultiRotor(Nr=6)
    mc.setInertia(m=m, I=np.diag([Ixx, Iyy, Izz]))
    spin = [-1., +1., -1., +1., -1., +1.]  # sign of cm, from sign of indi_act_g1_yaw
    for i in range(6):
        psi = -i * np.pi / 3.  # 0 front, then counter-clockwise seen from above
        mc.setRotor(i, X=[R*np.cos(psi), R*np.sin(psi), 0.], k=k, cm=spin[i]*cm,
                    wmax=OMEGA_MAX, tau=TAU, kESC=K_ESC, I=Irotor)


    #%% craft interfaces
    # The firmware rotates sensor readings into the body frame with
    #   v_body = Rz(align_board_yaw) * Ry(align_board_pitch) * Rx(align_board_roll) * v_sensor
    # so that rotation is exactly qBody of the IMU (scalar-first). For
    # roll=0, pitch=-90, yaw=-120: sensor x points down, sensor z points
    # towards motor 5 (60deg right of the nose), i.e. the board is mounted vertically.
    align_roll, align_pitch, align_yaw = 0., -90., -120.  # deg, align_board_*
    rotBoard = Rotation.from_euler('ZYX', [align_yaw, align_pitch, align_roll], degrees=True)
    qx, qy, qz, qw = rotBoard.as_quat()
    qBody = [qw, qx, qy, qz]

    # acc_offset is the IMU location in the FRD body frame in mm (see learner.c)
    rImu = np.array([-22., -13., 9.]) * 1e-3

    # acc_calibration = 63,-92,5 are subtracted from the (body-aligned) raw acc
    # readings. In the MOCKUP target, acc_1G is 256, so these trims correspond to a
    # large ~0.25g bias. Give the simulated sensor exactly that bias, such that
    # the firmware sees the true specific force after its calibration.
    MOCKUP_ACC_1G = 256.
    accTrimsBody = np.array([63., -92., 5.]) * 9.81 / MOCKUP_ACC_1G
    accBiasSensor = rotBoard.inv().apply(accTrimsBody)

    imu = IMU(mc, r=rImu, qBody=qBody, accBias=accBiasSensor, accStd=0.8, gyroStd=0.08)

    mocap = Mocap(mc, args.mocap_host, args.mocap_port) if args.mocap else None
    hil = IndiflightHIL(mc, imu, device=args.hil, baud=args.hil_baud) if args.hil else None
    sil = IndiflightSITLWrapper(mc, imu, args.sil, Nr=mc.Nr, Ns=0) if args.sil else None


    #%% indiflight configuration, if software in the loop
    if sil is not None:
        sil.mockup.load_profile( args.sil_profile_txt ) if args.sil_profile_txt else None
        sil.mockup.setLogging( not args.no_sil_log )

        sil.sendMocap()
        sil.mockup.sendPositionSetpoint( [0., 0., -1.5], 0. )
        sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE | flightModeFlags.INDI_MODE)

        if args.learn:
            sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE)

        if args.catapult:
            sil.mockup.enableFlightMode(flightModeFlags.CATAPULT_MODE)
        elif args.throw:
            sil.mockup.enableRxBox(boxId.BOXTHROWTOARM)
            sil.mockup.enableRxBox(boxId.BOXARM)


    #%% initial conditions
    mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
    #mc.setPose(x=[0., 0., -0.1], q=[0.707, 0., 0., 0.707])
    mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

    sim = Sim(mc, imu, mocap, hil, sil)

    if not args.no_vis:
        print("\n\n##########################################\n")
        print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    if args.throw:
        mc.throw(height=4.,
                 wB=[2., -4., 3.], # approx body rotation in rad/s
                 vHorz=[1., -2.], # final speed in x-y-plane in m/s
                 at_time=2.5)

    #%% run loop
    dt = 0.000125 # 8kHz
    T = 1000. # seconds
    dt_rt = None if args.no_real_time else 1*dt
    start_trajectory = False
    heading = False
    for i in tqdm(range(int(T / dt)), target_looptime=dt_rt):
        if not args.throw and sim.t > 2.5:
            sil.mockup.arm() if sil else None

        if not start_trajectory and sim.t > 9. and sil is not None:
            # start trajectory tracking at 8*0.5 = 4m/s target speed
            sil.mockup.sendKeyboard('1')
            if sim.t > 10.:
                for _ in range(8):
                    sil.mockup.sendKeyboard('3')
                start_trajectory = True

        if not heading and sim.t > 10. and sil is not None:
            sil.mockup.sendKeyboard('h')
            heading = True
            # test recovery mode
            # sil.sendMocap = lambda *args: None

        sim.tick(dt)
