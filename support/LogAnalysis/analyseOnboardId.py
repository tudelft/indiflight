from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt
import matplotlib
import re

plt.close('all')
matplotlib.use('tkagg') 

from pyFlightPlotter import local_rc, Craft3D
from indiflightPlotter import IndiflightPlotter, \
    IndiflightMotorSysIdPlotter, \
    IndiflightServoSysIdPlotter, \
    IndiflightViewport, \
    IndiflightIndividualSysIdPlotter, \
    IndiflightMoments, \
    IndiflightEffectiveness
from pyFlightPlotter import Quadrotor, Tailsitter, BlittedCursor, VideoViewport

# set local_rc
plt.rcParams.update(local_rc)

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import numpy as np

parser = ArgumentParser(description="Analyse onboard ID data from a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")
parser.add_argument("--mp4", required=False, type=str,
                    help="Optional path to an MP4 file to synchronize with the data timeline.")
parser.add_argument("--video-offset", required=False, type=float, default=0.0,
                    help="Time offset [s] applied as video_time = data_time + offset.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

class Hexarotor(Craft3D):
    def __init__(self, l=0.2, R=0.1):
        body = np.array([
            [    l,   -l/4,     0.],
            [    l,   +l/4,     0.],
            [    l+l/4,   0,     0.],
            [    l,   -l/4,     0.],
        ])
        geometry = [body]
        super().__init__(body_geometry=geometry)

        rotor_radius = l/2
        rotor_positions = np.array([
            [rotor_radius, 0.0, 0.0],
            [0.5 * rotor_radius,  0.8660254037844386 * rotor_radius, 0.0],
            [-0.5 * rotor_radius, 0.8660254037844386 * rotor_radius, 0.0],
            [-rotor_radius, 0.0, 0.0],
            [-0.5 * rotor_radius, -0.8660254037844386 * rotor_radius, 0.0],
            [0.5 * rotor_radius, -0.8660254037844386 * rotor_radius, 0.0],
        ])

        for rotor_position in rotor_positions:
            self.addRotor(xyz=rotor_position, axis=[0, 0, -1], R=R)


def _sorted_indexed_columns(columns, pattern):
    indexed = []
    for col in columns:
        m = re.match(pattern, col)
        if m:
            indexed.append((int(m.group(1)), col))
    indexed.sort(key=lambda x: x[0])
    return indexed


def _find_first_motor_command_time(data):
    columns = data.columns
    motor_cols = _sorted_indexed_columns(columns, r'^motor\[(\d+)\]$')

    # extract second element of each tuple into a list
    motor_cols = [col for _, col in motor_cols]

    # Prefer command channels u[i] for indices that are known motors.
    motor_active = (data[motor_cols] > 0).any(axis=1)
    if not motor_active.any():
        return None

    return data['timeS'].loc[motor_active[motor_active].index[0]]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    print("cropping")
    log.data, _ = log.crop(args.crop[0], args.crop[1])

auto_crop_start = _find_first_motor_command_time(log.data)
if auto_crop_start is not None:
    print("auto cropping to first motor command at time {:.2f} seconds".format(auto_crop_start))
    log.data, _ = log.crop(auto_crop_start-2, log.data['timeS'].iloc[-1])

# fplt = FlightPlotter(log.data, name=f"{args.name} -- Flight Data")
# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
fplt = IndiflightPlotter(log.data, Nr=6, Ns=2, name=f"{args.name} -- Flight Data -- {log.parameters['Firmware revision']}")
mplt = IndiflightMotorSysIdPlotter(log.data, Nr=4, name=f"{args.name} -- Onboard Motor Analysis (first 4 only)")
# splt = IndiflightServoSysIdPlotter(log.data, Nr=2, Ns=2, true=servo_true, name=f"{args.name} -- Onboard Servo Analysis")


#tplt = IndiflightEffectiveness(log.data, Nr=2, Ns=2, name=f"{args.name} -- Effectiveness -- {log.parameters['Firmware revision']}", scheduled=True)
#fplt.connect_viewport(tplt)
#aplt.connect_viewport(tplt)
#moplt.connect_viewport(tplt)

# craft = Quadrotor()
#craft = Tailsitter()
craft = Hexarotor()
pplt = IndiflightViewport(craft, log.data, Nr=6, Ns=0, follow=False, title=f"{args.name} -- Onboard ID Analysis -- {log.parameters['Firmware revision']}")
#pfplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=True, title=f"{args.name} -- Onboard ID Analysis")
fplt.connect_viewport(pplt)
mplt.connect_viewport(pplt)
#splt.connect_viewport(pplt)

#fplt.connect_viewport(pfplt)
#aplt.connect_viewport(pfplt)

if args.mp4:
    vplt = VideoViewport(
        log.data['timeS'].to_numpy(),
        title=f"{args.name} -- Video",
        mp4_path=args.mp4,
        offset_s=args.video_offset,
    )
    fplt.connect_viewport(vplt)

# cursor = BlittedCursor(fplt.all_axes + mplt.all_axes + splt.all_axes + aplt.all_axes, sharex=True)
cursor = BlittedCursor(fplt.all_axes + mplt.all_axes, sharex=True)

plt.show()


#%% extract IMU location from effectiveness matrix

# iend = log.data.index[-1]
# n = 4
# 
# fx = log.data.loc[iend][[f'fx_x_rls_x[{i}]' for i in range(n)]].to_numpy()
# fy = log.data.loc[iend][[f'fx_y_rls_x[{i}]' for i in range(n)]].to_numpy()
# fz = log.data.loc[iend][[f'fx_z_rls_x[{i}]' for i in range(n)]].to_numpy()
# fp = log.data.loc[iend][[f'fx_p_rls_x[{i}]' for i in range(n)]].to_numpy()
# fq = log.data.loc[iend][[f'fx_q_rls_x[{i}]' for i in range(n)]].to_numpy()
# fr = log.data.loc[iend][[f'fx_r_rls_x[{i}]' for i in range(n)]].to_numpy()
# 
# fdp = log.data.loc[iend][[f'fx_p_rls_x[{i}]' for i in range(2*n, 3*n)]].to_numpy()
# fdq = log.data.loc[iend][[f'fx_q_rls_x[{i}]' for i in range(2*n, 3*n)]].to_numpy()
# fdr = log.data.loc[iend][[f'fx_r_rls_x[{i}]' for i in range(2*n, 3*n)]].to_numpy()
# 
# import numpy as np
# 
# G1 = np.vstack((fx, fy, fz, fp, fq, fr))
# G2 = np.vstack((fdp, fdq, fdr))
# G2n = G2 / np.linalg.norm(G2, axis=0)
# G2n = np.zeros((3, n))
# G2n[2, :] = 1.0
# 
# def skew(x):
#     return np.array([[0, -x[2], x[1]],
#                      [x[2], 0, -x[0]],
#                      [-x[1], x[0], 0]])
# 
# A = np.zeros((3*n, 3))
# b = np.zeros((3*n))
# for m in range(n):
#     G1m = G1[:, m]
#     G2nm = G2n[:, m]
# 
#     # build skew-symmetric matrix of rotational part of G1
#     M = -skew(G1m[3:])
# 
#     Pperp = np.eye(3) - G2nm.reshape(3, 1) @ G2nm.reshape(1, 3)
# 
#     # normal equations  Pperp @ M @ x = -Pperp @ G2nm
#     A[3*m:3*m+3, :3] = Pperp @ M
#     b[3*m:3*m+3] = -Pperp @ G1m[:3] # only take the translational part of G1m
# 
# # solve for x
# x = np.linalg.lstsq(A, b, rcond=None)[0]
# 
# 
# G1fixed = G1.copy()
# for m in range(n):
#     G1fixed[:3, m] -= skew(G1[3:, m]) @ x
# 
# print("IMU location (x, y, z):")
# print(x)
# print("G1 standard:")
# print( (1e6*G1).round(2))
# print("G1 fixed:")
# print( (1e6*G1fixed).round(2))
