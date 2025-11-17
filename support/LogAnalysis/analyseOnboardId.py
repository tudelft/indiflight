from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt
import matplotlib

plt.close('all')
matplotlib.use('tkagg') 

from pyFlightPlotter import local_rc
from indiflightPlotter import IndiflightPlotter, IndiflightMotorSysIdPlotter, IndiflightServoSysIdPlotter, IndiflightViewport
from pyFlightPlotter import Quadrotor, Tailsitter, BlittedCursor

# set local_rc
plt.rcParams.update(local_rc)

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Analyse onboard ID data from a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

servo_true = {
    'motor_2_rls_x[0]': 1.75,    # max angle in rad
    'motor_2_rls_x[1]': 0.0,     # neutral angle in rad
    'motor_2_rls_x[2]': 0.03,    # delay in seconds
    'motor_2_rls_x[3]': 0.,     # time constant in seconds

    'motor_3_rls_x[0]': 1.75,    # max angle in rad
    'motor_3_rls_x[1]': 0.0,     # neutral angle in rad
    'motor_3_rls_x[2]': 0.03,    # delay in seconds
    'motor_3_rls_x[3]': 0.,     # time constant in seconds

    'fx_p_rls_x[0]': (1e-6 * 0.13) / 6e-3,
    'fx_p_rls_x[1]': 0,
    'fx_p_rls_x[2]': 0,
    'fx_p_rls_x[3]': 0,

    'fx_q_rls_x[0]': (1e-6 * -0.01*2) / 2e-3,
    'fx_q_rls_x[1]': -4.47e-8 / 2e-3,
    'fx_q_rls_x[2]': 0,
    'fx_q_rls_x[3]': 0,

    'fx_r_rls_x[0]': (-0.005 * 1e-6) / 6.5e-3,
    'fx_r_rls_x[1]': -1.08e-7 / 6.5e-3,
    'fx_r_rls_x[2]': 0,
    'fx_r_rls_x[3]': 0,

    'sigma_rls[0]':  0.75,
    'sigma_rls[1]': -0.25,
    'sigma_rls[2]': -0.61538,
}

# fplt = FlightPlotter(log.data, name=f"{args.name} -- Flight Data")
# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
fplt = IndiflightPlotter(log.data, Nr=2, Ns=2, name=f"{args.name} -- Flight Data")
splt = IndiflightMotorSysIdPlotter(log.data, Nr=2, name=f"{args.name} -- Onboard Motor Analysis")
splt = IndiflightServoSysIdPlotter(log.data, Nr=2, Ns=2, true=servo_true, name=f"{args.name} -- Onboard Servo Analysis")

# craft = Quadrotor()
craft = Tailsitter()
pplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Onboard ID Analysis")
fplt.connect_viewport(pplt)
splt.connect_viewport(pplt)

cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)

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
