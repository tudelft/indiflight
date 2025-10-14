from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import LMS, RLS, RLS_fortescue, EWMV, Welford
from pyFlightPlotter import BlittedCursor
from pyFlightPlotter.crafts import Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport

import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
plt.close('all')

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Perform offline estimator prototyping on a log file.",
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

fplt = IndiflightPlotter(log.data, name=f"{args.name} -- Flight Data")

craft = Tailsitter()
pplt = IndiflightViewport(craft, log.data, follow=False, title=f"{args.name} -- Flight Data")
fplt.connect_viewport(pplt)

# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
# splt.connect_viewport(pplt)

# cursor = BlittedCursor(fplt.all_axes, sharex=True)
# cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)


#%% load data for offline estimator

N = log.data.shape[0]
iend = log.data.index[-1]
n = 2

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
dm_raw = log.data[[f"motor[{i}]" for i in range(n)]]         .to_numpy()
d_raw = log.data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy() / 100 / 180 * np.pi
q_raw = log.data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()
v_raw = log.data[[f"localVel[{i}]" for i in range(3)]]       .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
dt = np.mean(np.diff(t))

O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
dm = Signal(t_raw, dm_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)
q = Signal(t_raw, q_raw, rebase=t)
v = Signal(t_raw, v_raw, rebase=t)

order = 2
freq_hz = 15
Of = O.filter("lowpass", order, freq_hz)
af = a.filter("lowpass", order, freq_hz)
wf = w.filter("lowpass", order, freq_hz)
dmf = dm.filter("lowpass", order, freq_hz)
df = d.filter("lowpass", order, freq_hz)
qf = q.filter("lowpass", order, freq_hz)
vf = v.filter("lowpass", order, freq_hz)

import quaternion
qfy_norm = quaternion.as_quat_array( qf.y / np.linalg.norm(qf.y, axis=1, keepdims=True))
qfy_norm_inv = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
R_BI = quaternion.as_rotation_matrix(qfy_norm)
R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)
v_B = (R_IB @ vf.y[:, :, np.newaxis]).squeeze()
v_norm = np.linalg.norm(v_B, axis=1)
O_norm = np.linalg.norm(Of.y, axis=1)

eta_B = np.hstack((v_B, Of.y))
phi = 1.
eta = np.sqrt(v_norm**2 + phi * O_norm**2)

#%% do moments first

Oyz = Of.y[:, 1] * Of.y[:, 2]
Ozx = Of.y[:, 2] * Of.y[:, 0]
Oxy = Of.y[:, 0] * Of.y[:, 1]
OI = np.array([np.diag([Oxy[i], Oyz[i], Ozx[i]]) for i in range(Oyz.shape[0])])

w2 = wf.y*wf.y
wdot = wf.dot().y
w2d = w2 * df.y
ddot = df.dot().y
ddotdot = df.dot().dot().y

w2a = np.repeat(w2[:,0] + w2[:,1], 3).reshape(-1, 3)
w2t = np.array([w2[:,0] - w2[:,1], w2[:,0] + w2[:,1], w2[:,0] - w2[:,1]]).T
wdott = np.array([wdot[:,0] - wdot[:,1], wdot[:,0] + wdot[:,1], wdot[:,0] - wdot[:,1]]).T
w2dt = np.array([w2d[:,0] - w2d[:,1], w2d[:,0] + w2d[:,1], w2d[:,0] - w2d[:,1]]).T
ddott = np.array([ddot[:,0] - ddot[:,1], ddot[:,0] + ddot[:,1], ddot[:,0] - ddot[:,1]]).T
ddotdott = np.array([ddotdot[:,0] - ddotdot[:,1], ddotdot[:,0] + ddotdot[:,1], ddotdot[:,0] - ddotdot[:,1]]).T


rls_m_act = RLS(9, 3, gamma=1e-11, forgetting=0.9999)
rls_m_act.setTitle("RLS Moments -- Actuators Only")
rls_m_act.setParameters([0, 0, 0, 0, 0, 0, 0, 0, 0])
rls_m_act.setCovariance(1e-11 * np.diag([1, 1, 1, # Cw2
                                         1e-6, 1e-6, # Cw2d (y, z only)
                                         1e+2, # Cwdot (z only)
                                         1e+1, 1e+1, # Cddot (y, z only)
                                         1e-1 # Cddotdot (y only)
                                         ]))

rls_m_act_I = RLS(12, 3, gamma=1e-11, forgetting=0.9999)
rls_m_act_I.setTitle("RLS Moments -- Actuators and inertias")
rls_m_act_I.setParameters([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
rls_m_act_I.setCovariance(1e-11 * np.diag([1e9, 1e9, 1e9, # inertia terms
                                           1, 1, 1, # Cw2
                                           1e-6, 1e-6, # Cw2d (y, z only)
                                           1e+2, # Cwdot (z only)
                                           1e+1, 1e+1, # Cddot (y, z only)
                                           1e-1, # Cddotdot (y only)
                                           ]))

rls_phi5 = RLS(17, 3, gamma=1e-11, forgetting=0.9999)
rls_phi5.setTitle("RLS Moments -- Inertias, Actuators and 5-param Phi")
rls_phi5.setParameters([0, 0, 0,  0, 0, 0,  0, 0,  0,  0, 0,  0,    0, 0,  0, 0, 0])
rls_phi5.setCovariance(1e-11 * np.diag([1e9, 1e9, 1e9, # inertia terms 
                                        1, 1, 1, # Cw2
                                        1e-6, 1e-6, # Cw2d (y, z only)
                                        1e+2, # Cwdot (z only)
                                        1e+1, 1e+1, # Cddot (y, z only)
                                        1e-1, # Cddotdot (y only)
                                        1e9, 1e9,    # C_r_vy, C_q_vz
                                        1e9, 1e9, 1e9, # rate damping
                                        ]))

rls_phi8 = RLS(20, 3, gamma=1e-11, forgetting=0.9999)
rls_phi8.setTitle("RLS Moments -- Inertias, Actuators and 8-param Phi")
rls_phi8.setParameters([0, 0, 0,  0, 0, 0,  0, 0,  0,  0, 0,  0,    0, 0,   0, 0, 0, 0, 0, 0])
rls_phi8.setCovariance(1e-11 * np.diag([1e9, 1e9, 1e9, # inertia terms
                                        1, 1, 1, # Cw2
                                        1e-6, 1e-6, # Cw2d (y, z only)
                                        1e+2, # Cwdot (z only)
                                        1e+1, 1e+1, # Cddot (y, z only)
                                        1e-1, # Cddotdot (y only)
                                        1e9, 1e9, 1e9, 1e9, # C_p_vy, C_q_vx, C_q_vz, C_r_vy
                                        1e9, 1e9, 1e9, 1e9, # rate damping diagonal, and C_r_wx
                                        ]))

rls_phi9 = RLS(21, 3, gamma=1e-11, forgetting=0.9999)
rls_phi9.setTitle("RLS Moments -- Inertias, Actuators and 9-param Phi")
rls_phi9.setParameters([0, 0, 0,  0, 0, 0,  0, 0,  0,  0, 0,  0,    0, 0,   0, 0, 0, 0, 0, 0, 0])
rls_phi9.setCovariance(1e-11 * np.diag([1e9, 1e9, 1e9, # inertia terms
                                        1, 1, 1, # Cw2
                                        1e-1, 1e-1, # Cw2d (y, z only)
                                        1e+2, # Cwdot (z only)
                                        1e+6, 1e+6, # Cddot (y, z only)
                                        1e+4, # Cddotdot (y only)
                                        1e9, 1e9, 1e9, 1e9, # C_p_vy, C_q_vx, C_q_vz, C_r_vy
                                        1e9, 1e9, 1e9, 1e9, 1e9, # rate damping diagonal, and C_r_wx
                                        ]))

from copy import deepcopy

rls_phi9_noI = deepcopy(rls_phi9)
rls_phi9_noI.setTitle("RLS Moments -- Actuators and 9-param Phi")

rls_phi7_noI = deepcopy(rls_phi9)
rls_phi7_noI.setTitle("RLS Moments -- Actuators and 7-param Phi")

rls_phi3_noI = deepcopy(rls_phi9)
rls_phi3_noI.setTitle("RLS Moments -- Actuators and 3-param Phi")

rls_noPhi_noI = deepcopy(rls_phi9)
rls_noPhi_noI.setTitle("RLS Moments -- Actuators only")

rls_phi7 = deepcopy(rls_phi9)
rls_phi7.setTitle("RLS Moments -- Inertias, Actuators and 7-param Phi")

rls_phi3 = deepcopy(rls_phi9)
rls_phi3.setTitle("RLS Moments -- Inertias, Actuators and 3-param Phi")

rls_noPhi = deepcopy(rls_phi9)
rls_noPhi.setTitle("RLS Moments -- Inertias and Actuators only")


rls_var = EWMV(forgetting=0.99)
rls_var.setTitle("EMWV Moments Variance")
rls_var.setParameters([0, 0])

rls_var_welford = Welford()
rls_var_welford.setTitle("Welford Moments Variance")
rls_var_welford.setParameters([0, 0])



def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

for ti, w2_ai, w2_ti, w2_d_ti, wdot_ti, ddot_ti, ddotdot_ti, Oi, Odoti, eta_Bi, eta_i in tqdm(zip(t, w2a, w2t, w2dt, wdott, ddott, ddotdott, Of.y, Of.dot().y, eta_B, eta), total=len(w2a)):
    y = Odoti
    # first try: actuayors only, negelct aerodynamics
    # T = Cw2 * w2ti  +  Cw2d * w2dti  +  Cwdot * wdotti  +  Cddot * ddti  +  Cddotdot * dddti
    inertia_regs = -np.diag([Oi[1]*Oi[2], Oi[2]*Oi[0], Oi[0]*Oi[1]])

    Aphi9 = np.zeros((3, 21))
    Aphi9[0:3, 0:3] = inertia_regs
    Aphi9[0:3, 3:6] = np.diag(w2_ti[0:3])
    Aphi9[1:3, 6:8] = np.diag(w2_d_ti[1:3])
    Aphi9[2:3, 8:9] = np.diag(wdot_ti[2:3])
    Aphi9[1:3, 9:11] = 0.*np.diag(ddot_ti[1:3])
    Aphi9[1:2, 11:12] = 0.*np.diag(ddotdot_ti[1:2])
    Aphi9[0, 12] = eta_i * eta_Bi[1] # C_p_vy
    Aphi9[1, 13] = eta_i * eta_Bi[0] # C_q_vx
    Aphi9[1, 14] = eta_i * eta_Bi[2] # C_q_vz
    Aphi9[2, 15] = eta_i * eta_Bi[1] # C_r_vy
    Aphi9[:, 16:19] = eta_i * np.diag(eta_Bi[3:]) # C_m_w
    Aphi9[2, 19] = eta_i * eta_Bi[3] # C_r_wx
    Aphi9[0, 20] = eta_i * eta_Bi[5] # C_p_wz
    rls_phi9.newSample(Aphi9, y, ti); rls_phi9.update()

    A_phi9_noI = Aphi9.copy()
    A_phi9_noI[0:3, 0:3] = 0.
    rls_phi9_noI.newSample(A_phi9_noI, y, ti); rls_phi9_noI.update()

    A_phi7_noI = A_phi9_noI.copy()
    A_phi7_noI[0:3, 19:21] = 0.
    rls_phi7_noI.newSample(A_phi7_noI, y, ti); rls_phi7_noI.update()

    A_phi3_noI = A_phi7_noI.copy()
    A_phi3_noI[0:3, 12:16] = 0.
    rls_phi3_noI.newSample(A_phi3_noI, y, ti); rls_phi3_noI.update()

    A_noPhi_noI = A_phi3_noI.copy()
    A_noPhi_noI[0:3, 16:19] = 0.
    rls_noPhi_noI.newSample(A_noPhi_noI, y, ti); rls_noPhi_noI.update()

    A_phi7 = Aphi9.copy()
    A_phi7[0:3, 19:21] = 0.
    rls_phi7.newSample(A_phi7, y, ti); rls_phi7.update()

    A_phi3 = A_phi7.copy()
    A_phi3[0:3, 12:16] = 0.
    rls_phi3.newSample(A_phi3, y, ti); rls_phi3.update()

    A_noPhi = A_phi3.copy()
    A_noPhi[0:3, 16:19] = 0.
    rls_noPhi.newSample(A_noPhi, y, ti); rls_noPhi.update()

    # EMWV variance estimation
    e_sample = rls_noPhi.predictNew(A_noPhi).squeeze() - y
    rls_var.newSample(np.zeros(2), e_sample[0], ti); rls_var.update()


# remove all [9, 10] and [11] entries from all these parGroups, and also parGroupNames, go!

# rls_phi9.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [12,13,14,15], [16,17,18], [19,20]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$", "$C_{m\\omega_{cross}}$"],
#                         sharey=False, zoomy=False)
# 
# rls_phi9_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [12,13,14,15], [16,17,18], [19,20]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$", "$C_{m\\omega_{cross}}$"],
#                             sharey=False, zoomy=False)
# 
# rls_phi7_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [12,13,14,15], [16,17,18]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$"],
#                             sharey=False, zoomy=False)

# rls_phi3_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [16,17,18]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{m\\omega diag}$"],
#                             sharey=False, zoomy=False)
# 
# rls_noPhi_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$"],
#                             sharey=False, zoomy=False)

# rls_phi7.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [12,13,14,15], [16,17,18]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$"],
#                         sharey=False, zoomy=False)
# 
rls_phi3.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [16,17,18]],
                        parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{m\\omega diag}$"],
                        sharey=False, zoomy=False)

# rls_noPhi.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$"],
#                         sharey=False, zoomy=False)

rls_var.plotParameters()

all_rls = [
           # rls_phi9, rls_phi9_noI, rls_phi7_noI,
           rls_phi3_noI, rls_noPhi_noI,
           # rls_phi7,
           rls_phi3, rls_noPhi,
           ]

all_rls = [rls_phi3, rls_var]
# all_rls = [rls_noPhi, rls_var]

# display figures
all_axes = []
for rls in all_rls:
    all_axes.extend(rls.all_axes)

cursor = BlittedCursor(all_axes + fplt.all_axes, sharex=True)
plt.show()


# # save all figures as eps
# for rls in all_rls:
#     rls.f.savefig("output/" + rls.name.replace(" ", "_") + ".eps", format='eps', dpi=300)
# cursor = BlittedCursor(fplt.all_axes, sharex=True)
