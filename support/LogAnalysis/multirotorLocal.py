#!/usr/bin/env python3

from indiflight_log_tools import IndiflightLog
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm

from handy_signal_tools import Signal
from estimators import LS, RLS
from plotting import BlittedCursor

# plt.close('all')

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Analyse onboard ID data from a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
# parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=False)

# crop log if requested
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

# import data to numpy
N = log.data.shape[0]
n = 4

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
a_obd = log.data[[f"accSmooth[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
w_raw[:, 3] *= 6/7
d_raw = log.data[[f"motor[{i}]" for i in range(n)]]          .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_obd, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)

order = 2
freq_hz = 20
Of = O.filtfilt("lowpass", order, freq_hz)
af = a.filtfilt("lowpass", order, freq_hz)
wf = w.filtfilt("lowpass", order, freq_hz)
df = d.filtfilt("lowpass", order, freq_hz)

# get arming time
arm_time = None
for event in log.events:
    if event['name'] == 'Sync beep':
        arm_time = event['time'] * 1e-6
        break

if arm_time is None:
    raise ValueError("No 'Sync beep' event found in log, cannot determine arm time.")

# get indices where we are definitely in flight
MIN_GYRO = 100 / 180 * np.pi  # 100 deg/s
MAX_ACC = 0.9 * 9.81 # m/s^2
ground_contact = (np.linalg.norm(O.y, axis=1) < MIN_GYRO) & \
                 (np.linalg.norm(af.y, axis=1) > MAX_ACC) & \
                 (np.linalg.norm(d.y, axis=1) < 0.4)

flight = ~ground_contact & (t > arm_time) 

n_flight = np.sum(flight)

# get indices where we are learning
LEARNER_MODE = 11
LEARNER_TIMEOUT = 2.
learning = flight.copy()
if len(log.flags) > 1 and LEARNER_MODE in log.flags.iloc[0].enable:
    learning &= (t*1e6 > log.flags.iloc[0].timeUs)
    learning &= (t*1e6 < log.flags.iloc[1].timeUs)
    learning &= (t < arm_time + LEARNER_TIMEOUT)

n_learn = np.sum(learning)

# Features
# 1. [DONE] take in log file
# 2. [DONE] extract throw time, if applicable
# 3. [DONE] run global LS analysis with all data, and also with only throw data
# 4. run incremental LS analysis with all data, and also with only throw data
# 5. run same data through RLS with 0 initial conditions
# 6. plot results comparing 6 outputs (3x acc, 3x gyro)
# 7. print out effectiveness matrices in omega domain
# hold off on motor model for now

# clarification on all data: only in flight (not on ground) and only in cropped region, when crop is passed


#%% effectiveness regressors

# model:
#   \Delta a_i             =  \sum{ c_ij  2\omega_j \Delta \omega_j }_j
#   \Delta \dot{\Omega_i}  =  \sum{ c_ij  2\omega_j \Delta \omega_j }_j

estimators_flight = {
    'ls_global': None,
    'ls_incremental': None,
    'rls_global': None,
    'rls_incremental': None,
}
estimators_learning = {
    'ls_global': None,
    'ls_incremental': None,
    'rls_global': None,
    'rls_incremental': None,
}

t_plot = np.concatenate((t[learning], [t[learning][-1] + 0.002])) * 1

ntg = n*3 + 3
nrg = n*2*3 + 3
ng = ntg + nrg
nti = n*3
nri = n*2*3

groups_global = [
    range(0, n), range(n, 2*n), range(2*n, 3*n),
    range(3*n, ntg),
    range(ntg+0*n, ntg+1*n), range(ntg+2*n, ntg+3*n), range(ntg+4*n, ntg+5*n),
    range(ntg+1*n, ntg+2*n), range(ntg+3*n, ntg+4*n), range(ntg+5*n, ntg+6*n),
    range(ntg+6*n, ntg+nrg)
]

ltx_o2 = "\\omega^2 \\rightarrow "
ltx_od = "\\dot{\\omega} \\rightarrow "

groupNames_global = [
    f"${ltx_o2}f_x$", f"${ltx_o2}f_y$", f"${ltx_o2}f_z$",
    "IMU offset",
    f"${ltx_o2}m_x$", f"${ltx_o2}m_y$", f"${ltx_o2}m_z$",
    f"${ltx_od}m_x$", f"${ltx_od}m_y$", f"${ltx_od}m_z$",
    "Intertia ratios"
]

groups_incremental = [
    range(0, n), range(n, 2*n), range(2*n, 3*n),
    range(nti+0*n, nti+1*n), range(nti+2*n, nti+3*n), range(nti+4*n, nti+5*n),
    range(nti+1*n, nti+2*n), range(nti+3*n, nti+4*n), range(nti+5*n, nti+6*n),
]

groupNames_incremental = [
    f"${ltx_o2}f_x$", f"${ltx_o2}f_y$", f"${ltx_o2}f_z$",
    f"${ltx_o2}m_x$", f"${ltx_o2}m_y$", f"${ltx_o2}m_z$",
    f"${ltx_od}m_x$", f"${ltx_od}m_y$", f"${ltx_od}m_z$",
]

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

all_figs = []
for name, select, estimators in zip(["full", "throw"], [flight, learning], [estimators_flight, estimators_learning]):
    n_samples = np.sum(select)

    # shorthands for global model
    w2 = wf.y[select]**2
    wd = wf.dot().y[select]
    O = Of.y[select]
    Od = Of.dot().y[select]
    a = af.y[select]
    Odx_OxOx = np.array([skew(Od[i]) + skew(O[i])@skew(O[i]) for i in range(n_samples)])

    # shorthands for incremental model
    w2diff = wf.diff().y[select]*2*wf.y[select]
    Oddiff = Of.dot().diff().y[select]
    wddiff = wf.dot().diff().y[select]
    adiff = af.diff().y[select]

#%% global model LS and RLS
    # translational regressors: motors and IMU offset
    Atg = np.zeros((3, n_samples, ntg))
    Atg[0, :, 0*n:1*n] = w2[np.newaxis]
    Atg[1, :, 1*n:2*n] = w2[np.newaxis]
    Atg[2, :, 2*n:3*n] = w2[np.newaxis]
    Atg[:, :, 3*n:]    = np.transpose(Odx_OxOx, (1, 0, 2))

    # rotational regressors: motors, derivatives, and gyro cross terms
    Arg = np.zeros((3, n_samples, nrg))
    Arg[0, :, 0*n:1*n] = w2[np.newaxis]
    Arg[1, :, 2*n:3*n] = w2[np.newaxis]
    Arg[2, :, 4*n:5*n] = w2[np.newaxis]
    Arg[0, :, 1*n:2*n] = wd[np.newaxis]
    Arg[1, :, 3*n:4*n] = wd[np.newaxis]
    Arg[2, :, 5*n:6*n] = wd[np.newaxis]
    Arg[0, :, 6*n+0]   = -O[:, 1] * O[:, 2]
    Arg[1, :, 6*n+1]   = -O[:, 0] * O[:, 2]
    Arg[2, :, 6*n+2]   = -O[:, 0] * O[:, 1]

    # combined regressors
    Ag = np.zeros((6, n_samples, ng))
    Ag[0:3, :, 0:ntg] = Atg
    Ag[3:6, :, ntg:ng] = Arg

    # targets
    yg = np.zeros((6, n_samples))
    yg[:3, :] = a.T
    yg[3:, :] = Od.T

#%% incremental model
    # translational regressors: motors only
    Ati = np.zeros((3, n_samples, nti))
    Ati[0, :, 0*n:1*n] = w2diff[np.newaxis]
    Ati[1, :, 1*n:2*n] = w2diff[np.newaxis]
    Ati[2, :, 2*n:3*n] = w2diff[np.newaxis]

    # rotational regressors: motors and derivatives only
    Ari = np.zeros((3, n_samples, nri))
    Ari[0, :, 0*n:1*n] = w2diff[np.newaxis]
    Ari[1, :, 2*n:3*n] = w2diff[np.newaxis]
    Ari[2, :, 4*n:5*n] = w2diff[np.newaxis]
    Ari[0, :, 1*n:2*n] = wddiff[np.newaxis]
    Ari[1, :, 3*n:4*n] = wddiff[np.newaxis]
    Ari[2, :, 5*n:6*n] = wddiff[np.newaxis]

    # combined regressors
    ni = nti + nri
    Ai = np.zeros((6, n_samples, ni))
    Ai[0:3, :, 0:nti] = Ati
    Ai[3:6, :, nti:ni] = Ari

    # targets
    yi = np.zeros((6, n_samples))
    yi[:3, :] = adiff.T
    yi[3:, :] = Oddiff.T


#%% run estimation
    # estimators
    ls_global = LS(d=6, n=ng); ls_global.setTitle(f"{name} -- Global LS")
    rls_global = RLS(d=6, n=ng, forgetting=0.9999); rls_global.setTitle(f"{name} -- Global RLS")
    ls_incremental = LS(d=6, n=ni); ls_incremental.setTitle(f"{name} -- Incremental LS")
    rls_incremental = RLS(d=6, n=ni, forgetting=0.9999); rls_incremental.setTitle(f"{name} -- Incremental RLS")

    estimators['ls_global'] = ls_global
    estimators['rls_global'] = rls_global
    estimators['ls_incremental'] = ls_incremental
    estimators['rls_incremental'] = rls_incremental

    # rls_global.setCovariance(np.diag(
    #     3*(n*[1e-10] + n*[1e-7]) + 3*[1e-3]
    # ))

    print()
    print("--- starting ", name, " calculations ---")

    for i in tqdm(range(n_samples), desc="Batched LS"):
        ls_global.newSample(Ag[:, i, :], yg[:, i], t=t[select][i])
        if i > 100 and i%100 == 0:
            ls_global.update()

    for i in tqdm(range(n_samples), desc="RLS"):
        rls_global.newSample(Ag[:, i, :], yg[:, i], t=t[select][i])
        rls_global.update()

    for i in tqdm(range(n_samples), desc="Incremental Batched LS"):
        ls_incremental.newSample(Ai[:, i, :], yi[:, i], t=t[select][i])
        if i > 100 and i%100 == 0:
            ls_incremental.update()

    for i in tqdm(range(n_samples), desc="Incremental RLS"):
        rls_incremental.newSample(Ai[:, i, :], yi[:, i], t=t[select][i])
        rls_incremental.update()

    # plots
    figsize = (25, 15)
    f = ls_global.plotParameters(parGroups=groups_global, parGroupNames=groupNames_global, sharey=False, figsize=figsize)
    f.savefig(f"output/{args.name}_{ls_global.name}.eps")

    f = rls_global.plotParameters(parGroups=groups_global, parGroupNames=groupNames_global, sharey=False, figsize=figsize)
    f.savefig(f"output/{args.name}_{rls_global.name}.eps")

    f = ls_incremental.plotParameters(parGroups=groups_incremental, parGroupNames=groupNames_incremental, sharey=False, figsize=figsize)
    f.savefig(f"output/{args.name}_{ls_incremental.name}.eps")

    f = rls_incremental.plotParameters(parGroups=groups_incremental, parGroupNames=groupNames_incremental, sharey=False, figsize=figsize)
    f.savefig(f"output/{args.name}_{rls_incremental.name}.eps")


axes = []
for f in all_figs:
    axes.extend(f.axes)

bc = BlittedCursor(axes, sharex=True)

plt.show()

# for f in figs:
#     f.savefig(f"outputs/{args.name}_{f._title.replace(' ', '_')}.eps", dpi=300)


#%% output effectiveness matrices for each estimator in omega domain and indiflight format

wmax = 4200 # rad/s
for est_name, estimators in zip(["full", "throw"], [estimators_flight, estimators_learning]):
    print()
    print(f"----- Effectiveness Matrices for {est_name} data -----")
    print()
    for est_key in estimators.keys():
        est = estimators[est_key]
        G1 = np.zeros((6, n))
        G2 = np.zeros((3, n))

        if "incremental" in est_key:
            G1[0, :] = est.theta[0*n:1*n].squeeze()
            G1[1, :] = est.theta[1*n:2*n].squeeze()
            G1[2, :] = est.theta[2*n:3*n].squeeze()
            G1[3, :] = est.theta[3*n:4*n].squeeze()
            G1[4, :] = est.theta[5*n:6*n].squeeze()
            G1[5, :] = est.theta[7*n:8*n].squeeze()

            G2[0, :] = est.theta[4*n:5*n].squeeze()
            G2[1, :] = est.theta[6*n:7*n].squeeze()
            G2[2, :] = est.theta[8*n:9*n].squeeze()
        elif "global" in est_key:
            G1[0, :] = est.theta[0*n:1*n].squeeze()
            G1[1, :] = est.theta[1*n:2*n].squeeze()
            G1[2, :] = est.theta[2*n:3*n].squeeze()
            G1[3, :] = est.theta[3*n+3:4*n+3].squeeze()
            G1[4, :] = est.theta[5*n+3:6*n+3].squeeze()
            G1[5, :] = est.theta[7*n+3:8*n+3].squeeze()

            G2[0, :] = est.theta[4*n+3:5*n+3].squeeze()
            G2[1, :] = est.theta[6*n+3:7*n+3].squeeze()
            G2[2, :] = est.theta[8*n+3:9*n+3].squeeze()
        else:
            raise ValueError("Unknown estimator key.")


        print(f"--- {est.name} ---")
        print()
        print("Effectiveness matrix G1:")
        print( (G1 * (wmax**2) * np.array([100, 100, 100, 10, 10, 10])[:, np.newaxis]).round(0) )
        print()
        print("Effectiveness matrix G2:")
        print((1e5*G2).round(0))
        print()

#%% motor model

# model:
#   tau * w_dot = ws - w
#   ws = wmax  (k d  +  (1-k) sqrt(d))  +  widle
#   ws = wmax*k * d  +  wmax*(1-k) * sqrt(d)  +  widle
#   ws = a * d  +  b * sqrt(d)  +  widle
#   w  =  ws  -  tau * w_dot
#   w  =  a*d + b*sqrt(d) + widle - tau * w_dot

# Am = np.empty((N, 4, 4))
# Am[:, 0]  =  df.y
# Am[:, 1]  =  np.sqrt(df.y)
# Am[:, 2]  =  1.
# Am[:, 3]  =  -wf.dot().y
# 
# ym = wf.y
# 
# wmax = np.empty((4,))
# kappa = np.empty((4,))
# widle = np.empty((4,))
# tau = np.empty((4,))
# for i in range(4):
#     Xm, _, _, _ = np.linalg.lstsq(Am[:, :, i], ym[:, i])
#     a, b, widle[i], tau[i] = Xm
#     # wmax*k = a
#     # wmax*(1-k) = b
#     # a + b = wmax
#     # k = a / wmax
#     wmax[i] = a+b
#     kappa[i] = a / (a+b)
# 
# print()
# print(f"----- Motor Model found -----")
# print()
# print(f"set indi_act_time_constant_ms = {','.join([str(int(np.round(x))) for x in 1e3*tau])}")
# print(f"set indi_act_max_rpm = {         ','.join([str(int(np.round(x))) for x in wmax/(2*np.pi)*60])}")
# print(f"set indi_act_hover_rpm = {       ','.join([str(int(np.round(x))) for x in 0.5*wmax/(2*np.pi)*60])}") # todo: bad approx, shouldnt matter
# print(f"set indi_act_nonlinearity = {    ','.join([str(int(np.round(x))) for x in 100.*kappa])}")
# print()


#%% scaled fx model

# wmax2 = wmax**2
# G1 = np.empty((6, 4))
# G1[:] = np.nan
# 
# G1[:2, :] = 0.
# G1[ 2, :] = (Xa[:, 2] @ wmax2) / 4
# G1[3:, :] = XO[:4, :].T * wmax2
# 
# G2 = XO[4:, :].T
# G2[:2, :] = 0.
# 
# # convert to indiflight units
# G1_int = np.zeros_like(G1, dtype=np.int16)
# G1_int[:3, :] = G1[:3, :] * 100
# G1_int[3:, :] = G1[3:, :] * 10
# 
# G2_int = np.zeros_like(G2, dtype=np.int16)
# G2_int[:] = G2 * 1e5
# 
# print()
# print(f"----- Effectiveness G1/G2 found -----")
# print()
# print(f"set indi_act_g1_fx = {   ','.join([str(x) for x in G1_int[0, :]])}")
# print(f"set indi_act_g1_fy = {   ','.join([str(x) for x in G1_int[1, :]])}")
# print(f"set indi_act_g1_fz = {   ','.join([str(x) for x in G1_int[2, :]])}")
# print(f"set indi_act_g1_roll = { ','.join([str(x) for x in G1_int[3, :]])}")
# print(f"set indi_act_g1_pitch = {','.join([str(x) for x in G1_int[4, :]])}")
# print(f"set indi_act_g1_yaw = {  ','.join([str(x) for x in G1_int[5, :]])}")
# print()
# print(f"set indi_act_g2_roll = { ','.join([str(x) for x in G2_int[0, :]])}")
# print(f"set indi_act_g2_pitch = {','.join([str(x) for x in G2_int[1, :]])}")
# print(f"set indi_act_g2_yaw = {  ','.join([str(x) for x in G2_int[2, :]])}")
# print()


#%% plotting

# plt.close('all')
# fig, axs = plt.subplots(6, 2, sharex=True)
# 
# AXES = ["fx", "fy", "fz", "mx", "my", "mz"]
# for i, ax in enumerate(AXES):
#     if i < 3:
#         axs[i, 0].plot(t, ya[:, i], label=f"{ax} targets")
#         axs[i, 0].plot(t, Aa @ Xa[:, i], label=f"{ax} model")
#         axs[i, 0].set_ylabel("$\Delta m\ s^{-2}$")
#     else:
#         axs[i, 0].plot(t, yO[:, i-3], label=f"{ax} targets")
#         axs[i, 0].plot(t, AO @ XO[:, i-3], label=f"{ax} model")
#         axs[i, 0].set_ylabel("$\Delta rad\ s^{-2}$")
# 
# MOTORS = ["1", "2", "3", "4"]
# for i, m in enumerate(MOTORS):
#     axs[i, 1].plot(t, w_raw[:, i], label=f"motor {m}")
#     axs[i, 1].plot(t, wf.y [:, i], label=f"motor {m} filtered")
#     axs[i, 1].set_ylabel("$rad\ s^{-1}$")
# 
# axs[-1, 0].set_xlabel("Time [s]")
# axs[-1, 1].set_xlabel("Time [s]")
# 
# for ax in axs.flatten():
#     ax.legend()
# 
# fig.show()
