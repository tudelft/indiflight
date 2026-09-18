from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from scipy.optimize import minimize

from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import LS
from pyFlightPlotter import BlittedCursor
from pyFlightPlotter.crafts import Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport

plt.close("all")


parser = ArgumentParser(
    description="Perform offline estimator prototyping on a log file.",
    formatter_class=ArgumentDefaultsHelpFormatter,
)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument(
    "--crop",
    required=False,
    nargs=2,
    metavar=("START", "END"),
    type=float,
    help="Crop the log to the given time range (in seconds).",
)
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

fplt = IndiflightPlotter(log.data, Ns=2, Nr=2, name=f"{args.name} -- Flight Data")

craft = Tailsitter()
pplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Flight Data")
fplt.connect_viewport(pplt)

# %% load data for offline estimator

N = log.data.shape[0]
iend = log.data.index[-1]
n = 2

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
dm_raw = log.data[[f"motor[{i}]" for i in range(n)]]         .to_numpy() + 0.084
u_raw = log.data[[f"u[{i}]" for i in range(4)]]              .to_numpy()
d_raw = log.data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy()
# d_raw = np.roll(d_raw, -15, axis=0)
q_raw = log.data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()
#v_raw = log.data[[f"localVel[{i}]" for i in range(3)]]       .to_numpy()
v_raw = log.data[[f"vel[{i}]" for i in range(3)]]       .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
dt = np.mean(np.diff(t))

O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
dm = Signal(t_raw, dm_raw, rebase=t)
u = Signal(t_raw, u_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)
q = Signal(t_raw, q_raw, rebase=t)
v = Signal(t_raw, v_raw, rebase=t)

order = 2
freq_hz = 5
Of = O.filter("lowpass", order, freq_hz)
af = a.filter("lowpass", order, freq_hz)
wf = w.filter("lowpass", order, freq_hz)
dmf = dm.filter("lowpass", order, freq_hz)
uf = u.filter("lowpass", order, freq_hz)
df = d.filter("lowpass", order, freq_hz)
qf = q.filter("lowpass", order, freq_hz)
vf = v.filter("lowpass", order, freq_hz)

qify = (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True)

import quaternion
qfy_norm = quaternion.as_quat_array( qf.y / np.linalg.norm(qf.y, axis=1, keepdims=True))
qfy_norm_inv = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
R_BI = quaternion.as_rotation_matrix(qfy_norm)
R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)
v_B = (R_IB @ vf.y[:, :, np.newaxis]).squeeze()
v_norm = np.linalg.norm(v_B, axis=1)
O_norm = np.linalg.norm(Of.y, axis=1)

eta_B = np.hstack((v_B, Of.y))
phi = 0.0
eta = np.sqrt(v_norm**2 + phi * O_norm**2)

# add __file__/../simulation/PyNDIflight to path for pyNDIflight import
import os
import sys
sys.path.append( os.path.abspath( os.path.join( os.path.dirname(__file__), '..', 'simulation' ) ) )

from PyNDIflight.crafts import TailsitterPhi, TailsitterPhiFF
from PyNDIflight.helpers import quatRotate


mass = 0.564
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

k = 1.413e-6


def build_tail():
    tail = TailsitterPhi()
    tail.setInertia(m=mass, I=np.diag([5.73e-3, 1.35e-3, 5.43e-3]))
    tail.setRotor(0, X=[1.104e-02, -1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=k, cm=-6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # RR
    tail.setRotor(1, X=[1.104e-02, +1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=k, cm=+6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # FR
    tail.setPhiModel(0, np.zeros((6, 6)))
    tail.setElevonModel(cd, cdd, cddd, d0)
    return tail


def mean_r2(y_true, y_pred):
    scores = []
    for idx in range(y_true.shape[1]):
        target = y_true[:, idx]
        total = np.sum((target - np.mean(target)) ** 2)
        if total <= np.finfo(float).eps:
            continue
        residual = np.sum((target - y_pred[:, idx]) ** 2)
        scores.append(1.0 - residual / total)
    return float(np.mean(scores)) if scores else -np.inf


def run_analysis(wind_xy, progress=False):
    tail = build_tail()
    ls_Phi = LS(14, 6)

    wind = np.array([wind_xy[0], wind_xy[1], 0.0], dtype=float)
    ysim_hist = []
    yobs_hist = []
    nu_hist = []

    iterator = tqdm(
        zip(t, dmf.y, Of.y, Of.dot().y, wf.y, wf.dot().y, uf.y, df.y, df.dot().y, df.dot().dot().y, vf.y, qify, af.y),
        total=len(dmf.y),
        disable=not progress,
    )

    for ti, dmi, Oi, Odoti, wi, wdoti, ui, di, ddoti, dddoti, vi, qIi, ai in iterator:
        tail.r_w[:] = wi
        tail.r_wdot[:] = [0., 0.]
        tail.r_tau[:] = np.inf
        tail.s_d[:] = di
        tail.s_dd[:] = [0., 0.]
        tail.s_D *= 0.
        tail.s_P *= 0.
        tail.setPose(np.array([0., 0., -1.]), np.array([1., 0., 0., 0.]))

        wind_free_velocity = vi - wind
        vB = quatRotate(qIi, wind_free_velocity)

        tail.setTwist(wind_free_velocity, Oi)
        tail.vB[:] = vB

        tail.tick(1e-9)

        ysim = np.concatenate((tail.fspB, tail.ODotB))
        yobs = np.concatenate((ai, Odoti))
        nu = yobs - ysim
        ysim_hist.append(ysim)
        yobs_hist.append(yobs)
        nu_hist.append(nu)

        u, v, w = vB[0], vB[1], vB[2]
        p, q, r = Oi[0], Oi[1], Oi[2]

        vnorm = np.linalg.norm([u, v, w])

        A = np.zeros((6, 14))

        # forces [Cxu, Czu, Cyv, Czw]
        A[:3, :4] = -vnorm * np.array([
            [u, w, 0, 0],
            [0, 0, v, 0],
            [0, u, 0, w],
        ])

        # cross terms: [Cmu, Clv, Cnv, Cmw]
        A[:, 4:8] = -vnorm * np.array([
            [q, 0, 0, 0],
            [0, p, r, 0],
            [0, 0, 0, q],
            [0, v, 0, 0],
            [u, 0, 0, w],
            [0, 0, v, 0],
        ])

        # rate: [Clp, Cnp, Cmq, Cnr]
        A[3:, 8:12] = -vnorm * np.array([
            [p, r, 0, 0],
            [0, 0, q, 0],
            [0, p, 0, r],
        ])

        # elevons
        Ddi = di - d0
        A[3:, 12:14] = -vnorm * np.array([
            [0, 0],
            [w * (Ddi[0] + Ddi[1]), 0],
            [0, w * (Ddi[0] - Ddi[1])],
        ])

        ls_Phi.newSample(A, nu, ti)

    ls_Phi.update()
    nu_hat = ls_Phi.predictNew(ls_Phi.A_h).squeeze()

    ysim_hist = np.asarray(ysim_hist)
    yobs_hist = np.asarray(yobs_hist)
    nu_hist = np.asarray(nu_hist)

    return {
        "wind": wind,
        "score": mean_r2(nu_hist[:, 0:2], nu_hat[:, 0:2]),
        "ls_Phi": ls_Phi,
        "ysim_hist": ysim_hist,
        "yobs_hist": yobs_hist,
        "nu_hist": nu_hist,
    }


wind_guess = np.array([-0.0, -0.0], dtype=float)
#wind_bounds = [(-10.0, 10.0), (-10.0, 10.0)]
wind_bounds = [(-0.0, 0.0), (-0.0, 0.0)]


def objective(wind_xy):
    return -run_analysis(wind_xy, progress=True)["score"]


opt_result = minimize(objective, wind_guess, method="Powell", bounds=wind_bounds, options={"maxiter": 25, "xtol": 1e-2, "ftol": 1e-3})
print(f"Optimized wind parameters [wind_x, wind_y] = {opt_result.x.tolist()}")
print(f"Best mean R2 = {-opt_result.fun:.6f}")

analysis = run_analysis(opt_result.x, progress=True)
ls_Phi = analysis["ls_Phi"]
ysim_hist = analysis["ysim_hist"]
yobs_hist = analysis["yobs_hist"]
nu_hist = analysis["nu_hist"]

fig, axes = plt.subplots(2, 3, sharex=True, figsize=(16, 8))
axes = axes.ravel()
series_labels = ["force x", "force y", "force z", "moment p", "moment q", "moment r"]
for idx, axis in enumerate(axes):
    axis.plot(t[:ysim_hist.shape[0]], ysim_hist[:, idx], label="ysim", linewidth=1.0)
    axis.plot(t[:yobs_hist.shape[0]], yobs_hist[:, idx], label="yobs", linewidth=1.0, alpha=0.8)
    axis.plot(t[:nu_hist.shape[0]], nu_hist[:, idx], label="nu_hist", linewidth=1.0, alpha=0.8)
    axis.set_title(series_labels[idx])
    axis.grid(True, alpha=0.3)
    if idx >= 3:
        axis.set_xlabel("time [s]")
    if idx % 3 == 0:
        axis.set_ylabel("value")

axes[0].legend(loc="upper right", fontsize=8)
fig.suptitle(f"{args.name} -- ysim / yobs / nu_hist")
fig.tight_layout(rect=[0, 0, 1, 0.96])


#def compute_rmse(estimator, A_data, y_data):
#    y_pred = estimator.predictNew(A_data)
#    e = y_pred - y_data
#    return float(np.sqrt(np.mean(e**2)))
#
#
#A_data = np.vstack(A_hist)
#y_data = Odot.reshape(-1, 1)
#rmse = compute_rmse(ls_act_12, A_data, y_data)
#print(f"RMSE for {ls_act_12.name}: {rmse:.2f} rad/s^2")

#ls_Phi.plotParameters(
#    parGroups=[[0, 1, 2], [3,4,5], [6,7], [9,10], [8], [11], [12,13]],
#    parGroupNames=["Cxu,Czu,Cmu", "Cyv,Clv,Cnv", "Czw,Cmw", "Clp,Cnp", "Cmq", "Cnr", "elev"],
#    sharey=False,
#    zoomy=False,
#)

ls_Phi.plotParameters(
    parGroups=[[0, 2, 3], [1], [4,5,6,7], [8,10,11], [9], [12,13]],
    parGroupNames=["Cxu,Cyv,Czw","Czu", "Cmu,Clv,Cnv,Cnw", "Clp,Cmq,Cnr", "Cnp", "elev"],
    sharey=False,
    zoomy=False,
)

all_axes = list(ls_Phi.all_axes)
cursor = BlittedCursor(all_axes + fplt.all_axes, sharex=True)

plt.show()
