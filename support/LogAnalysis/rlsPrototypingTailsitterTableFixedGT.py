from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import LMS, RLS, RLS_fortescue, EWMV, Welford, LS
from flight_metrics import infer_learning_indices
from pyFlightPlotter import BlittedCursor
from pyFlightPlotter.crafts import Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport

import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
plt.close('all')

from copy import deepcopy
from itertools import product
from pathlib import Path
import importlib
import pickle
import sys

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Perform offline estimator prototyping on a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logpath", type=str, help="Path to the log file.")
# parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
# parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--skip-ls", action="store_true",
                    help="Skip LS estimators (faster; runs only RLS variants).")
parser.add_argument("--skip-plots", action="store_true",
                    help="Skip generating/saving diagnostic plots and plot text outputs.")
parser.add_argument("--params-pkl", required=True, type=str,
                    help="Parameter pickle produced by tailsitterParameterStudy.py (used for GT model parameters).")
parser.add_argument("--rls-in-recovery", type=float,
                    help="Run RLS for this many seconds during recovery maneuver.")
# parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()
# 
# if args.name is None:
#     args.name = args.logfile.split("/")[-1].split(".")[0]

# log_path  = "/mnt/data/WorkData/BlackboxLogs/tailsitterSimStudy"

import glob
log_glob = "LOG*.BFL"
log_list = sorted( glob.glob(f"{args.logpath}/{log_glob}") )
logs = []
for log_file in log_list:
    print(f"Processing log file: {log_file}")
    log = IndiflightLog(log_file, logId=1, resetTime=False)
    log.name = log_file.split("/")[-1].split(".")[0]
    if args.crop:
        log.data, _ = log.crop(args.crop[0], args.crop[1])

    logs.append(log)

    # fplt = IndiflightPlotter(log.data, Ns=2, Nr=2, name=f"{args.name} -- Flight Data")

    # craft = Tailsitter()
    # pplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Flight Data")
    # fplt.connect_viewport(pplt)

# add output folder to logfolder path
import os
output_path = f"{args.logpath}/output"
os.makedirs(output_path, exist_ok=True)


#%% load data for offline estimator

def ensure_pickle_import_path():
    simulation_dir = Path(__file__).resolve().parents[1] / "simulation"
    simulation_dir_str = str(simulation_dir)
    if simulation_dir_str not in sys.path:
        sys.path.insert(0, simulation_dir_str)

    # Pre-import package so pickle can resolve class names.
    importlib.import_module("PyNDIflight.crafts")


def load_base_tail_from_pickle(params_pkl):
    ensure_pickle_import_path()
    with open(params_pkl, "rb") as f:
        payload = pickle.load(f)

    if not isinstance(payload, dict) or "base_tail" not in payload:
        raise KeyError(f"Parameter pickle does not contain 'base_tail': {params_pkl}")
    return payload["base_tail"]


BASE_TAIL = load_base_tail_from_pickle(args.params_pkl)

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

import pandas as pd
table = pd.DataFrame()

class Parameters(object):
    def __init__(self, theta, tail):
        # Ground-truth data taken from the nominal tailsitter stored in params pickle.
        I_diag = np.diag(np.asarray(tail.I, dtype=np.float64))
        Ixx, Iyy, Izz = [float(v) for v in I_diag]
        cd = np.asarray(tail.cd, dtype=np.float64)
        cdd = np.asarray(tail.cdd, dtype=np.float64)
        cddd = np.asarray(tail.cddd, dtype=np.float64)
        d0 = np.asarray(tail.d0, dtype=np.float64)
        Phi = np.asarray(tail.Phi, dtype=np.float64)
        r_X = np.asarray(tail.r_X, dtype=np.float64)
        r_ax = np.asarray(tail.r_ax, dtype=np.float64)
        r_k = np.asarray(tail.r_k, dtype=np.float64)
        r_cm = np.asarray(tail.r_cm, dtype=np.float64)
        r_I = np.asarray(tail.r_I, dtype=np.float64)

        Cld = float(0.0)   / Ixx
        Cmd = float(cd[4]) / Iyy
        Cnd = float(cd[5]) / Izz

        Cmdd = float(cdd[4]) / Iyy
        Cmddd = float(cddd[4]) / Iyy

        # Rotor-induced moment coefficients for each rotor.
        clww_thrust = np.array([
            ((r_X[1, i] * r_ax[2, i]) - (r_X[2, i] * r_ax[1, i])) * r_k[i] / Ixx
            for i in range(2)
        ], dtype=np.float64)
        cmww_thrust = np.array([
            ((r_X[2, i] * r_ax[0, i]) - (r_X[0, i] * r_ax[2, i])) * r_k[i] / Iyy
            for i in range(2)
        ], dtype=np.float64)
        cnww_thrust = np.array([
            ((r_X[0, i] * r_ax[1, i]) - (r_X[1, i] * r_ax[0, i])) * r_k[i] / Izz
            for i in range(2)
        ], dtype=np.float64)

        clww_drag = np.array([
            (-r_k[i] * r_cm[i] * r_ax[0, i]) / Ixx
            for i in range(2)
        ], dtype=np.float64)
        cmww_drag = np.array([
            (-r_k[i] * r_cm[i] * r_ax[1, i]) / Iyy
            for i in range(2)
        ], dtype=np.float64)
        cnww_drag = np.array([
            (-r_k[i] * r_cm[i] * r_ax[2, i]) / Izz
            for i in range(2)
        ], dtype=np.float64)


        # Keep existing parameterization where d0-shifted elevon terms contribute to w^2 channels.
        Clww = clww_thrust + clww_drag
        Cmww = cmww_thrust + cmww_drag - Cmd * d0
        Cnww = cnww_thrust + cnww_drag - Cnd * d0

        # Matches the sign conventions used in the identification model for wdot channels.
        Clwd = np.array([
            (-np.sign(r_cm[i]) * r_I[i] * r_ax[0, i]) / Ixx
            for i in range(2)
        ], dtype=np.float64)
        Cnwd = np.array([
            (-np.sign(r_cm[i]) * r_I[i] * r_ax[2, i]) / Izz
            for i in range(2)
        ], dtype=np.float64)

        Clp = Phi[3,3] / Ixx
        Cmq = Phi[4,4] / Iyy
        Cnr = Phi[5,5] / Izz

        inertia_ratios = np.array([(Izz - Iyy) / Ixx,
                                   (Ixx - Izz) / Iyy,
                                   (Iyy - Ixx) / Izz])

        eval_pars = {'Clww1': [0, Clww[0]],      'Clww2': [3, Clww[1]],
                     'Cld1':  [1, Cld],          'Cld2':  [4, -Cld],
                     'Clwd1':  [2, Clwd[0]], 'Clwd2':  [5, Clwd[1]],
                     'Cmww1': [6, Cmww[0]],   'Cmww2': [10, Cmww[1]],
                     'Cmd1':  [7, Cmd],       'Cmd2':  [11, Cmd],
                     'Cmdd1':  [8, Cmdd],      'Cmdd2':  [12, Cmdd],
                     'Cmddd1':  [9, Cmddd],     'Cmddd2':  [13, Cmddd],
                     'Cnww1': [14, Cnww[0]],  'Cnww2': [17, Cnww[1]],
                     'Cnd1':  [15, Cnd],      'Cnd2':  [18, -Cnd],
                     'Cnwd1':  [16, Cnwd[0]],     'Cnwd2':  [19, Cnwd[1]],
                     'sigmap': [20, inertia_ratios[0]], 'sigmaq': [21, inertia_ratios[1]], 'sigmar': [22, inertia_ratios[2]],
                     'Clp': [23, Clp], 'Cmq': [24, Cmq], 'Cnr': [25, Cnr]
                    }

        self.eval_pars = eval_pars
        self.param_names = [name for name in self.eval_pars.keys()]
        self.d0 = d0

        # make sure theta has same length as param_names. Use indices in first elements of the lists in eval_pars dict
        self.theta = np.asarray(theta, dtype=float).reshape(-1)
        if self.theta.size != len(self.param_names):
            print(f"Warning: theta has {self.theta.size} elements, but expected {len(self.param_names)}.")

        self.df = pd.DataFrame(columns=self.param_names)

        # add eval_pars as row with name "GT" and theta as row with name "EST"
        self.df.loc["GT"] = [par_true for _, (_, par_true) in self.eval_pars.items()]
        self.df.loc["EST"] = [self.theta[idx] for _, (idx, _) in self.eval_pars.items()]

        # generate a row that holds typical scale of each parameter for error normalization
        self.df.loc["SCALE"] = [1.0 for _, (_, par_true) in self.eval_pars.items()]

        # imrove this! max in group should be per-axis, and typical values of ww and d should be multiplied
        groups = (
            (["Clww1", "Clww2", "Cmww1", "Cmww2", "Cnww1", "Cnww2"], "max_in_group"),
            (["Cmd1", "Cmd2", "Cnd1", "Cnd2"], "max_in_group"),
            (["Cmdd1", "Cmdd2"], "max_in_group"),
            (["Cmddd1", "Cmddd2"], "max_in_group"),
            (["Cnwd1", "Cnwd2"], "max_in_group"),
            (["sigmap", "sigmaq", "sigmar"], "unity"),
            (["Clp", "Cmq", "Cnr"], "max_in_group"),
        )
        for group in groups:
            if group[1] == "unity":
                continue
            elif group[1] == "max_in_group":
                max_in_group = np.max(np.abs(self.df.loc["GT", group[0]]))
                self.df.loc["SCALE", group[0]] = max_in_group
            else:
                raise ValueError(f"Unknown scale group type {group[1]}")

        # controller-relevant parameters
        self.controller = ["Clww1", "Clww2", "Cmd1", "Cmd2", "Cnd1", "Cnd2"]

    def error_metric(self, only_controller=False):
        if only_controller:
            par_names = self.controller
        else:
            par_names = self.df.columns

        # calculate error as rmse over scale-normalized parameters
        self.df.loc["SCALED_ERRORS"] = 0.0
        self.df.loc["SCALED_ERRORS", par_names] = (self.df.loc["EST", par_names] - self.df.loc["GT", par_names]) / self.df.loc["SCALE", par_names]
        error = np.sqrt(np.mean(self.df.loc["SCALED_ERRORS", par_names] ** 2))
        return error

    def correct_signs(self, only_controller=False):
        if only_controller:
            par_names = self.controller
        else:
            par_names = list(self.df.columns)

        gt = self.df.loc["GT", par_names].to_numpy(dtype=float)
        est = self.df.loc["EST", par_names].to_numpy(dtype=float)

        gt_sign = np.sign(gt)
        est_sign = np.sign(est)

        # Keep zero-sign handling identical to other tooling: 0 matches only 0.
        sign_matches = (gt_sign == est_sign)
        n_correct = int(np.sum(sign_matches))
        n_total = int(len(par_names))
        fraction_correct = float(n_correct / n_total) if n_total > 0 else float("nan")

        self.df.loc["SIGN_MATCH", par_names] = sign_matches.astype(float)
        return {
            "n_correct": n_correct,
            "n_total": n_total,
            "fraction_correct": fraction_correct,
            "matched_parameters": [name for name, ok in zip(par_names, sign_matches) if ok],
            "mismatched_parameters": [name for name, ok in zip(par_names, sign_matches) if not ok],
        }

groundtruth_parameters = Parameters(theta=np.zeros(26), tail=BASE_TAIL)
row = {'logfile': 'groundtruth', "model": "groundtruth"}
row['d0_1'] = float(groundtruth_parameters.d0[0])
row['d0_2'] = float(groundtruth_parameters.d0[1])
for par_name in groundtruth_parameters.param_names:
    row[par_name] = float(groundtruth_parameters.df.loc["GT", par_name])

table = pd.concat([table, pd.DataFrame([row])], ignore_index=True)



def runRls(log: IndiflightLog):
    n = 2
    ns = 2
    run_ls = not args.skip_ls

    # Restrict identification to the inferred learning interval.
    idx_learning_start, idx_learning_end = infer_learning_indices(log.data, nr=n, ns=ns)
    if args.rls_in_recovery is not None:
        recovery_time = args.rls_in_recovery
        recovery_indices = log.data["timeS"] <= (log.data["timeS"].iloc[idx_learning_end] + recovery_time)
        recovery_indices &= (log.data["timeS"] >= log.data["timeS"].iloc[idx_learning_start])
        data = log.data.iloc[recovery_indices].copy()
    else:
        data = log.data.iloc[idx_learning_start:idx_learning_end + 1].copy()

    if data.empty:
        raise ValueError("No data in the inferred learning interval for log {log.name}.")

    N = data.shape[0]
    learning_start_s = float(data["timeS"].iloc[0])
    learning_end_s = float(data["timeS"].iloc[-1])

    t_raw = data["timeS"]                                    .to_numpy()
    O_raw = data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
    a_raw = data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
    w_raw = data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
    dm_raw = data[[f"motor[{i}]" for i in range(n)]]         .to_numpy() + 0.084
    u_raw = data[[f"u[{i}]" for i in range(4)]]              .to_numpy()
    d_raw = data[[f'servo_feedback[{i}]' for i in range(ns)]] .to_numpy()
    # d_raw = np.roll(d_raw, -15, axis=0)
    q_raw = data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()
    v_raw = data[[f"localVel[{i}]" for i in range(3)]]       .to_numpy()

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
    freq_hz = 15
    Of = O.filter("lowpass", order, freq_hz)
    af = a.filter("lowpass", order, freq_hz)
    wf = w.filter("lowpass", order, freq_hz)
    dmf = dm.filter("lowpass", order, freq_hz)
    uf = u.filter("lowpass", order, freq_hz)
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
    phi = 1.0
    eta = np.sqrt(v_norm**2 + phi * O_norm**2) # TODODODOD

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

    rls_acts = []

    ls_act_diff = None
    if run_ls:
        ls_act_diff = LS(26, 3)
        ls_act_diff.setTitle("LS Diff-Moments -- Actuators")
        rls_acts.append(ls_act_diff)

    base_cov = 1e-12 * np.diag([
        1, 1, 1e8, 1, 1, 1e8,
        1, 1, 1e8, 1e7, 1, 1, 1e8, 1e7,
        1, 1, 1e8, 1, 1, 1e8,
        1e9, 1e9, 1e9, 1e8, 1e8, 1e8,
    ])

    # Comment out groups below to remove them from the ablation sweep.
    active_ablation_groups = [
        ("I", [20, 21, 22]),
        ("Phi", [23, 24, 25]),
        ("Ddot", [8, 9, 12, 13]),
        ("Cld_Clwd", [1, 2, 4, 5]),
        ("Wdot", [16, 19]),
    ]

    force_zero_groups = [
        # ("Cld_Clwd", [1, 2, 4, 5]),
    ]

    rls_ablation_models = []
    ls_ablation_models = []
    for enabled_flags in product([False, True], repeat=len(active_ablation_groups)):
        disabled_group_names = [
            group_name
            for (group_name, _), group_enabled in zip(active_ablation_groups, enabled_flags)
            if not group_enabled
        ]
        disabled_indices = sorted([
            idx
            for (_, group_indices), group_enabled in zip(active_ablation_groups, enabled_flags)
            if not group_enabled
            for idx in group_indices
        ])

        if disabled_group_names:
            suffix = f"no {' '.join(disabled_group_names)}"
        else:
            suffix = "all groups enabled"

        rls_model = RLS(26, 3, gamma=1e-11, forgetting=0.9999)
        rls_model.setTitle(f"RLS Moments -- {suffix}")
        rls_model.setCovariance(base_cov.copy())
        rls_ablation_models.append((rls_model, disabled_indices))
        rls_acts.append(rls_model)

        if run_ls:
            ls_model = LS(26, 3)
            ls_model.setTitle(f"LS Moments -- {suffix}")
            ls_ablation_models.append((ls_model, disabled_indices))
            rls_acts.append(ls_model)

    A_act_hist = []
    count = 0
    for ti, di, w2i, wdoti, w2_ai, w2_ti, w2_d_ti, wdot_ti, ddot_ti, ddotdot_ti, Oi, Odoti, eta_Bi, eta_i, ddot_i, ddotdot_i, Odotdiffi, wi, wdiffi, ddiffi, ddotdiffi in \
        tqdm(zip(t, df.y, w2, wdot, w2a, w2t, w2dt, wdott, ddott, ddotdott, Of.y, Of.dot().y, eta_B, eta, df.dot().y, df.dot().dot().y, Of.dot().diff().y, wf.y, wf.diff().y, df.diff().y, df.dot().diff().y), total=len(w2a)):

        y = Odoti
        # first try: actuayors only, negelct aerodynamics
        # T = Cw2 * w2ti  +  Cw2d * w2dti  +  Cwdot * wdotti  +  Cddot * ddti  +  Cddotdot * dddti
        inertia_regs = -np.diag([Oi[1]*Oi[2], Oi[2]*Oi[0], Oi[0]*Oi[1]])

        #  |  w1*w1  |  w1*w1*delta1  |  wdot1  |  w2*w2  |  w2*w2*delta2  |  wdot2  |
        A_act_all = np.zeros((3, 26))
        A_act_all[0, 0:3]   = [w2i[0], w2i[0] * di[0], wdoti[0]]
        A_act_all[0, 3:6]   = [w2i[1], w2i[1] * di[1], wdoti[1]]
        A_act_all[1, 6:10]  = [w2i[0], w2i[0] * di[0], ddot_i[0], ddotdot_i[0]]
        A_act_all[1, 10:14] = [w2i[1], w2i[1] * di[1], ddot_i[1], ddotdot_i[1]]
        A_act_all[2, 14:17] = [w2i[0], w2i[0] * di[0], wdoti[0]]
        A_act_all[2, 17:20] = [w2i[1], w2i[1] * di[1], wdoti[1]]
        A_act_all[:, 20:23] = inertia_regs
        A_act_all[:, 23:26] = - eta_i * np.diag(eta_Bi[3:]) # C_m_w
        A_act_hist.append(A_act_all)

        if run_ls:
            A_act_diff = np.zeros((3, 26))
            A_act_diff[0, 0:3]   = [2*wi[0]*wdiffi[0], 2*wi[0]*di[0]*wdiffi[0] + w2i[0]*ddiffi[0], 0]
            A_act_diff[0, 3:6]   = [2*wi[1]*wdiffi[1], 2*wi[1]*di[1]*wdiffi[1] + w2i[1]*ddiffi[1], 0]
            A_act_diff[1, 6:10]  = [2*wi[0]*wdiffi[0], 2*wi[0]*di[0]*wdiffi[0] + w2i[0]*ddiffi[0], ddotdiffi[0], 0]
            A_act_diff[1, 10:14] = [2*wi[1]*wdiffi[1], 2*wi[1]*di[1]*wdiffi[1] + w2i[1]*ddiffi[1], ddotdiffi[1], 0]
            A_act_diff[2, 14:17] = [2*wi[0]*wdiffi[0], 2*wi[0]*di[0]*wdiffi[0] + w2i[0]*ddiffi[0], 0]
            A_act_diff[2, 17:20] = [2*wi[1]*wdiffi[1], 2*wi[1]*di[1]*wdiffi[1] + w2i[1]*ddiffi[1], 0]
            A_act_diff[:, 20:23] = 0
            A_act_diff[:, 23:26] = 0
            ls_act_diff.newSample(A_act_diff, Odotdiffi, ti)

        for rls_model, disabled_indices in rls_ablation_models:
            A_variant = A_act_all.copy()
            if disabled_indices:
                A_variant[:, disabled_indices] = 0
            rls_model.newSample(A_variant, y, ti)
            rls_model.update()

        if run_ls:
            for ls_model, disabled_indices in ls_ablation_models:
                A_variant = A_act_all.copy()
                if disabled_indices:
                    A_variant[:, disabled_indices] = 0
                for force_zero_group in force_zero_groups:
                    _, group_indices = force_zero_group
                    A_variant[:, group_indices] = 0
                ls_model.newSample(A_variant, y, ti)

            count += 1
            if count > 150 and count % 10 == 0:
                for ls_model, _ in ls_ablation_models:
                    ls_model.update()
                ls_act_diff.update()

        # RLS ablations are handled above via rls_ablation_models.

    if run_ls:
        for ls_model, _ in ls_ablation_models:
            ls_model.update()
        ls_act_diff.update()

    if not args.skip_plots:
        parGroups = [[0,3], [1,4], [2,5],   [6,10], [7,11], [8,12], [9,13],   [14,17], [15,18], [16,19], [20,21,22], [23,24,25]]
        parGroupNames = ["$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                         "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\delta}, q}$", "$C_{\\ddot{\\delta}, q}$",
                         "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                         "$C_{m\\sigma}$",
                         "$C_{m\\omega diag}$"]
        theta_true = np.full(26, np.nan, dtype=np.float64)
        for _, (par_idx, par_true) in groundtruth_parameters.eval_pars.items():
            theta_true[par_idx] = float(par_true)
        truePars = [[float(theta_true[idx]) for idx in group] for group in parGroups]
        for rls in tqdm(rls_acts, desc="Generating plots for models"):
            _ = rls.plotParameters(parGroups=parGroups, truePars=truePars, parGroupNames=parGroupNames, sharey=False, zoomy=False)
            rls.f.savefig(f"{output_path}/{rls.name}_{log.name}_parameters.png", dpi=300)
            plt.close(rls.f)

            for i, axis in enumerate(["Roll", "Pitch", "Yaw"]):
                f, V, c, e, X, Y, t = rls.diagnose(i, output_name=axis)
                f.savefig(f"{output_path}/{rls.name}_diagnose_{axis.lower()}_{log.name}.png", dpi=300)
                plt.close(f)

                np.savetxt(f"{output_path}/{rls.name}_diagnose_V_{axis.lower()}_{log.name}.txt", V)
                np.savetxt(f"{output_path}/{rls.name}_diagnose_e_{axis.lower()}_{log.name}.txt", e)


    def get_d0(rls):
        minCmd1_d0 = rls.theta[9,0]
        Cmd1 = rls.theta[10,0]
        d0_1 = - minCmd1_d0 / Cmd1 if np.abs(Cmd1) > 1e-6 else 0.0

        minCmd2_d0 = rls.theta[12,0]
        Cmd2 = rls.theta[13,0]
        d0_2 = - minCmd2_d0 / Cmd2 if np.abs(Cmd2) > 1e-6 else 0.0

        return float(d0_1), float(d0_2)

    #%% error metrics

    # 1. prediction error (RMSE) over dataset
    def compute_RMSE(rls, A_data, y_data):
        y_preds = rls.predictNew(A_data).squeeze()
        e = y_preds - y_data
        rmse = np.sqrt(np.mean(np.linalg.norm(e, axis=1)**2))
        return float(rmse)


    A_data = A_act_hist
    y_data = Of.dot().y
    for rls in rls_acts:
        row = {'logfile': log.name}
        row["model"] = rls.name
        row["learning_idx_start"] = int(idx_learning_start)
        row["learning_idx_end"] = int(idx_learning_end)
        row["learning_start_s"] = learning_start_s
        row["learning_end_s"] = learning_end_s

        row["d0_1"], row["d0_2"] = get_d0(rls)

        row["RMSE"] = compute_RMSE(rls, A_data, y_data)
        row["RMSE_rel"] = row["RMSE"] / np.std(Of.dot().y)

        parameter_results = Parameters(theta=rls.theta[:, 0], tail=BASE_TAIL)
        row["param_rmse_all"] = parameter_results.error_metric()
        row["param_rmse_controller"] = parameter_results.error_metric(only_controller=True)
        signs_all = parameter_results.correct_signs()
        signs_controller = parameter_results.correct_signs(only_controller=True)
        row["n_correct_signs_all"] = signs_all["n_correct"]
        row["n_signs_all"] = signs_all["n_total"]
        row["sign_fraction_all"] = signs_all["fraction_correct"]
        row["n_correct_signs_controller"] = signs_controller["n_correct"]
        row["n_signs_controller"] = signs_controller["n_total"]
        row["sign_fraction_controller"] = signs_controller["fraction_correct"]

        for par_name, (par_idx, _) in parameter_results.eval_pars.items():

            row[par_name] = parameter_results.df.loc["EST", par_name]
            if hasattr(rls, 'theta_bounds'):
                row[f"{par_name}_lb"] = rls.theta_bounds[par_idx, 0]
                row[f"{par_name}_ub"] = rls.theta_bounds[par_idx, 1]

        global table
        table = pd.concat([table, pd.DataFrame([row])], ignore_index=True)

    return rls_acts

rls_list_list = []
for log in logs:
    rls_list = runRls(log)
    rls_list_list.append(rls_list)


# sort table by logfile, then by first letter of the model name. otherwise keep order intact
table = table.sort_values(by=["logfile", "model"], key=lambda col: col.str[0])

# output table as csv
table.to_csv(f"{output_path}/estimator_comparison_table.csv", index=False)

# summary table that only contains the global error metrics
summary_columns = ['logfile', 'model', 'RMSE', 'RMSE_rel', 'param_rmse_all', 'param_rmse_controller', 'n_correct_signs_all', 'n_signs_all', 'sign_fraction_all', 'n_correct_signs_controller', 'n_signs_controller', 'sign_fraction_controller']
summary = table[summary_columns]
summary.to_csv(f"{output_path}/estimator_comparison_summary.csv", index=False)

# group summary table by estimator, and compute mean and std of the error metrics across logs
grouped_summary = summary.groupby("model").agg({
    'RMSE': ['mean', 'std'],
    'RMSE_rel': ['mean', 'std'],
    'param_rmse_all': ['mean', 'std'],
    'param_rmse_controller': ['mean', 'std'],
    'n_correct_signs_all': ['mean', 'std'],
    'n_signs_all': ['mean', 'std'],
    'sign_fraction_all': ['mean', 'std'],
    'n_correct_signs_controller': ['mean', 'std'],
    'n_signs_controller': ['mean', 'std'],
})
grouped_summary.to_csv(f"{output_path}/estimator_comparison_grouped_summary.csv")


# todo:
# define the important parameters for control
# compute RMSE over only the important parameters
# compute some sort of a-posteri reproduction error of the models
