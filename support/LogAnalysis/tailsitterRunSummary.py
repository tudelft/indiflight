from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pathlib import Path
import importlib
import json
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from indiflight_log_tools import IndiflightLog

from indiflightPlotter import IndiflightPlotter, IndiflightViewport, IndiflightIndividualSysIdPlotter
from pyFlightPlotter import Tailsitter, BlittedCursor, local_rc
from flight_metrics import (
    infer_learning_indices,
    extract_learning_metrics,
    compute_recovery_throw_metrics,
    compute_log_metrics,
    extract_theta,
)
plt.close('all')
plt.rcParams.update(local_rc)

LOG_ID = 1
OUTPUT_PATH = Path("support/LogAnalysis/tailsitter_run_summary.pkl")
RESET_TIME = True
GROUND_Z_THRESHOLD = -0.2
GROUND_TIME_THRESHOLD = 5.0
RMS_TIME_THRESHOLD = 5.0
EXCLUDE_END_TIME = 0.1

def ensure_pickle_import_path():
    simulation_dir = Path(__file__).resolve().parents[1] / "simulation"
    simulation_dir_str = str(simulation_dir)
    if simulation_dir_str not in sys.path:
        sys.path.insert(0, simulation_dir_str)

    # Pre-import the package so pickle can resolve PyNDIflight class names.
    importlib.import_module("PyNDIflight.crafts")


def build_parser():
    parser = ArgumentParser(
        description="Join tailsitter Monte Carlo logs with sampled parameters and export summary tables.",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--logs",
        type=str,
        required=True,
        help="Directory containing blackbox logs. All .bfl, .BFL, and .bbl files are discovered recursively.",
    )
    parser.add_argument(
        "--params-pkl",
        required=True,
        type=str,
        help="Parameter pickle produced by tailsitterParameterStudy.py.",
    )
    parser.add_argument(
        "--suppress-plots",
        action="store_true",
        help="Disable all plot windows during processing.",
    )
    return parser


def expand_inputs(log_path):
    root = Path(log_path)
    if root.is_file():
        if root.suffix.lower() in {".bfl", ".bbl"}:
            return [str(root)]
        raise FileNotFoundError(f"Unsupported log file type: {root}")

    if not root.exists():
        raise FileNotFoundError(f"Log directory does not exist: {root}")

    files = [str(path) for path in sorted(root.rglob("*")) if path.is_file() and path.suffix.lower() in {".bfl", ".bbl"}]
    if not files:
        raise FileNotFoundError(f"No .bfl/.bbl logs found under: {root}")
    return files


def load_parameter_runs(params_pkl):
    ensure_pickle_import_path()
    payload = pd.read_pickle(params_pkl)
    sampled = payload.get("sampled")
    if sampled is None:
        raise KeyError("Parameter pickle does not contain a 'sampled' list.")
    return payload, sampled

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
                     'Cld1':  [1, Cld],          'Cld2':  [4, Cld],
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

def safe_rms(values):
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(values))))


def channel_columns(data, prefix):
    cols = []
    for col in data.columns:
        if col.startswith(prefix + "[") and col.endswith("]"):
            cols.append(col)
    cols.sort(key=lambda col: int(col[col.find("[") + 1 : col.find("]")] ))
    return cols


def flatten_value(prefix, value, row):
    if isinstance(value, dict):
        for key, sub_value in value.items():
            flatten_value(f"{prefix}_{key}" if prefix else key, sub_value, row)
        return

    if isinstance(value, (list, tuple, np.ndarray)):
        arr = np.asarray(value)
        if arr.dtype.kind in "biufc" and arr.ndim == 1:
            for idx, item in enumerate(arr.tolist()):
                row[f"{prefix}_{idx}"] = item
            return

        if arr.dtype.kind in "biufc" and arr.ndim > 1:
            for index in np.ndindex(arr.shape):
                joined = "_".join(str(i) for i in index)
                row[f"{prefix}_{joined}"] = arr[index].item()
            return

        row[prefix] = json.dumps(arr.tolist())
        return

    if isinstance(value, (np.floating, np.integer)):
        row[prefix] = value.item()
        return

    row[prefix] = value


def sampled_to_row(sampled, sample_idx):
    row = {"sample_idx": sample_idx}
    flatten_value("sample", sampled, row)
    return row


def compute_learning_metrics(log):
    idx_start, idx_end = infer_learning_indices(log.data, nr=2, ns=2, learning_duration_s=0.5)
    return extract_learning_metrics(log.data, idx_start, idx_end)


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()

    log_files = expand_inputs(args.logs)
    payload, sampled_runs = load_parameter_runs(args.params_pkl)

    # if len(log_files) != len(sampled_runs):
    #     raise ValueError(
    #         f"Log count ({len(log_files)}) does not match sampled run count ({len(sampled_runs)})."
    #     )

    rows = []
    for idx, (log_file, sampled, tail) in enumerate(zip(log_files, sampled_runs, payload['sampled_tails'])):
        log = IndiflightLog(log_file, logId=LOG_ID, resetTime=RESET_TIME)

        row = {
            "log_idx": idx,
            "log_file": log_file,
        }
        row.update(
            compute_log_metrics(
                log.data,
                ground_time_threshold=GROUND_TIME_THRESHOLD,
                ground_z_threshold=GROUND_Z_THRESHOLD,
                rms_time_threshold=RMS_TIME_THRESHOLD,
                exclude_end_time=EXCLUDE_END_TIME,
            )
        )
        row["Success"] = not row["hit_ground_after_5s"]
        row.update(compute_learning_metrics(log))
        row.update(compute_recovery_throw_metrics(log.data))
        row.update(sampled_to_row(sampled, idx))
        idx_start, idx_end = infer_learning_indices(log.data, nr=2, ns=2, learning_duration_s=0.5)
        theta, param_names = extract_theta(log.data, idx_end)
        print(theta)

        pars = Parameters(theta, tail)

        row["sample_error"] = sampled.get("error", "") if isinstance(sampled, dict) else ""

        rows.append(row)

        if (not args.suppress_plots) and (not row["Success"]):
            print(f"Run {idx} ({log_file}) failed.")
            fplt = IndiflightPlotter(log.data, name=f"Run {idx} -- Flight Data", Nr=2, Ns=2)
            aplt = IndiflightIndividualSysIdPlotter(log.data, Nr=2, Ns=2, true=None, name=f"Run {idx} -- Individual ID")
            pplt = IndiflightViewport(Tailsitter(), log.data, follow=False, Nr=2, Ns=2, interpolation="previous",
                                      title=f"Run {idx} -- Onboard ID Analysis")
            fplt.connect_viewport(pplt)
            cursor = BlittedCursor(fplt.all_axes + aplt.all_axes, sharex=True)
            plt.show()


    df = pd.DataFrame(rows)
    # change to degrees
    angles = ['tilt0', 'tilt1', 'roll0', 'roll1', 'pitch0', 'pitch1', 'omega0x', 'omega0y', 'omega0z', 'omega1x', 'omega1y', 'omega1z', 'omega0_norm', 'omega1_norm']
    df[angles] = df[angles].apply(lambda x: np.degrees(x))

    # output mean and std of each column (excluding "Filename" and "Firmware Revision") grouped by success
    print()
    print("Summary Statistics:")
    summary = df.groupby(lambda _: "All").agg({col: ["mean", "std"] for col in df.columns if col not in ["log_idx", "log_file", "sample_error"]})
    summary['N'] = len(df)
    bysuccess = df.groupby("Success").agg({col: ["mean", "std"] for col in df.columns if col not in ["log_idx", "log_file", "sample_error"]})
    bysuccess['N'] = df.groupby("Success").size()
    summary = pd.concat([summary, bysuccess])
    print(summary)

    mapping = {
        "N": "N",
        # "Success": "Success",
        "fx_sign_correct_count": "N Signs Correct",
        "omega0_norm": "Start Gyro Norm",
        "tilt0": "Start Tilt",
        "omega1_norm": "End Gyro Norm",
        "tilt1": "End Tilt",
        "p1z": "End Position Z",
        "v1z": "End Velocity Z"
    }
    ltx = summary[mapping.keys()].rename(columns=mapping).round(1).to_latex(
            index=True,
            float_format="%.1f",
            bold_rows=True,
            multicolumn=True,
            multirow=True
    )
    print()
    print(ltx)

    output_path = OUTPUT_PATH
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_pickle(output_path)
    df.to_csv(output_path.with_suffix(".csv"), index=False)

    print(f"Wrote dataframe to {output_path}")
    print(f"Wrote CSV to {output_path.with_suffix('.csv')}")
    printcols = ["log_idx", "log_file", "Success", "omega0_norm", "tilt0", "omega1_norm", "tilt1", "p1z", "v1z", "max_lateral_extend_5_10s", "max_throw_height_4p5_5s"]
    print(df[printcols])
    print(summary[printcols[2:]])
