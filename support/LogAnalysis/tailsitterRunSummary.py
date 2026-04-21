from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pathlib import Path
import glob
import importlib
import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from indiflight_log_tools import IndiflightLog

from indiflightPlotter import IndiflightPlotter, IndiflightViewport, IndiflightIndividualSysIdPlotter
from pyFlightPlotter import Tailsitter, BlittedCursor, local_rc
import matplotlib.pyplot as plt
plt.close('all')
plt.rcParams.update(local_rc)

LOG_ID = 1
OUTPUT_PATH = Path("support/LogAnalysis/tailsitter_run_summary.pkl")
RESET_TIME = True
GROUND_Z_THRESHOLD = -0.2
GROUND_TIME_THRESHOLD = 5.0
RMS_TIME_THRESHOLD = 5.0
EXCLUDE_END_TIME = 0.1

ACT_TRUE = {
    'motor_2_rls_x[0]': 1.75,
    'motor_2_rls_x[1]': 0.0,
    'motor_2_rls_x[2]': 0.03,
    'motor_2_rls_x[3]': 0.,

    'motor_3_rls_x[0]': 1.75,
    'motor_3_rls_x[1]': 0.0,
    'motor_3_rls_x[2]': 0.03,
    'motor_3_rls_x[3]': 0.,

    'fx_p_rls_x[0]': (1.413e-6 * 0.106) / 5.735e-3,
    'fx_p_rls_x[1]': -(1.413e-6 * 0.106) / 5.735e-3,
    'fx_p_rls_x[2]': 0,
    'fx_p_rls_x[3]': 0,

    'fx_q_rls_x[0]': 0,
    'fx_q_rls_x[1]': 0,
    'fx_q_rls_x[2]': -1.836e-8 / 1.345e-3,
    'fx_q_rls_x[3]': -1.836e-8 / 1.345e-3,
    'fx_q_rls_x[8]': -1.411e-3 / 1.345e-3,
    'fx_q_rls_x[9]': -1.411e-3 / 1.345e-3,

    'fx_r_rls_x[0]': -1.413e-6 * 6.157e-4 / 5.413e-3,
    'fx_r_rls_x[1]': +1.413e-6 * 6.157e-4 / 5.413e-3,
    'fx_r_rls_x[2]': -6.061e-8 / 5.413e-3,
    'fx_r_rls_x[3]': +6.061e-8 / 5.413e-3,
    'fx_r_rls_x[8]': -2.840e-6 / 5.413e-3,
    'fx_r_rls_x[9]': +2.840e-6 / 5.413e-3,

    'sigma_rls[0]': 0.75,
    'sigma_rls[1]': -0.25,
    'sigma_rls[2]': -0.61538,
}

fx_true = {k: v for k, v in ACT_TRUE.items() if k.startswith("fx_") and "_rls_" in k}

sign_eval_keys = [
    "fx_p_rls_x[0]", "fx_p_rls_x[1]",
    "fx_q_rls_x[2]", "fx_q_rls_x[3]",
    "fx_r_rls_x[2]", "fx_r_rls_x[3]",
]

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


class suppress_output:
    def __enter__(self):
        self._stdout_fd = os.dup(1)
        self._stderr_fd = os.dup(2)

        self._devnull = os.open(os.devnull, os.O_WRONLY)

        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.dup2(self._stdout_fd, 1)
        os.dup2(self._stderr_fd, 2)

        os.close(self._devnull)
        os.close(self._stdout_fd)
        os.close(self._stderr_fd)


def compute_log_metrics(log):
    data = log.data
    t = data["timeS"].to_numpy(dtype=np.float64)
    z = data["pos[2]"].to_numpy(dtype=np.float64) if "pos[2]" in data.columns else np.array([], dtype=np.float64)

    ground_mask = (t >= GROUND_TIME_THRESHOLD) & (t <= t[-1] - EXCLUDE_END_TIME)
    hit_ground_after_threshold = bool(np.any(z[ground_mask] > GROUND_Z_THRESHOLD)) if z.size else False

    rms_mask = t >= RMS_TIME_THRESHOLD
    motor_cols = channel_columns(data, "motor")
    servo_cols = channel_columns(data, "servo_feedback")

    motor_rms = float("nan")
    motor_rms_by_channel = []
    if motor_cols:
        motor_values = data[motor_cols].to_numpy(dtype=np.float64)
        motor_rms_by_channel = [safe_rms(motor_values[rms_mask, idx]) for idx in range(motor_values.shape[1])]
        motor_rms = safe_rms(motor_values[rms_mask].ravel())

    servo_rms = float("nan")
    servo_rms_by_channel = []
    if servo_cols:
        servo_values = data[servo_cols].to_numpy(dtype=np.float64)
        servo_rms_by_channel = [safe_rms(servo_values[rms_mask, idx]) for idx in range(servo_values.shape[1])]
        servo_rms = safe_rms(servo_values[rms_mask].ravel())

    row = {
        "start_time_s": float(t[0]) if t.size else float("nan"),
        "end_time_s": float(t[-1]) if t.size else float("nan"),
        "hit_ground_after_5s": hit_ground_after_threshold,
        "motor_rms_after_5s": motor_rms,
        "servo_rms_after_5s": servo_rms,
        "motor_rms_ch0": motor_rms_by_channel[0] if len(motor_rms_by_channel) > 0 else float("nan"),
        "motor_rms_ch1": motor_rms_by_channel[1] if len(motor_rms_by_channel) > 1 else float("nan"),
        "servo_rms_ch0": servo_rms_by_channel[0] if len(servo_rms_by_channel) > 0 else float("nan"),
        "servo_rms_ch1": servo_rms_by_channel[1] if len(servo_rms_by_channel) > 1 else float("nan"),
    }
    return row


def quaternion_tilt(data, index):
    keys = [f"ekf_quat[{i}]" for i in range(4)]
    if all(key in data.columns for key in keys):
        x = float(data[keys[1]].iloc[index])
        y = float(data[keys[2]].iloc[index])
        return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))

    return float("nan")


def quaternion_roll_pitch(data, index):
    keys = [f"ekf_quat[{i}]" for i in range(4)]
    if all(key in data.columns for key in keys):
        w = float(data[keys[0]].iloc[index])
        x = float(data[keys[1]].iloc[index])
        y = float(data[keys[2]].iloc[index])
        z = float(data[keys[3]].iloc[index])

        roll = float(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        pitch = float(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
        return roll, pitch

    return float("nan"), float("nan")


def compute_learning_metrics(log):
    with suppress_output():
        plt.close('all')
        aplt = IndiflightIndividualSysIdPlotter(log.data, Nr=2, Ns=2, true=None, name="Onboard Individual Analysis")

    idx_start = aplt.idx_start_learning
    idx_end = aplt.idx_end_learning
    data = log.data

    fx_sq_errors = []
    fx_missing = []
    for key, true_val in fx_true.items():
        if key in data.columns:
            pred_val = float(data[key].iloc[idx_end])
            fx_sq_errors.append((pred_val - true_val) ** 2)
        else:
            fx_missing.append(key)

    sign_correct_count = 0
    sign_terms_checked = 0
    sign_terms_missing = 0
    for key in sign_eval_keys:
        if key in data.columns and key in ACT_TRUE:
            pred_val = float(data[key].iloc[idx_end])
            true_val = float(ACT_TRUE[key])
            sign_correct_count += int(np.sign(pred_val) == np.sign(true_val))
            sign_terms_checked += 1
        else:
            sign_terms_missing += 1

    return aplt, {
        "p0x": float(data["pos[0]"].iloc[idx_start]) if "pos[0]" in data.columns else float("nan"),
        "p0y": float(data["pos[1]"].iloc[idx_start]) if "pos[1]" in data.columns else float("nan"),
        "p0z": float(data["pos[2]"].iloc[idx_start]) if "pos[2]" in data.columns else float("nan"),
        "v0x": float(data["vel[0]"].iloc[idx_start]) if "vel[0]" in data.columns else float("nan"),
        "v0y": float(data["vel[1]"].iloc[idx_start]) if "vel[1]" in data.columns else float("nan"),
        "v0z": float(data["vel[2]"].iloc[idx_start]) if "vel[2]" in data.columns else float("nan"),
        "omega0x": float(data["gyroADCafterRpm[0]"].iloc[idx_start]) if "gyroADCafterRpm[0]" in data.columns else float("nan"),
        "omega0y": float(data["gyroADCafterRpm[1]"].iloc[idx_start]) if "gyroADCafterRpm[1]" in data.columns else float("nan"),
        "omega0z": float(data["gyroADCafterRpm[2]"].iloc[idx_start]) if "gyroADCafterRpm[2]" in data.columns else float("nan"),
        "omega0_norm": float(np.linalg.norm([
            float(data["gyroADCafterRpm[0]"].iloc[idx_start]) if "gyroADCafterRpm[0]" in data.columns else 0.0,
            float(data["gyroADCafterRpm[1]"].iloc[idx_start]) if "gyroADCafterRpm[1]" in data.columns else 0.0,
            float(data["gyroADCafterRpm[2]"].iloc[idx_start]) if "gyroADCafterRpm[2]" in data.columns else 0.0,
        ])),
        "tilt0": quaternion_tilt(data, idx_start),
        "roll0": quaternion_roll_pitch(data, idx_start)[0],
        "pitch0": quaternion_roll_pitch(data, idx_start)[1],
        "p1x": float(data["pos[0]"].iloc[idx_end]) if "pos[0]" in data.columns else float("nan"),
        "p1y": float(data["pos[1]"].iloc[idx_end]) if "pos[1]" in data.columns else float("nan"),
        "p1z": float(data["pos[2]"].iloc[idx_end]) if "pos[2]" in data.columns else float("nan"),
        "v1x": float(data["vel[0]"].iloc[idx_end]) if "vel[0]" in data.columns else float("nan"),
        "v1y": float(data["vel[1]"].iloc[idx_end]) if "vel[1]" in data.columns else float("nan"),
        "v1z": float(data["vel[2]"].iloc[idx_end]) if "vel[2]" in data.columns else float("nan"),
        "omega1x": float(data["gyroADCafterRpm[0]"].iloc[idx_end]) if "gyroADCafterRpm[0]" in data.columns else float("nan"),
        "omega1y": float(data["gyroADCafterRpm[1]"].iloc[idx_end]) if "gyroADCafterRpm[1]" in data.columns else float("nan"),
        "omega1z": float(data["gyroADCafterRpm[2]"].iloc[idx_end]) if "gyroADCafterRpm[2]" in data.columns else float("nan"),
        "omega1_norm": float(np.linalg.norm([
            float(data["gyroADCafterRpm[0]"].iloc[idx_end]) if "gyroADCafterRpm[0]" in data.columns else 0.0,
            float(data["gyroADCafterRpm[1]"].iloc[idx_end]) if "gyroADCafterRpm[1]" in data.columns else 0.0,
            float(data["gyroADCafterRpm[2]"].iloc[idx_end]) if "gyroADCafterRpm[2]" in data.columns else 0.0,
        ])),
        "tilt1": quaternion_tilt(data, idx_end),
        "roll1": quaternion_roll_pitch(data, idx_end)[0],
        "pitch1": quaternion_roll_pitch(data, idx_end)[1],
        "fx_sign_current_count": sign_correct_count,
        "fx_sign_correct_count": sign_correct_count,
        "fx_sign_terms_checked": sign_terms_checked,
        "fx_sign_terms_missing": sign_terms_missing,
        "fx_terms_used": len(fx_sq_errors),
        "fx_terms_missing": len(fx_missing),
    }


def compute_recovery_throw_metrics(log):
    data = log.data
    t = data["timeS"].to_numpy(dtype=np.float64) if "timeS" in data.columns else np.array([], dtype=np.float64)
    if t.size == 0:
        return {
            "max_lateral_extend_5_10s": float("nan"),
            "max_throw_height_4p5_5s": float("nan"),
        }

    max_lateral_extend = float("nan")
    if "pos[0]" in data.columns and "pos[1]" in data.columns:
        x = data["pos[0]"].to_numpy(dtype=np.float64)
        y = data["pos[1]"].to_numpy(dtype=np.float64)
        recovery_mask = (t >= 5.0) & (t <= 10.0)
        if np.any(recovery_mask):
            start_idx = int(np.flatnonzero(recovery_mask)[0])
            lateral = np.hypot(x - x[start_idx], y - y[start_idx])
            max_lateral_extend = float(np.max(lateral[recovery_mask]))

    max_throw_height = float("nan")
    if "pos[2]" in data.columns:
        z = data["pos[2]"].to_numpy(dtype=np.float64)
        throw_mask = (t >= 4.5) & (t <= 5.0)
        if np.any(throw_mask):
            start_idx = int(np.flatnonzero(throw_mask)[0])
            # z is positive downward, so upward throw height is a decrease in z.
            max_throw_height = float(z[start_idx] - np.min(z[throw_mask]))

    return {
        "max_lateral_extend_5_10s": max_lateral_extend,
        "max_throw_height_4p5_5s": max_throw_height,
    }


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
    for idx, (log_file, sampled) in enumerate(zip(log_files, sampled_runs)):
        log = IndiflightLog(log_file, logId=LOG_ID, resetTime=RESET_TIME)

        row = {
            "log_idx": idx,
            "log_file": log_file,
        }
        row.update(compute_log_metrics(log))
        row["Success"] = not row["hit_ground_after_5s"]
        aplt, crow = compute_learning_metrics(log)
        row.update(crow)
        row.update(compute_recovery_throw_metrics(log))
        row.update(sampled_to_row(sampled, idx))

        row["sample_error"] = sampled.get("error", "") if isinstance(sampled, dict) else ""

        rows.append(row)

        if (not args.suppress_plots) and (not row["Success"]):
            print(f"Run {idx} ({log_file}) failed.")
            fplt = IndiflightPlotter(log.data, name=f"Run {idx} -- Flight Data", Nr=2, Ns=2)
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
        "Success": "Success",
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
    print(df[["log_idx", "log_file", "Success", "hit_ground_after_5s", "fx_sign_current_count", "omega0_norm", "tilt0", "omega1_norm", "tilt1", "p1z", "v1z", "max_lateral_extend_5_10s", "max_throw_height_4p5_5s"]])
