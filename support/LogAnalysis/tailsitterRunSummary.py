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
    for idx, (log_file, sampled) in enumerate(zip(log_files, sampled_runs)):
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
    printcols = ["log_idx", "log_file", "Success", "omega0_norm", "tilt0", "omega1_norm", "tilt1", "p1z", "v1z", "max_lateral_extend_5_10s", "max_throw_height_4p5_5s"]
    print(df[printcols])
    print(summary[printcols[2:]])
