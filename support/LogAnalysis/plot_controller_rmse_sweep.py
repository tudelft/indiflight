from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ACTIVE_GROUPS = ("I", "Phi", "Ddot", "Cld_Clwd", "Wdot")


def build_parser():
    parser = ArgumentParser(
        description="Plot param_rmse_controller across grouped-summary CSV files.",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "directory",
        type=str,
        help="Directory containing estimator_comparison_grouped_summary_*.csv files.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output image path. Defaults to <directory>/param_rmse_controller_sweep.png.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search for CSV files recursively.",
    )
    return parser


def load_grouped_summary(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path, header=[0, 1], index_col=0)
    if not isinstance(df.columns, pd.MultiIndex):
        raise ValueError(f"Expected a grouped-summary CSV with a two-row header: {csv_path}")
    if ("param_rmse_controller", "mean") not in df.columns or ("param_rmse_controller", "std") not in df.columns:
        raise KeyError(f"CSV is missing param_rmse_controller mean/std columns: {csv_path}")
    return df


def parse_disabled_groups(model_name: str) -> tuple[str, ...] | None:
    if model_name == "groundtruth" or "--" not in model_name:
        return None

    suffix = model_name.split("--", 1)[1].strip()
    if suffix == "all groups enabled":
        return ()
    if not suffix.startswith("no "):
        return None

    disabled_groups = tuple(suffix[3:].split())
    return disabled_groups


def enabled_label_from_model(model_name: str) -> str | None:
    disabled_groups = parse_disabled_groups(model_name)
    if disabled_groups is None:
        return None

    enabled_groups = [group for group in ACTIVE_GROUPS if group not in disabled_groups]
    if not enabled_groups:
        return "actuators only"
    if len(enabled_groups) == len(ACTIVE_GROUPS):
        return "all groups enabled"
    return "\n".join(enabled_groups)


def should_skip_model(model_name: str) -> bool:
    disabled_groups = parse_disabled_groups(model_name)
    if disabled_groups is None:
        return True
    enabled_groups = [group for group in ACTIVE_GROUPS if group not in disabled_groups]
    return any(group in {"Cld_Clwd", "Wdot"} for group in enabled_groups)


def label_sort_key(label: str) -> tuple[int, str]:
    if label == "all groups enabled":
        return (0, label)
    if label == "none":
        return (len(ACTIVE_GROUPS) + 1, label)
    enabled_count = label.count("\n") + 1
    return (len(ACTIVE_GROUPS) - enabled_count + 1, label.replace("\n", " "))


def collect_series(csv_paths: list[Path]):
    series_by_file = []
    all_labels = set()

    for csv_path in csv_paths:
        df = load_grouped_summary(csv_path)
        rows = []

        for model_name, row in df.iterrows():
            label = enabled_label_from_model(str(model_name))
            if label is None or should_skip_model(str(model_name)):
                continue

            controller_mean = float(row[("param_rmse_controller", "mean")])
            controller_std = float(row[("param_rmse_controller", "std")])
            reproduction_mean = float(row[("RMSE", "mean")])
            reproduction_std = float(row[("RMSE", "std")])
            rows.append((label, controller_mean, controller_std, reproduction_mean, reproduction_std))
            all_labels.add(label)

        rows.sort(key=lambda item: label_sort_key(item[0]))
        series_by_file.append((csv_path.stem, rows))

    ordered_labels = sorted(all_labels, key=label_sort_key)
    return series_by_file, ordered_labels


def main():
    args = build_parser().parse_args()
    input_dir = Path(args.directory)
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Not a directory: {input_dir}")

    pattern = "**/*.csv" if args.recursive else "*.csv"
    csv_paths = sorted(
        path for path in input_dir.glob(pattern) if path.is_file()
    )
    if not csv_paths:
        raise FileNotFoundError(f"No CSV files found in {input_dir}")

    series_by_file, ordered_labels = collect_series(csv_paths)
    x_positions = np.arange(len(ordered_labels))

    fig, (ax, ax_rmse) = plt.subplots(1, 2, figsize=(2 * max(8, len(ordered_labels) * 0.2), 6))

    n_series = len(series_by_file)
    series_offset_scale = 0.15 if n_series > 1 else 0.0
    
    for series_idx, (series_name, rows) in enumerate(series_by_file):
        x_offset = (series_idx - (n_series - 1) / 2.0) * series_offset_scale
        values_by_label = {
            label: (controller_mean, controller_std, reproduction_mean, reproduction_std)
            for label, controller_mean, controller_std, reproduction_mean, reproduction_std in rows
        }
        controller_values = []
        controller_errors = []
        reproduction_values = []
        reproduction_errors = []
        for label in ordered_labels:
            if label in values_by_label:
                controller_mean, controller_std, reproduction_mean, reproduction_std = values_by_label[label]
                controller_values.append(controller_mean)
                controller_errors.append(controller_std)
                reproduction_values.append(reproduction_mean)
                reproduction_errors.append(reproduction_std)
            else:
                controller_values.append(np.nan)
                controller_errors.append(np.nan)
                reproduction_values.append(np.nan)
                reproduction_errors.append(np.nan)

        controller_line = ax.errorbar(
            x_positions + x_offset,
            controller_values,
            yerr=controller_errors,
            linestyle='none',
            marker="o",
            capsize=3,
            label=f"{series_name}",
        )
        ax_rmse.errorbar(
            x_positions + x_offset,
            reproduction_values,
            yerr=reproduction_errors,
            marker="s",
            linestyle="--",
            linewidth=1.6,
            capsize=3,
            color=controller_line[0].get_color(),
            label=f"{series_name}",
        )

    ax.set_xticks(x_positions)
    ax.set_xticklabels(ordered_labels, rotation=45, ha="right")
    ax.set_ylabel("param_rmse_controller")
    ax.set_xlabel("Enabled groups")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(title="Series", loc="best")

    ax_rmse.set_xticks(x_positions)
    ax_rmse.set_xticklabels(ordered_labels, rotation=45, ha="right")
    ax_rmse.set_ylabel("RMSE")
    ax_rmse.set_xlabel("Enabled groups")
    ax_rmse.grid(True, axis="y", alpha=0.3)
    ax_rmse.legend(title="Series", loc="best")

    fig.tight_layout()

    output_path = Path(args.output) if args.output else input_dir / "param_rmse_controller_sweep.eps"
    fig.savefig(output_path, dpi=300, format='eps')
    print(f"Saved plot to {output_path}")


if __name__ == "__main__":
    main()