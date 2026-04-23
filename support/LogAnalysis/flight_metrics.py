import os
import numpy as np


ACT_TRUE = {
    "motor_2_rls_x[0]": 1.75,
    "motor_2_rls_x[1]": 0.0,
    "motor_2_rls_x[2]": 0.03,
    "motor_2_rls_x[3]": 0.0,
    "motor_3_rls_x[0]": 1.75,
    "motor_3_rls_x[1]": 0.0,
    "motor_3_rls_x[2]": 0.03,
    "motor_3_rls_x[3]": 0.0,
    "fx_p_rls_x[0]": (1.413e-6 * 0.106) / 5.735e-3,
    "fx_p_rls_x[1]": -(1.413e-6 * 0.106) / 5.735e-3,
    "fx_p_rls_x[2]": 0.0,
    "fx_p_rls_x[3]": 0.0,
    "fx_q_rls_x[0]": 0.0,
    "fx_q_rls_x[1]": 0.0,
    "fx_q_rls_x[2]": -1.836e-8 / 1.345e-3,
    "fx_q_rls_x[3]": -1.836e-8 / 1.345e-3,
    "fx_q_rls_x[8]": -1.411e-3 / 1.345e-3,
    "fx_q_rls_x[9]": -1.411e-3 / 1.345e-3,
    "fx_r_rls_x[0]": -1.413e-6 * 6.157e-4 / 5.413e-3,
    "fx_r_rls_x[1]": +1.413e-6 * 6.157e-4 / 5.413e-3,
    "fx_r_rls_x[2]": -6.061e-8 / 5.413e-3,
    "fx_r_rls_x[3]": +6.061e-8 / 5.413e-3,
    "fx_r_rls_x[8]": -2.840e-6 / 5.413e-3,
    "fx_r_rls_x[9]": +2.840e-6 / 5.413e-3,
    "sigma_rls[0]": 0.75,
    "sigma_rls[1]": -0.25,
    "sigma_rls[2]": -0.61538,
}

FX_TRUE = {k: v for k, v in ACT_TRUE.items() if k.startswith("fx_") and "_rls_" in k}

SIGN_EVAL_KEYS = [
    "fx_p_rls_x[0]",
    "fx_p_rls_x[1]",
    "fx_q_rls_x[2]",
    "fx_q_rls_x[3]",
    "fx_r_rls_x[2]",
    "fx_r_rls_x[3]",
]


def infer_learning_indices(data, nr=2, ns=2, learning_duration_s=0.5):
    t = data["timeS"].to_numpy(dtype=np.float64)
    if t.size == 0:
        return 0, 0

    n = nr + ns
    p = np.array([data[f"fx_p_rls_x[{i}]"].to_numpy(dtype=np.float64) for i in range(n)])
    q = np.array([data[f"fx_q_rls_x[{i}]"].to_numpy(dtype=np.float64) for i in range(n)])
    r = np.array([data[f"fx_r_rls_x[{i}]"].to_numpy(dtype=np.float64) for i in range(n)])
    d = np.array([data[f"servo_feedback[{i}]"].to_numpy(dtype=np.float64) for i in range(ns)]) if ns > 0 else np.zeros((0, t.size), dtype=np.float64)

    eff = np.zeros((6, n, t.size), dtype=np.float64)
    if nr > 0:
        eff[3, :nr] = p[:nr]
        if ns > 0:
            sd = np.sin(d)
            cd = np.cos(d)
            eff[4, :nr] = q[:nr] + q[nr:n] * sd
            eff[5, :nr] = r[:nr] + r[nr:n] * sd
            eff[4, nr:n] = q[nr:n] * cd
            eff[5, nr:n] = r[nr:n] * cd
        else:
            eff[4, :nr] = q[:nr]
            eff[5, :nr] = r[:nr]

    nonzero = np.flatnonzero(np.any(eff != 0.0, axis=(0, 1)))
    if nonzero.size == 0:
        return 0, min(len(t) - 1, max(0, int(round(learning_duration_s / (t[1] - t[0]))) if len(t) > 1 else 0))

    idx_start = int(nonzero[0])
    t_end = t[idx_start] + learning_duration_s
    idx_end = int(np.argmin(np.abs(t - t_end)))
    return idx_start, idx_end


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


def compute_fx_fit_metrics(data, idx_end):
    fx_sq_errors = []
    fx_missing = []
    for key, true_val in FX_TRUE.items():
        if key in data.columns:
            pred_val = float(data[key].iloc[idx_end])
            fx_sq_errors.append((pred_val - true_val) ** 2)
        else:
            fx_missing.append(key)

    fx_mse = float(np.mean(fx_sq_errors)) if fx_sq_errors else float("nan")
    fx_rmse = float(np.sqrt(fx_mse)) if fx_sq_errors else float("nan")

    sign_correct_count = 0
    sign_terms_checked = 0
    sign_terms_missing = 0
    for key in SIGN_EVAL_KEYS:
        if key in data.columns and key in ACT_TRUE:
            pred_val = float(data[key].iloc[idx_end])
            true_val = float(ACT_TRUE[key])
            sign_correct_count += int(np.sign(pred_val) == np.sign(true_val))
            sign_terms_checked += 1
        else:
            sign_terms_missing += 1

    return {
        "fx_mse_end_learning": fx_mse,
        "fx_rmse_end_learning": fx_rmse,
        "fx_terms_used": len(fx_sq_errors),
        "fx_terms_missing": len(fx_missing),
        # Backward-compatible alias kept for existing scripts.
        "fx_sign_current_count": sign_correct_count,
        "fx_sign_correct_count": sign_correct_count,
        "fx_sign_terms_checked": sign_terms_checked,
        "fx_sign_terms_missing": sign_terms_missing,
    }


def extract_learning_metrics(data, idx_start, idx_end):
    row = {
        "p0x": float(data["pos[0]"].iloc[idx_start]) if "pos[0]" in data.columns else float("nan"),
        "p0y": float(data["pos[1]"].iloc[idx_start]) if "pos[1]" in data.columns else float("nan"),
        "p0z": float(data["pos[2]"].iloc[idx_start]) if "pos[2]" in data.columns else float("nan"),
        "v0x": float(data["vel[0]"].iloc[idx_start]) if "vel[0]" in data.columns else float("nan"),
        "v0y": float(data["vel[1]"].iloc[idx_start]) if "vel[1]" in data.columns else float("nan"),
        "v0z": float(data["vel[2]"].iloc[idx_start]) if "vel[2]" in data.columns else float("nan"),
        "omega0x": float(data["gyroADCafterRpm[0]"].iloc[idx_start]) if "gyroADCafterRpm[0]" in data.columns else float("nan"),
        "omega0y": float(data["gyroADCafterRpm[1]"].iloc[idx_start]) if "gyroADCafterRpm[1]" in data.columns else float("nan"),
        "omega0z": float(data["gyroADCafterRpm[2]"].iloc[idx_start]) if "gyroADCafterRpm[2]" in data.columns else float("nan"),
        "omega0_norm": float(
            np.linalg.norm(
                [
                    float(data["gyroADCafterRpm[0]"].iloc[idx_start]) if "gyroADCafterRpm[0]" in data.columns else 0.0,
                    float(data["gyroADCafterRpm[1]"].iloc[idx_start]) if "gyroADCafterRpm[1]" in data.columns else 0.0,
                    float(data["gyroADCafterRpm[2]"].iloc[idx_start]) if "gyroADCafterRpm[2]" in data.columns else 0.0,
                ]
            )
        ),
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
        "omega1_norm": float(
            np.linalg.norm(
                [
                    float(data["gyroADCafterRpm[0]"].iloc[idx_end]) if "gyroADCafterRpm[0]" in data.columns else 0.0,
                    float(data["gyroADCafterRpm[1]"].iloc[idx_end]) if "gyroADCafterRpm[1]" in data.columns else 0.0,
                    float(data["gyroADCafterRpm[2]"].iloc[idx_end]) if "gyroADCafterRpm[2]" in data.columns else 0.0,
                ]
            )
        ),
        "tilt1": quaternion_tilt(data, idx_end),
        "roll1": quaternion_roll_pitch(data, idx_end)[0],
        "pitch1": quaternion_roll_pitch(data, idx_end)[1],
    }
    row.update(compute_fx_fit_metrics(data, idx_end))
    return row


def compute_recovery_throw_metrics(data):
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
            max_throw_height = float(z[start_idx] - np.min(z[throw_mask]))

    return {
        "max_lateral_extend_5_10s": max_lateral_extend,
        "max_throw_height_4p5_5s": max_throw_height,
    }


def channel_columns(data, prefix):
    cols = []
    for col in data.columns:
        if col.startswith(prefix + "[") and col.endswith("]"):
            cols.append(col)
    cols.sort(key=lambda col: int(col[col.find("[") + 1 : col.find("]")]))
    return cols


def safe_rms(values):
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(values))))


def compute_log_metrics(
    data,
    ground_time_threshold=5.0,
    ground_z_threshold=-0.2,
    rms_time_threshold=5.0,
    exclude_end_time=0.1,
):
    t = data["timeS"].to_numpy(dtype=np.float64)
    z = data["pos[2]"].to_numpy(dtype=np.float64) if "pos[2]" in data.columns else np.array([], dtype=np.float64)

    ground_mask = (t >= ground_time_threshold) & (t <= t[-1] - exclude_end_time)
    hit_ground_after_threshold = bool(np.any(z[ground_mask] > ground_z_threshold)) if z.size else False

    rms_mask = t >= rms_time_threshold
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

    return {
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
