
from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt
import matplotlib

plt.close('all')
matplotlib.use('tkagg') 

from pyFlightPlotter import local_rc
from indiflightPlotter import IndiflightPlotter, \
    IndiflightMotorSysIdPlotter, \
    IndiflightServoSysIdPlotter, \
    IndiflightViewport, \
    IndiflightIndividualSysIdPlotter, \
    IndiflightMoments, \
    IndiflightEffectiveness
from pyFlightPlotter import Quadrotor, Tailsitter, BlittedCursor

# set local_rc
plt.rcParams.update(local_rc)

import os
import sys
import numpy as np

act_true = {
    'motor_2_rls_x[0]': 1.75,    # max angle in rad
    'motor_2_rls_x[1]': 0.0,     # neutral angle in rad
    'motor_2_rls_x[2]': 0.03,    # delay in seconds
    'motor_2_rls_x[3]': 0.,      # time constant in seconds

    'motor_3_rls_x[0]': 1.75,    # max angle in rad
    'motor_3_rls_x[1]': 0.0,     # neutral angle in rad
    'motor_3_rls_x[2]': 0.03,    # delay in seconds
    'motor_3_rls_x[3]': 0.,      # time constant in seconds

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

fx_true = {k: v for k, v in act_true.items() if k.startswith("fx_") and "_rls_" in k}

# Sign-check subset requested by user.
sign_eval_keys = [
    "fx_p_rls_x[0]", "fx_p_rls_x[1]",
    "fx_q_rls_x[2]", "fx_q_rls_x[3]",
    "fx_r_rls_x[2]", "fx_r_rls_x[3]"
]

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

path = "/mnt/data/WorkData/BlackboxLogs/"
# files = [
#     "2025-12-17/LOG00250_success.BFL",
#     "2025-12-17/LOG00251_omg.BFL",
#     "2025-12-18/LOG00252_success_under_net.BFL",
#     "2025-12-18/LOG00253_success.BFL",
#     "2025-12-18/LOG00254_success.BFL",
#     "2025-12-18/LOG00255_crash.BFL",
#     "2025-12-18/LOG00256_success.BFL",
#     "2026-01-05/LOG00259_crash_demo1_wrongMocap.BFL",
#     "2026-01-05/LOG00260_crash_demo2_wrongMocap.BFL",
#     "2026-01-05/LOG00263_success.BFL",
#     "2026-01-13/LOG00270_crash.BFL",
#     "2026-01-13/LOG00271_crash.BFL",
#     "2026-01-13/LOG00272_crash_wall.BFL",
#     "2026-01-26/LOG00275_crash.BFL",
# ]
files = [
    ("2026-02-16/bat1vid1/LOG00296_badThrowCrash.BFL", False),
    ("2026-02-16/bat1vid1/LOG00297.BFL", True),
    ("2026-02-16/bat1vid1/LOG00298.BFL", True),
    ("2026-02-16/bat1vid1/LOG00299.BFL", True),
    ("2026-02-16/bat1vid1/LOG00300.BFL", True),
    ("2026-02-16/bat1vid1/LOG00301_crash.BFL", False),
    ("2026-02-16/bat2vid2/LOG00302.BFL", True),
    ("2026-02-16/bat2vid2/LOG00303.BFL", True),
    ("2026-02-16/bat2vid2/LOG00304.BFL", True),
    ("2026-02-16/bat2vid2/LOG00305_crashFlatSpin.BFL", False),
    ("2026-02-16/bat2vid3/LOG00306.BFL", True),
    ("2026-02-16/bat2vid3/LOG00307_crashMaybeLowThrow.BFL", False),
    ("2026-02-16/bat3vid4/LOG00308.BFL", True),
    ("2026-02-16/bat3vid4/LOG00309_crash.BFL", False),
    ("2026-02-16/bat3vid4/LOG00310.BFL", True),
    ("2026-02-16/bat3vid4/LOG00311.BFL", True),
    ("2026-02-16/bat3vid4/LOG00312.BFL", True),
    ("2026-02-16/bat3vid4/LOG00313.BFL", True),
    ("2026-02-16/bat3vid4/LOG00314.BFL", True),
]

import pandas as pd
df = pd.DataFrame(columns=["Filename", "Firmware Revision", "Success",
                           "p0x", "p0y", "p0z", "v0x", "v0y", "v0z", "omega0x", "omega0y", "omega0z",
                           "omega0_norm",
                           "p1x", "p1y", "p1z", "v1x", "v1y", "v1z", "omega1x", "omega1y", "omega1z",
                           "omega1_norm",
                           "fx_mse_end_learning", "fx_rmse_end_learning", "fx_terms_used", "fx_terms_missing",
                           "fx_sign_correct_count", "fx_sign_terms_checked", "fx_sign_terms_missing"])

for flight in files:
    file, is_success = flight
    with suppress_output():
        log = IndiflightLog(path + file, logId=1, resetTime=True)

    print()
    print(f"- Filename {file}")
    print(f"    - Firmware Revision `{log.parameters['Firmware revision']}`")
    print("```")
    aplt = IndiflightIndividualSysIdPlotter(log.data, Nr=2, Ns=2, true=None, name=f"{file} -- Onboard Individual Analysis")
    print("```")

    plt.close('all')

    idx = aplt.idx_end_learning
    fx_sq_errors = []
    fx_missing = []
    for key, true_val in fx_true.items():
        if key in log.data.columns:
            pred_val = float(log.data[key].iloc[idx])
            fx_sq_errors.append((pred_val - true_val) ** 2)
        else:
            fx_missing.append(key)

    fx_mse = float(np.mean(fx_sq_errors)) if fx_sq_errors else float("nan")
    fx_rmse = float(np.sqrt(fx_mse)) if fx_sq_errors else float("nan")

    sign_correct_count = 0
    sign_terms_checked = 0
    sign_terms_missing = 0
    for key in sign_eval_keys:
        if key in log.data.columns and key in act_true:
            pred_val = float(log.data[key].iloc[idx])
            true_val = float(act_true[key])
            sign_correct_count += int(np.sign(pred_val) == np.sign(true_val))
            sign_terms_checked += 1
        else:
            sign_terms_missing += 1

    print(f"    - fx MSE@idx_end_learning: {fx_mse:.6e} (terms used: {len(fx_sq_errors)}, missing: {len(fx_missing)})")
    print(f"    - fx sign matches: {sign_correct_count}/{sign_terms_checked} (missing: {sign_terms_missing})")

    raw_param_predictions = {}
    for key in act_true:
        col_name = f"pred_end_{key}"
        raw_param_predictions[col_name] = float(log.data[key].iloc[idx]) if key in log.data.columns else float("nan")

    row = {
        "Filename": file,
        "Firmware Revision": log.parameters['Firmware revision'],
        "Success": is_success,
        "p0x": log.data["pos[0]"].iloc[aplt.idx_start_learning],
        "p0y": log.data["pos[1]"].iloc[aplt.idx_start_learning],
        "p0z": log.data["pos[2]"].iloc[aplt.idx_start_learning],
        "v0x": log.data["vel[0]"].iloc[aplt.idx_start_learning],
        "v0y": log.data["vel[1]"].iloc[aplt.idx_start_learning],
        "v0z": log.data["vel[2]"].iloc[aplt.idx_start_learning],
        "omega0x": log.data["gyroADCafterRpm[0]"].iloc[aplt.idx_start_learning],
        "omega0y": log.data["gyroADCafterRpm[1]"].iloc[aplt.idx_start_learning],
        "omega0z": log.data["gyroADCafterRpm[2]"].iloc[aplt.idx_start_learning],
        "omega0_norm": float(np.linalg.norm([
            log.data["gyroADCafterRpm[0]"].iloc[aplt.idx_start_learning],
            log.data["gyroADCafterRpm[1]"].iloc[aplt.idx_start_learning],
            log.data["gyroADCafterRpm[2]"].iloc[aplt.idx_start_learning],
        ])),
        "p1x": log.data["pos[0]"].iloc[aplt.idx_end_learning],
        "p1y": log.data["pos[1]"].iloc[aplt.idx_end_learning],
        "p1z": log.data["pos[2]"].iloc[aplt.idx_end_learning],
        "v1x": log.data["vel[0]"].iloc[aplt.idx_end_learning],
        "v1y": log.data["vel[1]"].iloc[aplt.idx_end_learning],
        "v1z": log.data["vel[2]"].iloc[aplt.idx_end_learning],
        "omega1x": log.data["gyroADCafterRpm[0]"].iloc[aplt.idx_end_learning],
        "omega1y": log.data["gyroADCafterRpm[1]"].iloc[aplt.idx_end_learning],
        "omega1z": log.data["gyroADCafterRpm[2]"].iloc[aplt.idx_end_learning],
        "omega1_norm": float(np.linalg.norm([
            log.data["gyroADCafterRpm[0]"].iloc[aplt.idx_end_learning],
            log.data["gyroADCafterRpm[1]"].iloc[aplt.idx_end_learning],
            log.data["gyroADCafterRpm[2]"].iloc[aplt.idx_end_learning],
        ])),
        "fx_mse_end_learning": fx_mse,
        "fx_rmse_end_learning": fx_rmse,
        "fx_terms_used": len(fx_sq_errors),
        "fx_terms_missing": len(fx_missing),
        "fx_sign_correct_count": sign_correct_count,
        "fx_sign_terms_checked": sign_terms_checked,
        "fx_sign_terms_missing": sign_terms_missing,

    }
    row.update(raw_param_predictions)

    # do not use append because it is deprecated, but it is easier to read than the alternative
    df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)

# make columns floats, not objects
for col in df.columns:
    if col not in ["Filename", "Firmware Revision", "Success"]:
        df[col] = df[col].astype(float)

# output mean and std of each column (excluding "Filename" and "Firmware Revision") grouped by success
print()
print("Summary Statistics:")
summary = df.groupby("Success").agg({col: ["mean", "std"] for col in df.columns if col not in ["Filename", "Firmware Revision"]})
print(summary)

overall_fx_mse = df["fx_mse_end_learning"].mean()
overall_fx_rmse = np.sqrt(overall_fx_mse)
print()
print(f"Overall fx MSE@idx_end_learning: {overall_fx_mse:.6e}")
print(f"Overall fx RMSE@idx_end_learning: {overall_fx_rmse:.6e}")

# make a total of 6 3d scatter plots to the pos, vel, and omega at the start and end of learning, colored by success
plt.close('all')
fig = plt.figure(figsize=(18, 12))
ax1 = fig.add_subplot(231, projection='3d')
ax2 = fig.add_subplot(232, projection='3d')
ax3 = fig.add_subplot(233, projection='3d')
ax4 = fig.add_subplot(234, projection='3d')
ax5 = fig.add_subplot(235, projection='3d')
ax6 = fig.add_subplot(236, projection='3d')

ax1.set_title("Position at Start")
ax1.scatter(df[df["Success"] == True]["p0x"], df[df["Success"] == True]["p0y"], df[df["Success"] == True]["p0z"], c='green', label='Success')
ax1.scatter(df[df["Success"] == False]["p0x"], df[df["Success"] == False]["p0y"], df[df["Success"] == False]["p0z"], c='red', label='Failure')

ax2.set_title("Velocity at Start")
ax2.scatter(df[df["Success"] == True]["v0x"], df[df["Success"] == True]["v0y"], df[df["Success"] == True]["v0z"], c='green', label='Success')
ax2.scatter(df[df["Success"] == False]["v0x"], df[df["Success"] == False]["v0y"], df[df["Success"] == False]["v0z"], c='red', label='Failure')

ax3.set_title("Angular Velocity at Start")
ax3.scatter(df[df["Success"] == True]["omega0x"], df[df["Success"] == True]["omega0y"], df[df["Success"] == True]["omega0z"], c='green', label='Success')
ax3.scatter(df[df["Success"] == False]["omega0x"], df[df["Success"] == False]["omega0y"], df[df["Success"] == False]["omega0z"], c='red', label='Failure')

ax4.set_title("Position at End")
ax4.scatter(df[df["Success"] == True]["p1x"], df[df["Success"] == True]["p1y"], df[df["Success"] == True]["p1z"], c='green', label='Success')
ax4.scatter(df[df["Success"] == False]["p1x"], df[df["Success"] == False]["p1y"], df[df["Success"] == False]["p1z"], c='red', label='Failure')

ax5.set_title("Velocity at End")
ax5.scatter(df[df["Success"] == True]["v1x"], df[df["Success"] == True]["v1y"], df[df["Success"] == True]["v1z"], c='green', label='Success')
ax5.scatter(df[df["Success"] == False]["v1x"], df[df["Success"] == False]["v1y"], df[df["Success"] == False]["v1z"], c='red', label='Failure')

ax6.set_title("Angular Velocity at End")
ax6.scatter(df[df["Success"] == True]["omega1x"], df[df["Success"] == True]["omega1y"], df[df["Success"] == True]["omega1z"], c='green', label='Success')
ax6.scatter(df[df["Success"] == False]["omega1x"], df[df["Success"] == False]["omega1y"], df[df["Success"] == False]["omega1z"], c='red', label='Failure')


for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

fig.show()
plt.show()

