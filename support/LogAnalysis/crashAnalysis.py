
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
from flight_metrics import ACT_TRUE, suppress_output, compute_fx_fit_metrics, extract_learning_metrics, infer_learning_indices

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
# files = [
#     ("2026-02-16/bat1vid1/LOG00296_badThrowCrash.BFL", False),
#     ("2026-02-16/bat1vid1/LOG00297.BFL", True),
#     ("2026-02-16/bat1vid1/LOG00298.BFL", True),
#     ("2026-02-16/bat1vid1/LOG00299.BFL", True),
#     ("2026-02-16/bat1vid1/LOG00300.BFL", True),
#     ("2026-02-16/bat1vid1/LOG00301_crash.BFL", False),
#     ("2026-02-16/bat2vid2/LOG00302.BFL", True),
#     ("2026-02-16/bat2vid2/LOG00303.BFL", True),
#     ("2026-02-16/bat2vid2/LOG00304.BFL", True),
#     ("2026-02-16/bat2vid2/LOG00305_crashFlatSpin.BFL", False),
#     ("2026-02-16/bat2vid3/LOG00306.BFL", True),
#     ("2026-02-16/bat2vid3/LOG00307_crashMaybeLowThrow.BFL", False),
#     ("2026-02-16/bat3vid4/LOG00308.BFL", True),
#     ("2026-02-16/bat3vid4/LOG00309_crash.BFL", False),
#     ("2026-02-16/bat3vid4/LOG00310.BFL", True),
#     ("2026-02-16/bat3vid4/LOG00311.BFL", True),
#     ("2026-02-16/bat3vid4/LOG00312.BFL", True),
#     ("2026-02-16/bat3vid4/LOG00313.BFL", True),
#     ("2026-02-16/bat3vid4/LOG00314.BFL", True),
# ]

files = [
    ("2026-04-29/LOG00315.BFL", True),
    ("2026-04-29/LOG00316.BFL", True),
    ("2026-04-29/LOG00317.BFL", True),
    ("2026-04-29/LOG00318.BFL", True),
    ("2026-04-29/LOG00319_crashSide.BFL", False),
    ("2026-04-29/LOG00320.BFL", True),
    ("2026-04-29/LOG00321.BFL", True),
    ("2026-04-29/LOG00322.BFL", True),
    ("2026-04-29/LOG00323.BFL", True),
    ("2026-04-29/LOG00324.BFL", True),
    ("2026-04-29/LOG00325.BFL", True),
    ("2026-04-29/LOG00326_crash.BFL", False),
    ("2026-04-29/LOG00328.BFL", True),
    ("2026-04-29/LOG00329.BFL", True),
    ("2026-04-29/LOG00330_crash.BFL", False),
    ("2026-04-29/LOG00331.BFL", True),
    ("2026-04-29/LOG00332.BFL", True),
    ("2026-04-29/LOG00333.BFL", True),
    ("2026-04-29/LOG00334.BFL", True),
    ("2026-04-29/LOG00335.BFL", True),
]

import pandas as pd
df = pd.DataFrame(columns=["Filename", "Firmware Revision", "Success",
                           "p0x", "p0y", "p0z", "v0x", "v0y", "v0z", "omega0x", "omega0y", "omega0z",
                           "omega0_norm", "tilt0", "roll0", "pitch0",
                           "p1x", "p1y", "p1z", "v1x", "v1y", "v1z", "omega1x", "omega1y", "omega1z",
                           "omega1_norm", "tilt1", "roll1", "pitch1",
                           "fx_mse_end_learning", "fx_rmse_end_learning", "fx_terms_used", "fx_terms_missing",
                           "fx_sign_correct_count", "fx_sign_terms_checked", "fx_sign_terms_missing"])

#%% iterate over data files and compute metrics

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

    idx_start, idx_end = infer_learning_indices(log.data, nr=2, ns=2, learning_duration_s=0.5)
    idx = idx_end
    fit_metrics = compute_fx_fit_metrics(log.data, idx)

    print(
        f"    - fx MSE@idx_end_learning: {fit_metrics['fx_mse_end_learning']:.6e} "
        f"(terms used: {fit_metrics['fx_terms_used']}, missing: {fit_metrics['fx_terms_missing']})"
    )
    print(
        f"    - fx sign matches: {fit_metrics['fx_sign_correct_count']}/{fit_metrics['fx_sign_terms_checked']} "
        f"(missing: {fit_metrics['fx_sign_terms_missing']})"
    )

    raw_param_predictions = {}
    for key in ACT_TRUE:
        col_name = f"pred_end_{key}"
        raw_param_predictions[col_name] = float(log.data[key].iloc[idx]) if key in log.data.columns else float("nan")

    row = {
        "Filename": file,
        "Firmware Revision": log.parameters['Firmware revision'],
        "Success": is_success,
    }
    row.update(extract_learning_metrics(log.data, idx_start, idx_end))
    row.update(raw_param_predictions)

    # do not use append because it is deprecated, but it is easier to read than the alternative
    df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)

# make columns floats, not objects
for col in df.columns:
    if col not in ["Filename", "Firmware Revision", "Success"]:
        df[col] = df[col].astype(float)

# todo: test statistic to see which features are most correlated with success


# change to degrees
angles = ['tilt0', 'tilt1', 'roll0', 'roll1', 'pitch0', 'pitch1', 'omega0x', 'omega0y', 'omega0z', 'omega1x', 'omega1y', 'omega1z', 'omega0_norm', 'omega1_norm']
df[angles] = df[angles].apply(lambda x: np.degrees(x))

# output mean and std of each column (excluding "Filename" and "Firmware Revision") grouped by success
print()
print("Summary Statistics:")
summary = df.groupby(lambda _: "All").agg({col: ["mean", "std"] for col in df.columns if col not in ["Filename", "Firmware Revision"]})
summary['N'] = len(df)
bysuccess = df.groupby("Success").agg({col: ["mean", "std"] for col in df.columns if col not in ["Filename", "Firmware Revision"]})
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

overall_fx_mse = df["fx_mse_end_learning"].mean()
overall_fx_rmse = np.sqrt(overall_fx_mse)
print()
print(f"Overall fx MSE@idx_end_learning: {overall_fx_mse:.6e}")
print(f"Overall fx RMSE@idx_end_learning: {overall_fx_rmse:.6e}")



#%% plotting 


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

