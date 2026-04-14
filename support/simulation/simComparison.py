import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

from tqdm import tqdm

SIM_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "LogAnalysis"))
if SIM_ROOT not in sys.path:
    sys.path.append(SIM_ROOT)

from indiflight_log_tools import IndiflightLog
from indiflightPlotter import IndiflightPlotter, IndiflightViewport
from pyFlightPlotter import BlittedCursor, local_rc
from pyFlightPlotter import Quadrotor, Tailsitter



from PyNDIflight.crafts import MultiRotor, TailsitterPhi


def build_multirotor_model():
    mc = MultiRotor()
    # Same baseline model as in support/simulation/exampleQuadSim.py.
    mc.setInertia(m=0.41, I=np.diag([0.75e-3, 0.8e-3, 0.9e-3]))
    mc.setRotor(0, X=[-0.05, +0.0635, 0.0], k=1.88e-7, cm=-0.01, wmax=4900.0, tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(1, X=[+0.05, +0.0635, 0.0], k=1.88e-7, cm=+0.01, wmax=4900.0, tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(2, X=[-0.05, -0.0635, 0.0], k=1.88e-7, cm=+0.01, wmax=4900.0, tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(3, X=[+0.05, -0.0635, 0.0], k=1.88e-7, cm=-0.01, wmax=4900.0, tau=0.02, kESC=0.5, I=5e-7)
    return mc


def build_tailsitter_model():
    tail = TailsitterPhi()
    tail.setInertia(m=0.564, I=np.diag([5.73e-3, 1.35e-3, 5.43e-3]))

    phi = 3.297e-1
    Phi = np.array([
        [+3.106e-01, 0, +4.148e-02, 0, -1.613e-04, 0],
        [0, +3.603e-02, 0, +1.355e-03, 0, -3.538e-03],
        [+4.148e-02, 0, +4.465e-02, 0, +1.951e-04, 0],
        [0, +1.355e-03, 0, +8.290e-04, 0, +7.494e-04],
        [-1.613e-04, 0, +1.951e-04, 0, +4.093e-04, 0],
        [0, -3.538e-03, 0, +7.494e-04, 0, +2.790e-03],
    ], dtype=np.float32)

    cd = np.array([-7.032e-07, 0, 0, 0, -1.835e-08, -6.047e-08], dtype=np.float32)
    cdd = np.array([0, 0, 0, 0, -1.412e-03, 0], dtype=np.float32)
    cddd = np.array([0, 0, 0, 0, -2.201e-05, 0], dtype=np.float32)
    d0 = np.array([-3.618e-01, -1.540e-01], dtype=np.float32)

    tail.setPhiModel(phi, Phi)
    tail.setElevonModel(cd, cdd, cddd, d0)

    tail.setRotor(0, X=[1.104e-02, -1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e00, -9.878e-01], k=1.413e-06, cm=-6.938e-04, wmax=2700.0, tau=0.035, kESC=0.4, I=2.842e-6)
    tail.setRotor(1, X=[1.104e-02, +1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e00, -9.878e-01], k=1.413e-06, cm=+6.938e-04, wmax=2700.0, tau=0.035, kESC=0.4, I=2.842e-6)
    return tail


def _existing_columns(data, names):
    return [name for name in names if name in data.columns]


def _fill_if_exists(df, name, values):
    if name in df.columns:
        df[name] = values


def replay_simulation(data, craft_type="multirotor", t0=0.0, input_source="u"):
    if craft_type == "tailsitter":
        craft = build_tailsitter_model()
        Nr, Ns = 2, 2
    else:
        craft = build_multirotor_model()
        Nr, Ns = 4, 0

    t = data["timeS"].to_numpy()
    i0 = int(np.searchsorted(t, t0, side="left"))
    i0 = int(np.clip(i0, 0, len(t) - 1))
    t = t[i0:]
    ref = data.iloc[i0:].copy().reset_index(drop=True)

    if len(t) < 2:
        raise ValueError("Not enough samples after selected initial time.")

    pos0 = ref[[f"pos[{i}]" for i in range(3)]].iloc[0].to_numpy(dtype=np.float32)
    quat0 = ref[[f"quat[{i}]" for i in range(4)]].iloc[0].to_numpy(dtype=np.float32)
    vel0 = ref[[f"vel[{i}]" for i in range(3)]].iloc[0].to_numpy(dtype=np.float32)
    omega0 = ref[[f"gyroADCafterRpm[{i}]" for i in range(3)]].iloc[0].to_numpy(dtype=np.float32)

    craft.setPose(x=pos0, q=quat0)
    craft.setTwist(v=vel0, w=omega0)

    motor_feedback_cols = [f"omegaUnfiltered[{i}]" for i in range(Nr)]
    if all(c in ref.columns for c in motor_feedback_cols):
        craft.r_w[:] = ref[motor_feedback_cols].iloc[0].to_numpy(dtype=np.float32)

    if Ns > 0 and all(f"servo_feedback[{i}]" in ref.columns for i in range(Ns)):
        craft.s_d[:] = ref[[f"servo_feedback[{i}]" for i in range(Ns)]].iloc[0].to_numpy(dtype=np.float32)

    cmd_cols = [f"{input_source}[{i}]" for i in range(Nr + Ns)]
    if not all(c in ref.columns for c in cmd_cols):
        raise KeyError(f"Input columns not found for --input-source={input_source}: {cmd_cols}")

    pos = np.zeros((len(t), 3), dtype=np.float64)
    vel = np.zeros((len(t), 3), dtype=np.float64)
    quat = np.zeros((len(t), 4), dtype=np.float64)
    gyro = np.zeros((len(t), 3), dtype=np.float64)
    alpha = np.zeros((len(t), 3), dtype=np.float64)
    spf = np.zeros((len(t), 3), dtype=np.float64)
    omega = np.zeros((len(t), Nr), dtype=np.float64)
    servo = np.zeros((len(t), Ns), dtype=np.float64) if Ns > 0 else None

    def sample(k):
        pos[k, :] = craft.xI
        vel[k, :] = craft.vI
        quat[k, :] = craft.q
        gyro[k, :] = craft.OB
        alpha[k, :] = craft.ODotB
        spf[k, :] = craft.fspB
        omega[k, :] = craft.r_w
        if Ns > 0:
            servo[k, :] = craft.s_d

    sample(0)
    dt_default = float(np.median(np.diff(t)))
    dt_default = max(dt_default, 1e-4)

    for k in tqdm(range(len(t) - 1)):
        uk = ref[cmd_cols].iloc[k].to_numpy(dtype=np.float32)
        # craft.r_u[:] = uk[:Nr]
        craft.r_u[:] = ref[[f"motor[{i}]" for i in range(Nr)]].iloc[k].to_numpy(dtype=np.float32)
        if Ns > 0:
            craft.s_u[:] = uk[Nr:Nr + Ns]

        dt = float(t[k + 1] - t[k])
        if not np.isfinite(dt) or dt <= 0.0:
            dt = dt_default
        dt = float(np.clip(dt, 1e-4, 0.02))
        craft.tick(dt)
        sample(k + 1)

    sim = ref.copy()
    sim["timeS"] = t

    for i in range(3):
        _fill_if_exists(sim, f"pos[{i}]", pos[:, i])
        _fill_if_exists(sim, f"vel[{i}]", vel[:, i])
        _fill_if_exists(sim, f"quat[{i}]", quat[:, i])
        _fill_if_exists(sim, f"gyroADCafterRpm[{i}]", gyro[:, i])
        _fill_if_exists(sim, f"gyroADC[{i}]", gyro[:, i])
        _fill_if_exists(sim, f"alpha[{i}]", alpha[:, i])
        _fill_if_exists(sim, f"accADCafterRpm[{i}]", spf[:, i])
        _fill_if_exists(sim, f"accSmooth[{i}]", spf[:, i])

    _fill_if_exists(sim, "quat[3]", quat[:, 3])

    for i in range(Nr):
        _fill_if_exists(sim, f"omegaUnfiltered[{i}]", omega[:, i])
        _fill_if_exists(sim, f"omega[{i}]", omega[:, i])
        _fill_if_exists(sim, f"u_state[{i}]", ref[f"{input_source}[{i}]"].to_numpy())

    if Ns > 0 and servo is not None:
        for i in range(Ns):
            _fill_if_exists(sim, f"servo_feedback[{i}]", servo[:, i])
            _fill_if_exists(sim, f"u_state[{Nr + i}]", ref[f"{input_source}[{Nr + i}]"].to_numpy())

    return ref, sim, Nr, Ns


def plot_overlay(ref, sim, Nr, title):
    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    t = ref["timeS"].to_numpy()

    COLORS = ["C0", "C1", "C2", "C3", "C4", "C5"]
    LINESTYLES = ["-", "--", "-.", ":"]
    ax_angacc = axs[0, 0]
    ax_spf = axs[1, 0]
    ax_motor = axs[0, 1]
    ax_servo = axs[1, 1]

    # Left column: angular acceleration, specific force
    dt_mean = np.mean(np.diff(t))
    for i, lbl in enumerate(["roll", "pitch", "yaw"]):
        gyro_dot_ref = np.gradient(ref[f"gyroADCafterRpm[{i}]"].to_numpy(), dt_mean)
        gyro_dot_sim = np.gradient(sim[f"gyroADCafterRpm[{i}]"].to_numpy(), dt_mean)
        ax_angacc.plot(t, gyro_dot_ref, alpha=0.35, lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[0], label=lbl)
        ax_angacc.plot(t, gyro_dot_sim, lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[1], label=lbl)
    ax_angacc.set_ylabel("angular accel [rad/s²]")
    ax_angacc.set_title("Angular acceleration: log (solid) vs simulation (dashed)")
    ax_angacc.legend(loc="upper right")
    ax_angacc.grid(True)

    for i, lbl in enumerate(["x", "y", "z"]):
        ax_spf.plot(t, ref[f"accADCafterRpm[{i}]"], alpha=0.35, lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[0], label=lbl)
        ax_spf.plot(t, sim[f"accADCafterRpm[{i}]"], lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[1], label=lbl)
    ax_spf.set_ylabel("specific force [N/kg]")
    ax_spf.set_title("Specific force: log (solid) vs simulation (dashed)")
    ax_spf.legend(loc="upper right")
    ax_spf.grid(True)

    # Right column: motor speed, servo feedback
    for i in range(Nr):
        ax_motor.plot(t, ref[f"omegaUnfiltered[{i}]"], alpha=0.35, lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[0], label=f"motor {i + 1}")
        ax_motor.plot(t, sim[f"omegaUnfiltered[{i}]"], lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[1], label=f"motor {i + 1}")
    ax_motor.set_ylabel("motor speed [rad/s]")
    ax_motor.set_title("Motor speed: log (solid) vs simulation (dashed)")
    ax_motor.legend(loc="upper right")
    ax_motor.grid(True)

    servo_cols = [c for c in ref.columns if c.startswith("servo_feedback[")]
    Ns = len(servo_cols)
    if Ns > 0 and all(f"servo_feedback[{i}]" in sim.columns for i in range(Ns)):
        for i in range(Ns):
            ax_servo.plot(t, ref[f"servo_feedback[{i}]"], alpha=0.35, lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[0], label=f"servo {i + 1}")
            ax_servo.plot(t, sim[f"servo_feedback[{i}]"], lw=1.0, color=COLORS[i % len(COLORS)], linestyle=LINESTYLES[1], label=f"servo {i + 1}")
        ax_servo.set_ylabel("servo [rad]")
        ax_servo.set_title("Servo feedback: log (solid) vs simulation (dashed)")
        ax_servo.legend(loc="upper right")
    else:
        ax_servo.text(0.5, 0.5, "No servo feedback in log", ha="center", va="center", transform=ax_servo.transAxes)
        ax_servo.set_title("Servo feedback")
        ax_servo.set_ylabel("servo [rad]")
    ax_servo.grid(True)

    ax_spf.set_xlabel("time [s]")
    ax_servo.set_xlabel("time [s]")

    fig.suptitle(title)
    return fig, list(axs.flatten())


if __name__ == "__main__":
    plt.close("all")
    plt.rcParams.update(local_rc)

    parser = ArgumentParser(
        description="Replay a log through PyNDIflight and compare against recorded flight states.",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
    parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
    parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                        help="Crop the log to the given time range (in seconds).")
    parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")
    parser.add_argument("--type", type=str, default="multirotor", choices=["tailsitter", "multirotor"],
                        help="Craft model type to use for replay.")
    parser.add_argument("--init-time", type=float, default=None,
                        help="Time in seconds used as simulation initial condition. Defaults to first sample.")
    parser.add_argument("--input-source", type=str, default="u", choices=["u", "u_state"],
                        help="Which logged actuator signal drives the simulation.")
    args = parser.parse_args()

    if args.name is None:
        args.name = args.logfile.split("/")[-1].split(".")[0]

    log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
    if args.crop:
        log.data, _ = log.crop(args.crop[0], args.crop[1])

    t0 = args.init_time if args.init_time is not None else float(log.data["timeS"].iloc[0])
    ref, sim, Nr, Ns = replay_simulation(log.data, craft_type=args.type, t0=t0, input_source=args.input_source)

    fplt_log = IndiflightPlotter(ref, Nr=Nr, Ns=Ns, name=f"{args.name} -- Log")
    fplt_sim = IndiflightPlotter(sim, Nr=Nr, Ns=Ns, name=f"{args.name} -- Simulation Replay")

    craft_vis = Tailsitter() if args.type == "tailsitter" else Quadrotor()
    pplt_log = IndiflightViewport(craft_vis, ref, Nr=Nr, Ns=Ns, follow=False,
                                title=f"{args.name} -- Log Viewport")
    pplt_sim = IndiflightViewport(craft_vis, sim, Nr=Nr, Ns=Ns, follow=False,
                                title=f"{args.name} -- Sim Viewport")
    fplt_log.connect_viewport(pplt_log)
    fplt_sim.connect_viewport(pplt_sim)

    f_cmp, cmp_axes = plot_overlay(ref, sim, Nr, title=f"{args.name} -- Log vs Simulation")
    cursor = BlittedCursor(fplt_log.all_axes + fplt_sim.all_axes + cmp_axes, sharex=True)
    _ = cursor, f_cmp
    plt.show()
    
    # cursor = BlittedCursor(fplt_log.all_axes + fplt_sim.all_axes, sharex=True)
    # plt.show()

