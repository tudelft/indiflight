"""flightPlotter.py in the browser, through pyFlightPlotter.web.

Same panels and 3D view as flightPlotter.py / IndiflightPlotter, drawn with
uPlot + three.js. Needs pyFlightPlotter with the web module (branch js-version).

    python webPlotter.py LOG00039.BFL --id 1 --crop 200 260 [--type tailsitter]

From a script or notebook, add your own signals and panels before showing:

    from indiflight_log_tools import IndiflightLog
    from webPlotter import indiflight_web_plotter

    log = IndiflightLog("LOG00039.BFL", logId=1)
    wp = indiflight_web_plotter(log, "hover test")
    wp.add_signal("thrust", k * log.data["omega[0]"] ** 2)
    wp.add_panel("Thrust model", solid=["thrust", "accSmooth[2]"],
                 series_labels=["model", "measured"], ylabel="[N/kg]")
    wp.show()
"""
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from pyFlightPlotter import Craft3D, Tailsitter
from pyFlightPlotter.web import WebPlotter

# full precision for bit masks and absolute times; everything else goes as float32
F64_COLUMNS = ("timeS", "timeMs", "timeUs", "time", "extTime", "flightModeFlags", "stateFlags",
               "GPS_coord[0]", "GPS_coord[1]")


class Hexarotor(Craft3D):
    """as in flightPlotter.py"""
    def __init__(self, l=0.4, R=0.1):
        super().__init__(body_geometry=[np.array([[l, -l/4, 0.], [l, l/4, 0.], [l + l/4, 0, 0.], [l, -l/4, 0.]])])
        r = l / 2
        s = 0.8660254037844386
        for xyz in ([r, 0, 0], [0.5*r, s*r, 0], [-0.5*r, s*r, 0], [-r, 0, 0], [-0.5*r, -s*r, 0], [0.5*r, -s*r, 0]):
            self.addRotor(xyz=xyz, axis=[0, 0, -1], R=R)


def _rotations(d):
    quat = d[[f'quat[{i}]' for i in [1, 2, 3, 0]]].to_numpy()
    quat[np.linalg.norm(quat, axis=1) < 1e-6] = [0, 0, 0, 1]  # avoid NaNs
    return R.from_quat(quat)


def _add_derived(wp, d, Ns):
    """the signals IndiflightPlotter computes on the fly"""
    has = lambda c: c in d.columns
    rot = _rotations(d)
    irot = rot.inv()
    for src, dst in (("vel", "velB"), ("localVel", "localVelB"), ("velSp", "velSpB")):
        if has(f"{src}[0]"):
            v = irot.apply(d[[f"{src}[{i}]" for i in range(3)]].to_numpy())
            for i in range(3):
                wp.add_signal(f"{dst}[{i}]", v[:, i])
    if has("accSmooth[0]"):
        accI = rot.apply(d[[f"accSmooth[{i}]" for i in range(3)]].to_numpy()) + np.array([0, 0, 9.81])
        for i in range(3):
            wp.add_signal(f"accI[{i}]", accI[:, i])
    if Ns > 0 and has("servo_feedback[0]"):
        sig = Signal(d["timeS"].to_numpy(), d[[f"servo_feedback[{i}]" for i in range(Ns)]].to_numpy()) \
            .filtfilt(type="lowpass", order=2, cutoff_hz=20.0)
        dot, dotdot = sig.dot(order=1).y, sig.dot(order=2).y
        for i in range(Ns):
            wp.add_signal(f"servoDot[{i}]", np.where(np.isfinite(dot[:, i]), dot[:, i], np.nan))
            wp.add_signal(f"servoDotDot[{i}]", np.where(np.isfinite(dotdot[:, i]), dotdot[:, i], np.nan))


def _add_panels(wp, d, Nr, Ns):
    """layout of IndiflightPlotter._populate"""
    N = Nr + Ns
    has = lambda c: c in d.columns
    xyz, rpy = ["X", "Y", "Z"], ["roll", "pitch", "yaw"]
    v3 = lambda name: [f"{name}[{i}]" for i in range(3)]
    p = wp.add_panel

    p("Body Angular Rates", row=0, col=2, ylabel="Angular Rate [rad/s]",
      light=v3("gyroADCafterRpm"), solid=v3("gyroADC"), dashed=v3("gyroSp"),
      series_labels=rpy, style_labels=["Raw", "Filtered", "Setpoint"])
    p("Body Specific Forces", row=1, col=1, ylabel="Specific Force [N/kg]",
      solid=v3("accSmooth"), dashed=v3("spfSp"),
      series_labels=xyz, style_labels=["Raw", "Filtered", "Setpoint"])
    p("Body Angular Accel.", row=1, col=2, ylabel="Angular Accel. [rad/s²]",
      solid=v3("alpha"), dashed=v3("alphaSp"),
      series_labels=rpy, style_labels=[None, "Filtered", "Setpoint"])
    if Nr > 0:
        p("Motor Speeds", row=3, col=2, ylabel="Motor Speed [rad/s]", ylimits=(-100, None),
          light=[f"omegaUnfiltered[{i}]" for i in range(Nr)], solid=[f"omega[{i}]" for i in range(Nr)],
          series_labels=[f"Motor {i}" for i in range(1, Nr + 1)], style_labels=["Raw", "Onboard filt.", None])
    p("RC Commands", row=3, col=0, ylabel="RC Command", ylimits=(-1.1, 1.1),
      solid=[f"rcCommand[{i}]" for i in range(4)],
      series_labels=["RC Roll", "RC Pitch", "RC Yaw", "RC Throttle"], style_labels=[None, "Raw", None])
    p("Actuator Commands Motors", row=2, col=2, ylabel="Actuator Commands [-]", ylimits=(-0.05, 1.05),
      solid=[f"u_state[{i}]" for i in range(Nr)], dashed=[f"u[{i}]" for i in range(Nr)],
      series_labels=[f"Actuator {i}" for i in range(1, Nr + 1)], style_labels=[None, "Est. state", "Command"])
    if Ns > 0:
        p("Actuator Commands Servos", row=2, col=1, ylabel="Actuator Commands [-]", ylimits=(-1.05, 1.05),
          solid=[f"u_state[{i}]" for i in range(Nr, N)], dashed=[f"u[{i}]" for i in range(Nr, N)],
          series_labels=[f"Actuator {i}" for i in range(Nr + 1, N + 1)], style_labels=[None, "Est. state", "Command"])
        if has("servo_feedback[0]"):
            p("Servo State", row=3, col=1, ylabel="Servo State [rad]",
              solid=[f"servo_feedback[{i}]" for i in range(Ns)],
              dashed=[(f"u[{i}]", 100. * np.pi / 180.) for i in range(Nr, N)],
              series_labels=[f"Servo {i}" for i in range(1, Ns + 1)], style_labels=[None, "Unfiltered state", "Scaled Command"])
            p("Servo Derivatives", row=4, col=1, ylabel="Servo Derivative",
              light=[f"servoDotDot[{i}]" for i in range(Ns)], solid=[(f"servoDot[{i}]", 10.) for i in range(Ns)],
              series_labels=[f"Servo {i}" for i in range(1, Ns + 1)], style_labels=["2nd Derivative", "1st Derivative ×10", None])
    if has("pos[0]"):
        p("Position", row=0, col=0, ylabel="Position [m]",
          light=v3("localPos") if has("localPos[0]") else None, solid=v3("pos"), dashed=v3("posSp"),
          series_labels=xyz, style_labels=["Mocap", "Estimated", "Setpoint"])
        p("Velocity Global", row=1, col=0, ylabel="Velocity [m/s]",
          light=v3("localVel") if has("localVel[0]") else None, solid=v3("vel"), dashed=v3("velSp"),
          series_labels=xyz, style_labels=["Mocap", "Estimated", "Setpoint"])
        p("Velocity Body", row=0, col=1, ylabel="Velocity [m/s]",
          light=v3("localVelB") if has("localVel[0]") else None, solid=v3("velB"), dashed=v3("velSpB"),
          series_labels=xyz, style_labels=["Mocap", "Estimated", "Setpoint"])
        p("Acceleration Global", row=2, col=0, ylabel="Acceleration [m/s²]",
          solid=v3("accI"), dashed=v3("accSp"),
          series_labels=xyz, style_labels=[None, "Estimated", "Setpoint"])


def _set_viewport(wp, d, craft, Nr, Ns):
    """data of IndiflightViewport"""
    has = lambda c: c in d.columns
    cols = lambda name, idx: [f"{name}[{i}]" for i in idx] if has(f"{name}[{idx[0]}]") else None
    omega_max = np.max(d[[f"omega[{i}]" for i in range(Nr)]].to_numpy()) + 1e-6
    wp.set_viewport(
        craft,
        att=cols("quat", range(4)), attSet=cols("quatSp", range(4)), attMeas=cols("localQuat", range(4)),
        pos=cols("pos", range(3)), posSet=cols("posSp", range(3)), posMeas=cols("localPos", range(3)),
        vel=cols("vel", range(3)), velSet=cols("velSp", range(3)), velMeas=cols("localVel", range(3)),
        acc=cols("accI", range(3)) if has("accSmooth[0]") else None, accSet=cols("accSp", range(3)),
        rotor=[(f"omega[{i}]", 1. / omega_max) for i in range(Nr)],   # normalized for visualization
        rotorSet=[f"u[{i}]" for i in range(Nr)],
        surface=cols("servo_feedback", range(Ns)) if Ns else None,
        surfaceSet=[(f"u[{i}]", 100. * np.pi / 180.) for i in range(Nr, Nr + Ns)] if Ns else None,
        interpolation="previous",
    )


def _add_events(wp, log):
    """log events (FC microseconds) and flight mode changes, on the timeS axis"""
    d = log.data
    if len(d) == 0:
        return
    idx0 = d.index[0]
    off_us = float(np.asarray(log.raw.loc[[idx0], "time"])[0]) - float(d["timeUs"].iloc[0])
    for e in log.events:
        wp.add_event((e["time"] - off_us) * 1e-6, e["name"])
    for _, row in log.flags.iterrows():
        parts = [f"+{IndiflightLog.modeToText(b)}" for b in row["enable"]] \
              + [f"−{IndiflightLog.modeToText(b)}" for b in row["disable"]]
        wp.add_event(row["timeUs"] * 1e-6, " ".join(parts), kind="mode")


def indiflight_web_plotter(log, name, craft_type="multirotor"):
    """WebPlotter with the flightPlotter.py layout for an IndiflightLog"""
    if craft_type == "tailsitter":
        craft, Nr, Ns = Tailsitter(), 2, 2
    else:
        craft, Nr, Ns = Hexarotor(), 6, 0
    d = log.data
    wp = WebPlotter(d["timeS"].to_numpy(), name=f"{name} -- Flight Data")
    wp.add_signals(d, float64=F64_COLUMNS)
    _add_derived(wp, d, Ns)
    _add_panels(wp, d, Nr, Ns)
    _set_viewport(wp, d, craft, Nr, Ns)
    _add_events(wp, log)
    return wp


if __name__ == "__main__":
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description="Plot an Indiflight log in the browser.",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
    parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
    parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                        help="Crop the log to the given time range (in seconds).")
    parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")
    parser.add_argument("--type", type=str, default="multirotor", choices=["tailsitter", "multirotor"],
                        help="Type of craft for visualization.")
    parser.add_argument("--port", type=int, default=0, help="HTTP port (0: pick a free one).")
    parser.add_argument("--no-browser", action="store_true", help="Only print the URL.")
    args = parser.parse_args()

    if args.name is None:
        args.name = os.path.basename(args.logfile).split(".")[0]

    log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
    if args.crop:
        log.data, _ = log.crop(args.crop[0], args.crop[1])

    indiflight_web_plotter(log, args.name, args.type).show(open_browser=not args.no_browser, port=args.port)
