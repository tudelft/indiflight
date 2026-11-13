"""Fit a phi-theory aero model to the part of a flight log that an actuation model does not explain.

An actuation model (forces and moments from rotors and elevons, plus rigid-body kinematics,
assuming hover and no airflow) is evaluated on the target log. Its prediction is subtracted
from the measured specific force and body rate derivative, and the residual, assumed to be
purely aerodynamic, is fitted with the phi-theory model of tailsitter_id.aero_fit.

Rewrite of lsPrototypingTailsitterFF.py. The actuation model comes from one of:

  manual      the TailsitterPhi configured in build_manual_craft() below
  global-phi  tailsitterGlobalPhi.py's global fit on a second log, aero part stripped
  rls         one of rlsPrototypingTailsitter.py's moment RLS models on a second log,
              aero part stripped (moments only: the force rows are not fitted)
  onboard     the final onboard learner estimate (LEARNER mode) of a log (moments + forces)
  file        an actuation model saved earlier with --save-actuation

Global options go before the subcommand, source options after it. Examples:

  python tailsitterFFAero.py LOG00335.BFL --auto-range manual
  python tailsitterFFAero.py LOG00335.BFL global-phi LOG00035_autoId.BFL --source-crop 53.5 114
  python tailsitterFFAero.py LOG00335.BFL rls LOG00335.BFL --source-auto-range --model act_phi3_I
  python tailsitterFFAero.py LOG00335.BFL rls LOG00335.BFL --source-auto-range   # choose interactively
  python tailsitterFFAero.py LOG00335.BFL onboard            # learner estimate from the target log
  python tailsitterFFAero.py LOG00335.BFL --save-actuation act.pkl global-phi ...
  python tailsitterFFAero.py LOG00336.BFL file act.pkl
"""

import sys
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

import matplotlib.pyplot as plt
import numpy as np

from tailsitter_id.data import load_log, prepare_flight_data
from tailsitter_id.actuation import (DEFAULT_INERTIA, DEFAULT_MASS, OUTPUT_LABELS, OUTPUT_UNITS,
                                     CraftActuationModel, OnboardLearnerActuationModel,
                                     import_pyndiflight, load_actuation_model)
from tailsitter_id import aero_fit


# %% option 1: manual PyNDIflight.crafts.TailsitterPhi configuration

def build_manual_craft():
    crafts, _ = import_pyndiflight()
    tail = crafts.TailsitterPhi()

    mass = 0.564
    k = 1.413e-6
    tail.setInertia(m=mass, I=np.diag([5.73e-3, 1.35e-3, 5.43e-3]))
    tail.setRotor(0, X=[1.104e-02, -1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=k, cm=-6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # RR
    tail.setRotor(1, X=[1.104e-02, +1.064e-01, -7.000e-02], ax=[1.558e-01, 0.000e+00, -9.878e-01], k=k, cm=+6.938e-04, wmax=3000., tau=0.03, kESC=0.5, I=2.842e-6) # FR

    cd = np.array([-7.032e-07,          0,          0,          0, -1.835e-08, -6.047e-08], dtype=np.float32)
    cdd = np.array([         0,          0,          0,          0, -1.412e-03,          0], dtype=np.float32)
    cddd = np.array([         0,          0,          0,          0, -2.201e-05,          0], dtype=np.float32)
    d0 = np.array([-3.618e-01, -1.540e-01], dtype=np.float32)
    tail.setElevonModel(cd, cdd, cddd, d0)

    # aero part: not used (stripped), only printed next to the fit for comparison
    phi = 3.297e-1
    Phi = np.array([
        [+3.106e-01,          0, +4.148e-02,          0, -1.613e-04,          0],
        [         0, +3.603e-02,          0, +1.355e-03,          0, -3.538e-03],
        [+4.148e-02,          0, +4.465e-02,          0, +1.951e-04,          0],
        [         0, +1.355e-03,          0, +8.290e-04,          0, +7.494e-04],
        [-1.613e-04,          0, +1.951e-04,          0, +4.093e-04,          0],
        [         0, -3.538e-03,          0, +7.494e-04,          0, +2.790e-03],
    ], dtype=np.float32)
    tail.setPhiModel(phi, Phi)
    return tail


# %% arguments

def add_source_args(p, positional=True):
    if positional:
        p.add_argument("source", type=str, help="Log file to identify the actuation model from.")
    p.add_argument("--source-id", type=int, default=1, help="Log ID in the source file.")
    p.add_argument("--source-resetTime", action="store_true", help="Reset source time to start of the log.")
    rng = p.add_mutually_exclusive_group()
    rng.add_argument("--source-crop", nargs=2, metavar=("START", "END"), type=float,
                     help="Crop the source log to this time range (s).")
    rng.add_argument("--source-auto-range", action="store_true",
                     help="Crop the source log to the span in which motors are commanded.")


parser = ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Target log (forward flight) to fit the aero model on.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
rng = parser.add_mutually_exclusive_group()
rng.add_argument("--crop", nargs=2, metavar=("START", "END"), type=float, help="Crop the target log (s).")
rng.add_argument("--auto-range", action="store_true", help="Crop the target log to the span in which motors are commanded.")
parser.add_argument("--name", help="Name for the analysis, used in plots.")

g = parser.add_argument_group("target signals")
g.add_argument("--filter-hz", type=float, default=5., help="Low-pass cutoff for all target signals.")
g.add_argument("--zero-phase", action="store_true", help="Use zero-phase (filtfilt) instead of causal filtering.")
g.add_argument("--vel-source", default="vel", choices=["vel", "localVel", "ekf_vel"], help="Inertial velocity field.")
g.add_argument("--acc-source", default="accADCafterRpm", choices=["accADCafterRpm", "accSmooth"], help="Accelerometer field.")

g = parser.add_argument_group("aero fit")
g.add_argument("--wind", nargs=2, type=float, default=[0., 0.], metavar=("WX", "WY"), help="Horizontal wind (m/s, inertial frame).")
g.add_argument("--fit-wind", action="store_true", help="Optimize the horizontal wind (starting at --wind).")
g.add_argument("--phi", type=float, default=0., help="phi in eta = sqrt(|v|^2 + phi |Omega|^2).")
g.add_argument("--fit-phi", action="store_true", help="Optimize phi (starting at --phi).")
g.add_argument("--d0", nargs=2, type=float, metavar=("D0", "D1"),
               help="Elevon zero angles (rad) for the elevon terms. Default: from the actuation model, else 0.")
g.add_argument("--mass", type=float, default=DEFAULT_MASS, help="Mass (kg) if the actuation model has none (rls, onboard).")
g.add_argument("--inertia", nargs=3, type=float, default=list(np.diag(DEFAULT_INERTIA)), metavar=("IXX", "IYY", "IZZ"),
               help="Principal inertia (kg m^2) if the actuation model has none (rls, onboard).")

g = parser.add_argument_group("output")
g.add_argument("--save-actuation", metavar="PATH", help="Save the actuation model (pickle) for later use with 'file'.")
g.add_argument("--no-flight-plots", action="store_true", help="Skip the IndiflightPlotter flight data figures.")
g.add_argument("--no-show", action="store_true", help="Do not open plot windows.")

sub = parser.add_subparsers(dest="actuation", required=True, metavar="ACTUATION",
                            help="Actuation model source: manual, global-phi, rls, onboard, file")

p = sub.add_parser("manual", help="TailsitterPhi configured in build_manual_craft().", formatter_class=ArgumentDefaultsHelpFormatter)
p.add_argument("--no-rate-terms", action="store_true",
               help="Zero rotor wdot and elevon rate/accel terms (as lsPrototypingTailsitterFF.py effectively did).")

p = sub.add_parser("global-phi", help="Fit tailsitterGlobalPhi.py's model on a source log.", formatter_class=ArgumentDefaultsHelpFormatter)
add_source_args(p)
p.add_argument("--epochs", type=int, default=100, help="LBFGS epochs.")
p.add_argument("--no-acc-smooth", action="store_true", help="Use accADCafterRpm instead of accSmooth in the fit.")

p = sub.add_parser("rls", help="Run rlsPrototypingTailsitter.py's moment RLS models on a source log.", formatter_class=ArgumentDefaultsHelpFormatter)
add_source_args(p)
p.add_argument("--model", help="Model to use (see tailsitter_id.offline_rls.MODELS). Prompted for if omitted.")

p = sub.add_parser("onboard", help="Final onboard learner estimate (LEARNER mode).", formatter_class=ArgumentDefaultsHelpFormatter)
p.add_argument("source", nargs="?", help="Log containing the learner estimate (default: the target log, uncropped).")
add_source_args(p, positional=False)
p.add_argument("--imu-offset", nargs=3, type=float, default=[0., 0., 0.], metavar=("X", "Y", "Z"),
               help="acc_offset (m) the learner corrected its specific force target with (not logged).")

p = sub.add_parser("file", help="Actuation model saved with --save-actuation.", formatter_class=ArgumentDefaultsHelpFormatter)
p.add_argument("path", help="Pickle file.")

args = parser.parse_args()
if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

plt.close("all")


# %% target log

log, data = load_log(args.logfile, log_id=args.id, reset_time=args.resetTime, crop=args.crop, auto_range=args.auto_range)
fd = prepare_flight_data(data, name=args.name, filter_hz=args.filter_hz, zero_phase=args.zero_phase,
                         vel_source=args.vel_source, acc_source=args.acc_source)
print(f"target: {args.logfile}, {fd.N} samples, {fd.t[0]:.2f} s to {fd.t[-1]:.2f} s")


# %% actuation model

def load_source():
    return load_log(args.source, log_id=args.source_id, reset_time=args.source_resetTime,
                    crop=args.source_crop, auto_range=args.source_auto_range)


def choose_rls_model(estimators, prep):
    from tailsitter_id import offline_rls
    names = list(estimators)
    print(f"\n{'#':>2}  {'model':14s} {'RMSE full':>10s} {'RMSE act':>10s}  title")
    for k, n in enumerate(names):
        th = estimators[n].theta_h[-1]
        rm_full = offline_rls.rmse(prep, n, th)
        rm_act = offline_rls.rmse(prep, n, th, strip_aero=True)
        print(f"{k:2d}  {n:14s} {rm_full:10.2f} {rm_act:10.2f}  {offline_rls.MODELS[n][2]}")
    print("RMSE in rad/s^2 on the source data; 'act' is with the aero columns stripped.")
    if not sys.stdin.isatty():
        sys.exit("No --model given and stdin is not interactive.")
    while True:
        sel = input("Select model (number or name): ").strip()
        if sel in names:
            return sel
        if sel.isdigit() and int(sel) < len(names):
            return names[int(sel)]
        print("invalid selection")


if args.actuation == "manual":
    act = CraftActuationModel(build_manual_craft(), name="manual config", include_rate_terms=not args.no_rate_terms)

elif args.actuation == "global-phi":
    from tailsitter_id import global_phi
    _, src = load_source()
    act, torch_model, _ = global_phi.fit_global_phi(src, name=f"global phi fit on {args.source.split('/')[-1]}",
                                                    epochs=args.epochs, use_acc_smooth=not args.no_acc_smooth)
    print("\nFull model of the global fit (actuation + aero):")
    global_phi.print_parameters(act.parameters)

elif args.actuation == "rls":
    from tailsitter_id import offline_rls
    _, src = load_source()
    prep = offline_rls.prepare_data(src)
    if args.model is not None and args.model not in offline_rls.MODELS:
        sys.exit(f"Unknown model '{args.model}', choose from {list(offline_rls.MODELS)}")
    estimators = offline_rls.run(prep, None if args.model is None else [args.model])
    model_name = args.model or choose_rls_model(estimators, prep)
    act = offline_rls.OfflineRLSActuationModel(model_name, estimators[model_name].theta_h[-1],
                                               name=f"offline RLS '{model_name}' on {args.source.split('/')[-1]}")

elif args.actuation == "onboard":
    if args.source is None:
        src_log, src_data = log, log.data  # target log, uncropped
        src_name = args.logfile
        if args.source_crop or args.source_auto_range:
            sys.exit("--source-crop/--source-auto-range need an explicit source log")
    else:
        src_log, src_data = load_source()
        src_name = args.source
    LEARNER_BOX = 11  # boxId_e BOXLEARNER, see IndiflightLog.modeToText
    if LEARNER_BOX not in {b for bits in src_log.flags['enable'] for b in bits}:
        print("WARNING: LEARNER mode was never enabled in the source log")
    act = OnboardLearnerActuationModel.from_log_data(src_data, row=-1, r_imu=args.imu_offset)
    act.name = f"{act.name} of {src_name.split('/')[-1]}"

elif args.actuation == "file":
    act = load_actuation_model(args.path)

print(f"\nActuation model: {act.describe()}")
if args.save_actuation:
    act.save(args.save_actuation)


# %% residual and aero fit

m = act.m if act.m is not None else args.mass
I = act.I if act.I is not None else np.diag(args.inertia)
if args.d0 is not None:
    d0 = np.array(args.d0)
elif act.d0 is not None:
    d0 = act.d0
else:
    d0 = np.zeros(2)
    print("actuation model has no elevon zero angle, using d0 = 0 (set with --d0)")
print(f"conversion to forces/moments: m = {m:.4g} kg, I = diag({', '.join(f'{v:.3e}' for v in np.diag(I))})")

y_act = act.predict(fd)
y_obs = fd.y_obs()
res = aero_fit.fit_aero_outer(fd, y_act, m, I, d0, wind0=args.wind, phi0=args.phi,
                              fit_wind=args.fit_wind, fit_phi=args.fit_phi)
aero_fit.print_result(res)

stripped = getattr(act, "stripped_phi", None)
if stripped is not None:
    print(f"\n# for comparison: aero part of the actuation model source (stripped), phi = {stripped[0]:.3e}")
    print(np.array2string(np.asarray(stripped[1]), formatter={"float_kind": aero_fit.format_scientific}, max_line_width=200))

# aero fit back in measurement units
nu = y_obs - y_act
nu_hat = np.hstack((res.FM_hat[:, :3] / m, res.FM_hat[:, 3:] @ np.linalg.inv(I).T))


# %% plots

title = f"{args.name} -- {act.name}"


def plot_signals(y_obs, y_act, nu, nu_hat, suptitle, sharex=False):
    # sharex may be an axes to share with: BlittedCursor re-shares everything with the first axes
    fig, axes = plt.subplots(2, 3, sharex=sharex, figsize=(16, 8))
    for idx, ax in enumerate(axes.ravel()):
        ax.plot(fd.t, y_obs[:, idx], color="k", lw=1.0, label="measured")
        if res.rows[idx]:
            ax.plot(fd.t, y_act[:, idx], lw=1.0, label="actuation model")
            ax.plot(fd.t, nu[:, idx], lw=1.0, alpha=0.8, label="residual")
            ax.plot(fd.t, nu_hat[:, idx], "--", lw=1.0, label=f"aero fit, $R^2$ = {res.r2[idx]:.2f}")
        else:
            ax.text(0.5, 0.5, "not modelled", transform=ax.transAxes, ha="center", va="center", color="gray")
        ax.set_title(OUTPUT_LABELS[idx])
        ax.set_ylabel(OUTPUT_UNITS[idx])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.suptitle(suptitle)
    fig.tight_layout()
    return fig, axes


fig, axes = plot_signals(y_obs, y_act, nu, nu_hat,
                         f"{title}\nmeasured / actuation model / residual / aero fit")

# same plot, every signal low-pass filtered (2nd order Butterworth, zero phase so that
# the signals stay time-aligned with the unfiltered plot and the flight plots)
from scipy.signal import butter, sosfiltfilt
LP_CUTOFF_HZ = 5.0
fs = 1.0 / np.median(np.diff(fd.t))
sos_lp = butter(2, LP_CUTOFF_HZ, btype="low", fs=fs, output="sos")
lp = lambda x: sosfiltfilt(sos_lp, x, axis=0)
fig_lp, axes_lp = plot_signals(lp(y_obs), lp(y_act), lp(nu), lp(nu_hat),
                               f"{title}\nmeasured / actuation model / residual / aero fit, "
                               f"all signals low-pass filtered ({LP_CUTOFF_HZ:g} Hz, 2nd order)")
                               #sharex=axes[0, 0])

figp, axp = plt.subplots(1, len(aero_fit.PARAM_GROUPS), figsize=(16, 4))
for ax, (group, names) in zip(axp, aero_fit.PARAM_GROUPS.items()):
    idx = [aero_fit.PARAM_NAMES.index(n) for n in names]
    th = res.theta[idx]
    sd = res.theta_std[idx]
    ok = np.isfinite(th)
    ax.errorbar(np.arange(len(names))[ok], th[ok], yerr=2 * sd[ok], fmt="o", capsize=4)
    ax.set_xticks(range(len(names)), names)
    ax.axhline(0., color="gray", lw=0.8)
    ax.set_title(group)
    ax.grid(True, alpha=0.3)
figp.suptitle(f"{title}\naero parameters, nominal $\\pm 2\\sigma$")
figp.tight_layout()

cursor_axes = list(axes.ravel()) + list(axes_lp.ravel())
if not args.no_flight_plots:
    from indiflightPlotter import IndiflightPlotter, IndiflightViewport
    from pyFlightPlotter.crafts import Tailsitter
    fplt = IndiflightPlotter(data, Ns=2, Nr=2, name=f"{args.name} -- Flight Data")
    pplt = IndiflightViewport(Tailsitter(), data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Flight Data")
    fplt.connect_viewport(pplt)
    cursor_axes += fplt.all_axes

from pyFlightPlotter import BlittedCursor
cursor = BlittedCursor(cursor_axes, sharex=True)

if not args.no_show:
    plt.show()
