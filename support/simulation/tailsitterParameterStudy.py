# PyNDIflight simulation definition for a multirotor.
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.


import numpy as np
from tqdm import tqdm
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import threading
import os
import sys
import importlib
import pickle
from types import SimpleNamespace
import yaml

from PyNDIflight.crafts import TailsitterPhi, IMU
from PyNDIflight.interfaces import IndiflightSITLWrapper, visApp
from PyNDIflight.sim import Sim


def make_parser():
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)

    # Optional SIL integration.
    parser.add_argument(
        "--sil",
        required=True,
        type=str,
        metavar="LIBRARY",
        default=None,
        help="Load INDIflight interface as shared library.",
    )
    parser.add_argument(
        "--sil-profile-txt",
        required=True,
        type=str,
        metavar="PROFILE.txt",
        help="Import these profile settings into the SIL",
    )

    # Runtime settings.
    parser.add_argument("--runs", type=int, default=1, help="Number of Monte Carlo simulations")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for repeatability")
    parser.add_argument("--no-vis", action="store_true", help="Do not launch visualization webserver")

    parser.add_argument(
        "--sigma-config",
        type=str,
        default="support/simulation/tailsitter_parameter_sigmas.yaml",
        help="YAML configuration for parameter randomization sigmas",
    )

    parser.add_argument(
        "--params-pkl",
        type=str,
        default="logs-mockup/tailsitter_parameter_samples.pkl",
        help="Output pickle containing base and sampled tails plus sampled parameter dicts",
    )

    return parser


def rel_scale(rng, sigma, min_scale=0.5, max_scale=1.5):
    scale = 1.0 + rng.normal(0.0, sigma)
    return float(np.clip(scale, min_scale, max_scale))


def load_sigma_config(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    return {
        "mass_rel_sigma": float(cfg.get("mass_rel_sigma", 0.03)),
        "inertia_rel_sigma": float(cfg.get("inertia_rel_sigma", 0.05)),
        "rotor_k_rel_sigma": float(cfg.get("rotor_k_rel_sigma", 0.08)),
        "rotor_cm_rel_sigma": float(cfg.get("rotor_cm_rel_sigma", 0.08)),
        "rotor_tau_rel_sigma": float(cfg.get("rotor_tau_rel_sigma", 0.10)),
        "rotor_pos_rel_sigma": float(cfg.get("rotor_pos_rel_sigma", 0.01)),
        "elevon_cd_rel_sigma": float(cfg.get("elevon_cd_rel_sigma", 0.10)),
        "randomize_rotor_y_sign": bool(cfg.get("randomize_rotor_y_sign", True)),
        "randomize_elevon_cd_sign": bool(cfg.get("randomize_elevon_cd_sign", True)),
        "throw": {
            "height_rel_sigma": float((cfg.get("throw") or {}).get("height_rel_sigma", 0.03)),
            "wb_rel_sigma": float((cfg.get("throw") or {}).get("wb_rel_sigma", 0.08)),
            "vhorz_rel_sigma": float((cfg.get("throw") or {}).get("vhorz_rel_sigma", 0.08)),
        },
    }


def get_base_model():
    base = {}
    base["m"] = 0.564
    base["I_diag"] = np.array([5.73e-3, 1.35e-3, 5.43e-3], dtype=np.float32)
    base["phi"] = 3.297e-1
    base["Phi"] = np.array([
        [+3.106e-01,          0, +4.148e-02,          0, -1.613e-04,          0],
        [         0, +3.603e-02,          0, +1.355e-03,          0, -3.538e-03],
        [+4.148e-02,          0, +4.465e-02,          0, +1.951e-04,          0],
        [         0, +1.355e-03,          0, +8.290e-04,          0, +7.494e-04],
        [-1.613e-04,          0, +1.951e-04,          0, +4.093e-04,          0],
        [         0, -3.538e-03,          0, +7.494e-04,          0, +2.790e-03],
    ], dtype=np.float32)

    base["cd"] = np.array([-7.032e-07,          0,          0,          0, -1.835e-08, -6.047e-08], dtype=np.float32)
    base["cdd"] = np.array([         0,          0,          0,          0, -1.412e-03,          0], dtype=np.float32)
    base["cddd"] = np.array([         0,          0,          0,          0, -2.201e-05,          0], dtype=np.float32)
    # base["d0"] = np.array([-3.618e-01, -1.540e-01], dtype=np.float32)
    base["d0"] = np.array([0, 0], dtype=np.float32)

    base["rotors"] = [
        {
            "X": np.array([1.104e-02, -1.064e-01, -7.000e-02], dtype=np.float32),
            "ax": [1.558e-01, 0.000e+00, -9.878e-01],
            "k": 1.413e-06,
            "cm": -6.938e-04,
            "wmax": 3000.0,
            "tau": 0.03,
            "kESC": 0.5,
            "I": 2.842e-6,
        },
        {
            "X": np.array([1.104e-02, +1.064e-01, -7.000e-02], dtype=np.float32),
            "ax": [1.558e-01, 0.000e+00, -9.878e-01],
            "k": 1.413e-06,
            "cm": +6.938e-04,
            "wmax": 3000.0,
            "tau": 0.03,
            "kESC": 0.5,
            "I": 2.842e-6,
        },
    ]

    base["imu"] = {
        "r": [-3.580e-02, -3.827e-03, +5.630e-03],
        "qBody": [0.707, 0.0, 0.707, 0.0],
        "accStd": 0.08,
        "gyroStd": 0.08,
    }

    return base


def create_tail_from_params(base, params):
    tail = TailsitterPhi()
    tail.setInertia(m=float(params["m"]), I=np.diag(params["I_diag"]))
    tail.setPhiModel(base["phi"], base["Phi"])
    tail.setElevonModel(params["cd"], base["cdd"], base["cddd"], base["d0"])

    for idx, rotor in enumerate(base["rotors"]):
        tail.setRotor(
            idx,
            X=params["rotor_X"][idx],
            ax=rotor["ax"],
            k=rotor["k"] * params["k_scale"],
            cm=rotor["cm"] * params["cm_scale"],
            wmax=rotor["wmax"],
            tau=rotor["tau"] * params["tau_scale"],
            kESC=rotor["kESC"],
            I=rotor["I"],
        )

    return tail


def create_nominal_craft(base):
    nominal_params = {
        "m": float(base["m"]),
        "I_diag": base["I_diag"].copy(),
        "cd": base["cd"].copy(),
        "rotor_X": [rotor["X"].copy() for rotor in base["rotors"]],
        "k_scale": 1.0,
        "cm_scale": 1.0,
        "tau_scale": 1.0,
    }
    return create_tail_from_params(base, nominal_params)


def create_randomized_craft(base, rng, sigma_cfg):
    base_m = base["m"]
    base_I_diag = base["I_diag"]
    base_cd = base["cd"]

    m = base_m * rel_scale(rng, sigma_cfg["mass_rel_sigma"], min_scale=0.67, max_scale=1.5)
    I_diag = base_I_diag * rel_scale(rng, sigma_cfg["inertia_rel_sigma"], min_scale=0.67, max_scale=1.5)

    cd_sigma = np.maximum(np.abs(base_cd), 1e-12) * sigma_cfg["elevon_cd_rel_sigma"]
    cd_sign = float(rng.choice([-1.0, 1.0])) if sigma_cfg["randomize_elevon_cd_sign"] else 1.0
    cd_delta = cd_sign * rng.normal(loc=0.0, scale=cd_sigma, size=base_cd.shape).astype(np.float32)
    cd = base_cd + cd_delta

    k_scale = rel_scale(rng, sigma_cfg["rotor_k_rel_sigma"], min_scale=0.67, max_scale=1.5)
    cm_scale = rel_scale(rng, sigma_cfg["rotor_cm_rel_sigma"], min_scale=0.67, max_scale=1.5)
    tau_scale = rel_scale(rng, sigma_cfg["rotor_tau_rel_sigma"], min_scale=0.67, max_scale=1.5)

    rotor_y_sign = float(rng.choice([-1.0, 1.0])) if sigma_cfg["randomize_rotor_y_sign"] else 1.0

    rotor_X = []
    rotor_X_perturb = []
    for idx, rotor in enumerate(base["rotors"]):
        x_sigma = np.maximum(np.abs(rotor["X"]), 1e-12) * sigma_cfg["rotor_pos_rel_sigma"]
        dX = rng.normal(loc=0.0, scale=x_sigma, size=3).astype(np.float32)
        signed_y = np.abs((rotor["X"] + dX)[1]) * (rotor_y_sign if idx == 1 else -rotor_y_sign)
        x_new = (rotor["X"] + dX).astype(np.float32)
        x_new[1] = signed_y
        rotor_X_perturb.append(dX)
        rotor_X.append(x_new)

    params = {
        "m": float(m),
        "I_diag": I_diag.astype(np.float32),
        "cd": cd,
        "rotor_X": rotor_X,
        "k_scale": k_scale,
        "cm_scale": cm_scale,
        "tau_scale": tau_scale,
    }

    tail = create_tail_from_params(base, params)

    imu = IMU(
        tail,
        r=base["imu"]["r"],
        qBody=base["imu"]["qBody"],
        accStd=base["imu"]["accStd"],
        gyroStd=base["imu"]["gyroStd"],
    )

    sampled = {
        "mass": float(m),
        "Ixx": float(I_diag[0]),
        "Iyy": float(I_diag[1]),
        "Izz": float(I_diag[2]),
        "cd": cd.tolist(),
        "cd_delta": cd_delta.tolist(),
        "cd_sign": cd_sign,
        "k_scale": k_scale,
        "cm_scale": cm_scale,
        "tau_scale": tau_scale,
        "rotor_X": [x.tolist() for x in rotor_X],
        "rotor_X_delta": [dx.tolist() for dx in rotor_X_perturb],
        "rotor_y_sign": rotor_y_sign,
    }
    return tail, imu, sampled


def sampled_to_tail_params(sampled):
    return {
        "m": float(sampled["mass"]),
        "I_diag": np.array([sampled["Ixx"], sampled["Iyy"], sampled["Izz"]], dtype=np.float32),
        "cd": np.array(sampled["cd"], dtype=np.float32),
        "rotor_X": [np.array(x, dtype=np.float32) for x in sampled["rotor_X"]],
        "k_scale": float(sampled["k_scale"]),
        "cm_scale": float(sampled["cm_scale"]),
        "tau_scale": float(sampled["tau_scale"]),
    }


def configure_sil(sil, sil_profile_txt, flightModeFlags, boxId, load_profile=False):
    if load_profile:
        sil.mockup.load_profile(sil_profile_txt)

    sil.mockup.setLogging(True)
    sil.sendMocap()
    sil.mockup.sendPositionSetpoint([0.0, 0.0, 0.0], 0.0)
    sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE)
    sil.mockup.enableRxBox(boxId.BOXTHROWTOARM)
    sil.mockup.enableRxBox(boxId.BOXARM)


def reboot_sil(sil, flightModeFlags, boxId):
    sil.mockup.saveConfig()
    configure_sil(sil, None, flightModeFlags, boxId, load_profile=False)


def sample_throw(base_throw, rng, sigma_cfg):
    throw_cfg = sigma_cfg["throw"]
    height = base_throw["height"] * rel_scale(rng, throw_cfg["height_rel_sigma"], min_scale=0.8, max_scale=1.2)
    wB = np.array(base_throw["wB"], dtype=np.float32)
    vHorz = np.array(base_throw["vHorz"], dtype=np.float32)

    wB = wB * rel_scale(rng, throw_cfg["wb_rel_sigma"], min_scale=0.6, max_scale=1.4)
    vHorz = vHorz * rel_scale(rng, throw_cfg["vhorz_rel_sigma"], min_scale=0.6, max_scale=1.4)

    return {
        "height": float(height),
        "wB": wB.tolist(),
        "vHorz": vHorz.tolist(),
    }


def run_single_sim(run_idx, args, base, rng, sigma_cfg, sil=None, flightModeFlags=None, boxId=None):
    tail, imu, sampled = create_randomized_craft(base, rng, sigma_cfg)
    ok = True
    err = ""

    try:
        if sil is None:
            raise ValueError("SIL not initialized")

        sil.uav = tail
        sil.imu = imu


        tail.setPose(x=[0.0, -3.5, -0.1], q=[0.707, 0.0, 0.0, 0.707])
        tail.setTwist(v=[0.0, 0.0, 0.0], w=[0.0, 0.0, 0.0])

        sim = Sim(tail, imu, None, None, sil)

        if not args.no_vis:
            print("\n\n##########################################\n")
            print("Welcome to the sim -- Starting visualization at http://localhost:5000")
            visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={"host": "0.0.0.0"})
            visThread.start()

        throw_base = {"height": 4.0, "wB": [-0.0, -4.0, 0.0], "vHorz": [0.0, 3.0], "at_time": 4.5}
        throw_sampled = sample_throw(throw_base, rng, sigma_cfg)
        tail.throw(
            height=throw_sampled["height"],
            wB=throw_sampled["wB"],
            vHorz=throw_sampled["vHorz"],
            at_time=4.5
        )
        sampled["throw"] = throw_sampled

        sim_time = 24.0
        dt = 0.000125

        goto_center = False
        goto_33 = False
        goto_back = False

        for _ in tqdm(range(int(sim_time / dt)), target_looptime=None, leave=False, desc=f"run {run_idx+1}/{args.runs}"):
            if sim.t > 3.0 and sil is not None:
                sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE)

            if not goto_center and sim.t > 10.0 and sil is not None:
                sil.mockup.sendPositionSetpoint([0.0, 0.0, -1.5], 0.0)
                goto_center = True

            if not goto_33 and sim.t > 15.0 and sil is not None:
                sil.mockup.sendPositionSetpoint([3.0, 3.0, -1.5], 0.0)
                goto_33 = True

            if not goto_back and sim.t > 18.0 and sil is not None:
                sil.mockup.sendPositionSetpoint([0.0, 0.0, -1.5], 0.0)
                goto_back = True

            sim.tick(dt)
    except Exception as exc:
        ok = False
        err = str(exc)

    sampled["ok"] = ok
    sampled["error"] = err
    return sampled

if __name__=="__main__":
    parser = make_parser()
    args = parser.parse_args()

    if args.runs < 1:
        raise ValueError("--runs must be >= 1")

    if args.sil and not os.path.isfile(args.sil):
        raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")
    if not os.path.isfile(args.sigma_config):
        raise FileNotFoundError(f"Sigma config not found: {args.sigma_config}")

    sys.path.append(os.path.join((os.path.dirname(args.sil)), "../../src/utils"))
    mockup_interface = importlib.import_module("indiflight_mockup_interface")
    flightModeFlags = mockup_interface.flightModeFlags
    boxId = mockup_interface.boxId

    base = get_base_model()
    base_tail = create_nominal_craft(base)
    sigma_cfg = load_sigma_config(args.sigma_config)

    # Create one SIL instance and reboot it between runs using saveConfig().
    sil_tail, sil_imu, _ = create_randomized_craft(base, np.random.default_rng(args.seed), sigma_cfg)
    sil = IndiflightSITLWrapper(sil_tail, sil_imu, args.sil, Nr=sil_tail.Nr, Ns=sil_tail.Ns)
    configure_sil(sil, args.sil_profile_txt, flightModeFlags, boxId, load_profile=True)

    rng = np.random.default_rng(args.seed)
    results = []
    for run_idx in range(args.runs):
        if run_idx > 0:
            reboot_sil(sil, flightModeFlags, boxId)

        run_no_vis = args.no_vis or (run_idx > 0)
        run_args = SimpleNamespace(**vars(args))
        run_args.no_vis = run_no_vis

        sampled = run_single_sim(run_idx, run_args, base, rng, sigma_cfg, sil=sil, flightModeFlags=flightModeFlags, boxId=boxId)
        results.append(sampled)

    sampled_tails = []
    for sampled in results:
        if sampled is not None and sampled.get("ok", False) and "rotor_X" in sampled and "cd" in sampled:
            params = sampled_to_tail_params(sampled)
            sampled_tails.append(create_tail_from_params(base, params))
        else:
            sampled_tails.append(None)

    out_dir = os.path.dirname(args.params_pkl)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    payload = {
        "seed": args.seed,
        "args": vars(args),
        "sigma_config": sigma_cfg,
        "base_tail": base_tail,
        "sampled": results,
        "sampled_tails": sampled_tails,
    }
    with open(args.params_pkl, "wb") as f:
        pickle.dump(payload, f)

    n_ok = sum(1 for r in results if r is not None and r.get("ok", False))
    print("\nMonte Carlo summary")
    print(f"  runs: {args.runs}")
    print(f"  successful: {n_ok}")
    print(f"  failed: {args.runs - n_ok}")
    print(f"  parameters pkl: {args.params_pkl}")

    mass_values = [r["mass"] for r in results if r is not None and r.get("ok", False) and "mass" in r]
    if mass_values:
        masses = np.array(mass_values, dtype=np.float64)
        print(f"  mass [kg]: mean={masses.mean():.4f}, std={masses.std():.4f}, min={masses.min():.4f}, max={masses.max():.4f}")

    if n_ok != args.runs:
        print("\nFailures:")
        for i, r in enumerate(results, start=1):
            if r is None or not r.get("ok", False):
                err = "no error message available" if r is None else r.get("error", "no error message available")
                print(f"  run {i}: {err}")
