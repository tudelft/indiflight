"""Actuation models: predicted IMU outputs from actuator states, without aerodynamics.

An ActuationModel predicts the IMU outputs [specific force at the IMU (3), body rate
derivative (3)] from rotor speeds, elevon angles, body rates and their derivatives, assuming
hover/no airflow. Everything that depends on the airspeed (or, for some models, on body
rates in a damping sense) is the aerodynamic part and has been stripped.

Outputs a model does not cover are NaN (e.g. the offline RLS models only predict moments).
"""

import os
import pickle
import sys

import numpy as np

OUTPUT_LABELS = ["$f_x$", "$f_y$", "$f_z$", "$\\dot p$", "$\\dot q$", "$\\dot r$"]
OUTPUT_UNITS = ["N/kg"] * 3 + ["rad/s$^2$"] * 3

# defaults for models that carry no mass/inertia (offline RLS, onboard learner)
DEFAULT_MASS = 0.564
DEFAULT_INERTIA = np.diag([5.73e-3, 1.35e-3, 5.43e-3])


def imu_lever_arm_acc(O, Odot, r):
    """Specific force at the IMU located at r (body frame) relative to the CG, minus CG specific force."""
    r = np.broadcast_to(np.asarray(r, dtype=float), O.shape)
    return np.cross(Odot, r) + np.cross(O, np.cross(O, r))


class ActuationModel:
    """Base class. Subclasses implement predict()."""
    name = "actuation model"
    m = None        # mass in kg, if the model defines one
    I = None        # inertia (3,3) in kg m^2, if the model defines one
    d0 = None       # zero-moment elevon angles (2,) in rad, if the model defines them

    def predict(self, fd):
        """Predicted [spf at IMU (3), Odot (3)] as (N,6); NaN for unmodelled outputs."""
        raise NotImplementedError

    def describe(self):
        return self.name

    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self, f)
        print(f"Saved actuation model '{self.name}' to {path}")


def load_actuation_model(path):
    add_pyndiflight_path()  # CraftActuationModel pickles reference PyNDIflight.crafts
    with open(path, 'rb') as f:
        model = pickle.load(f)
    if not isinstance(model, ActuationModel):
        raise TypeError(f"{path} does not contain an ActuationModel")
    return model


def add_pyndiflight_path():
    """Make support/simulation/PyNDIflight importable."""
    sim = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'simulation'))
    if sim not in sys.path:
        sys.path.append(sim)


def import_pyndiflight():
    """Returns the PyNDIflight (crafts, helpers) modules."""
    add_pyndiflight_path()
    from PyNDIflight import crafts, helpers
    return crafts, helpers


class CraftActuationModel(ActuationModel):
    """Actuation part of a PyNDIflight TailsitterPhi (or TailsitterPhiFF) instance.

    Evaluates the craft's own force functions per sample, with the measured actuator states:
    helpers.rotorForcesMoments + rotatingMassTorques + elevonForcesMoments, then fspB = F/m and
    helpers.angularRateDerivative, i.e. Craft.tick() without ground contact, throw forces,
    phiTheoryForcesMoments and (TailsitterPhiFF) the freestream elevon term. tick() itself is
    not used because it integrates motor/servo dynamics instead of taking measured wdot and
    elevon accelerations.

    include_rate_terms=False zeroes rotor wdot and elevon rate/acceleration, which is what
    lsPrototypingTailsitterFF.py effectively did by ticking the sim with tau=inf and s_dd=0.
    """

    def __init__(self, craft, name="TailsitterPhi", include_rate_terms=True):
        self.craft = craft
        self.name = name
        self.include_rate_terms = include_rate_terms
        self.m = float(craft.m)
        self.I = np.asarray(craft.I, dtype=float)
        self.d0 = np.asarray(craft.d0, dtype=float)
        self.stripped_phi = (craft.phi, np.asarray(craft.Phi))

    def describe(self):
        return (f"{self.name} ({type(self.craft).__name__} actuation part, rate terms "
                f"{'on' if self.include_rate_terms else 'off'})\n"
                f"  m = {self.m:.4g} kg, I = diag({', '.join(f'{v:.3e}' for v in np.diag(self.I))})\n"
                f"  d0 = {self.d0}")

    def predict(self, fd):
        _, h = import_pyndiflight()
        c = self.craft
        f32 = lambda a: np.ascontiguousarray(a, dtype=np.float32)
        w, d, O = f32(fd.w), f32(fd.d), f32(fd.O)
        rate = 1. if self.include_rate_terms else 0.
        wdot, dd, ddd = f32(rate * fd.wdot), f32(rate * fd.dd), f32(rate * fd.ddd)
        s = np.sign(c.r_cm)

        y = np.empty((fd.N, 6))
        for k in range(fd.N):
            FM = h.rotorForcesMoments(c.r_X, c.r_ax, w[k], c.r_k, c.r_cm)
            FM[3:] += h.rotatingMassTorques(c.r_I, c.r_ax, s * w[k], s * wdot[k], O[k])
            FM += h.elevonForcesMoments(w[k], d[k], dd[k], ddd[k], c.d0, c.cd, c.cdd, c.cddd)
            y[k, :3] = FM[:3] / c.m
            y[k, 3:] = h.angularRateDerivative(O[k], FM[3:], c.I, c.Iinv)
        return y


class OnboardLearnerActuationModel(ActuationModel):
    """Final estimate of the onboard learner (LEARNER mode, fxRls in flight/learner.c).

    Matches the non-symmetric LEARNER_IS_TAILSITTER regressors (learner.c, unchanged since at
    least 8a70341eb), with 2 motors (act 0, 1) and 2 servos (act 2, 3):

      spf_ax  = 0.1 * 1e-5 * (x0 w0^2 + x1 w1^2 + x2 w0^2 d0 + x3 w1^2 d1)    (x2, x3 unused for ax=x)
      Odot_ax = 1e-5 * (x1 w0^2 + x2 w1^2 + x3 w0^2 d0 + x4 w1^2 d1)          (x3, x4 unused for roll)
                - 0.1 * x7 * |Omega| * Omega_ax                                  <- aero, stripped
      (x0: inertia ratio, regressor 0; x5, x6, x8: elevon rate/acc, regressors 0)

    Logged as fx_{x,y,z}_rls_x[i] = 1e3 x[i] and fx_{p,q,r}_rls_x[i] = 1e3 x[1+i],
    fx_{p,q,r}_rls_x[N+i] = 1e3 x[5+i], with N = BLACKBOX_LEARNER_N (half the number of columns).

    The learner's specific force target is corrected to the CG with the acc_offset config (not
    logged); r_imu adds that lever arm back so the prediction matches the raw accelerometer.
    No gyroscopic term is modelled (inertia ratio regressor is 0 in firmware).
    """

    FORCE_MASKS = np.array([[1, 1, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]], dtype=float)
    MOMENT_MASKS = np.array([[1, 1, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]], dtype=float)

    def __init__(self, x_force, x_moment, x_aero, name="onboard learner", r_imu=(0., 0., 0.), info=None):
        self.name = name
        self.x_force = np.asarray(x_force, dtype=float)    # (3,4) unscaled fxRls[0..2].x
        self.x_moment = np.asarray(x_moment, dtype=float)  # (3,4) unscaled fxRls[3..5].x[1..4]
        self.x_aero = np.asarray(x_aero, dtype=float)      # (3,) unscaled fxRls[3..5].x[7] (stripped)
        self.r_imu = np.asarray(r_imu, dtype=float)
        self.info = info or {}

    @classmethod
    def from_log_data(cls, data, row=-1, name=None, r_imu=(0., 0., 0.)):
        n_cols = sum(c.startswith('fx_p_rls_x[') for c in data.columns)
        if n_cols == 0:
            raise KeyError("Log has no onboard learner fields (fx_p_rls_x). Was it recorded with USE_LEARNER?")
        Nh = n_cols // 2
        r = data.iloc[row]
        x_force = np.array([[r[f'fx_{ax}_rls_x[{i}]'] for i in range(4)] for ax in 'xyz']) * 1e-3
        x_moment = np.array([[r[f'fx_{ax}_rls_x[{i}]'] for i in range(4)] for ax in 'pqr']) * 1e-3
        x_aero = np.array([r[f'fx_{ax}_rls_x[{Nh + 2}]'] for ax in 'pqr']) * 1e-3
        unused = np.array([[r[f'fx_{ax}_rls_x[{Nh + j}]'] for j in (0, 1, 3)] for ax in 'pqr'])
        t = float(r['timeS'])
        if np.any(unused != 0) or np.any(x_force[0, 2:] != 0) or np.any(x_moment[0, 2:] != 0):
            print("WARNING: onboard learner parameters that should be unused are non-zero. The log was "
                  "probably recorded with a different learner regressor layout than assumed here.")
        return cls(x_force, x_moment, x_aero, name=name or f"onboard learner (t = {t:.2f} s)",
                   r_imu=r_imu, info=dict(time=t, n_cols=n_cols))

    def describe(self):
        np.set_printoptions(precision=4, suppress=False)
        return (f"{self.name}\n"
                f"  force  x (fx_xyz_rls_x[0..3] * 1e-3):\n{self.x_force}\n"
                f"  moment x (fx_pqr_rls_x[0..3] * 1e-3):\n{self.x_moment}\n"
                f"  stripped aero damping x7 (p, q, r): {self.x_aero}\n"
                f"  IMU offset used: {self.r_imu}")

    def regressors(self, fd):
        w2 = fd.w**2
        return np.stack((w2[:, 0], w2[:, 1], w2[:, 0] * fd.d[:, 0], w2[:, 1] * fd.d[:, 1]), axis=1)  # (N,4)

    def predict(self, fd):
        A = 1e-5 * self.regressors(fd)
        spf = 0.1 * A @ (self.x_force * self.FORCE_MASKS).T
        spf += imu_lever_arm_acc(fd.O, fd.Odot, self.r_imu)
        Odot = A @ (self.x_moment * self.MOMENT_MASKS).T
        return np.hstack((spf, Odot))
