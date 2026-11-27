"""Phi-theory aerodynamic model fitted to the residual of an actuation model.

Assumes the aerodynamic forces/moments are the only unmodelled ones:

    F_aero = m (spf_obs - spf_act)          M_aero = I (Odot_obs - Odot_act)

and fits  [F_aero; M_aero] = -eta Phi eta_B  +  elevon freestream terms,  with
eta_B = [u v w p q r] (body airspeed and rates), eta = sqrt(|v|^2 + phi |Omega|^2), and a
symmetric Phi with the sparsity of lsPrototypingTailsitterFF.py (12 coefficients):

        u     v     w     p     q     r
  u  [ Cxu   .    Czu    .    Cmu    .  ]
  v  [  .   Cyv    .    Clv    .    Cnv ]
  w  [ Czu   .    Czw    .    Cmw    .  ]
  p  [  .   Clv    .    Clp    .    Cnp ]
  q  [ Cmu   .    Cmw    .    Cmq    .  ]
  r  [  .   Cnv    .    Cnp    .    Cnr ]

Elevon freestream terms:  M_y += -eta w Ce_m (dd0 + dd1),  M_z += -eta w Ce_n (dd0 - dd1),
with dd = d - d0. Phi is directly usable in TailsitterPhi.setPhiModel(phi, Phi).

Unlike lsPrototypingTailsitterFF.py, the regression is done in force/moment units (not
N/kg and rad/s^2), so the shared force/moment entries of Phi are physically consistent.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

PARAM_NAMES = ["Cxu", "Czu", "Cyv", "Czw", "Cmu", "Clv", "Cnv", "Cmw", "Clp", "Cnp", "Cmq", "Cnr", "Ce_m", "Ce_n"]
# (row, col) in Phi for each of the 12 phi parameters (mirrored to (col, row))
PHI_ENTRIES = {
    "Cxu": (0, 0), "Czu": (0, 2), "Cyv": (1, 1), "Czw": (2, 2), "Cmu": (0, 4), "Clv": (1, 3),
    "Cnv": (1, 5), "Cmw": (2, 4), "Clp": (3, 3), "Cnp": (3, 5), "Cmq": (4, 4), "Cnr": (5, 5),
}
PARAM_GROUPS = {
    "velocity": ["Cxu", "Czu", "Cyv", "Czw"],
    "coupling": ["Cmu", "Clv", "Cnv", "Cmw"],
    "rate": ["Clp", "Cnp", "Cmq", "Cnr"],
    "elevon": ["Ce_m", "Ce_n"],
}


def aero_regressors(vB, O, d, d0, phi=0.):
    """Regressor matrices (N,6,14) mapping PARAM_NAMES to [F (3), M (3)]."""
    N = vB.shape[0]
    eta = np.sqrt(np.sum(vB**2, axis=1) + phi * np.sum(O**2, axis=1))
    etaB = np.hstack((vB, O))
    A = np.zeros((N, 6, len(PARAM_NAMES)))
    for k, name in enumerate(PARAM_NAMES[:12]):
        i, j = PHI_ENTRIES[name]
        A[:, i, k] = -eta * etaB[:, j]
        if i != j:
            A[:, j, k] = -eta * etaB[:, i]
    dd = d - np.asarray(d0)[np.newaxis]
    w = vB[:, 2]
    A[:, 4, 12] = -eta * w * (dd[:, 0] + dd[:, 1])
    A[:, 5, 13] = -eta * w * (dd[:, 0] - dd[:, 1])
    return A


def phi_matrix(theta):
    Phi = np.zeros((6, 6))
    for k, name in enumerate(PARAM_NAMES[:12]):
        i, j = PHI_ENTRIES[name]
        Phi[i, j] = Phi[j, i] = theta[k]
    return Phi


def r2(y, yhat):
    ss_tot = np.sum((y - y.mean())**2)
    return 1. - np.sum((y - yhat)**2) / ss_tot if ss_tot > 0 else np.nan


@dataclass
class AeroFitResult:
    theta: np.ndarray        # (14,)  NaN for unidentifiable parameters
    theta_std: np.ndarray    # (14,)  nominal 1-sigma (assumes white residuals, so optimistic)
    Phi: np.ndarray          # (6,6)
    phi: float
    wind: np.ndarray         # (3,)
    d0: np.ndarray           # (2,)
    rows: np.ndarray         # (6,) bool, outputs used in the fit
    weights: np.ndarray      # (6,) row weights
    FM_res: np.ndarray       # (N,6) residual forces/moments (target of the fit)
    FM_hat: np.ndarray       # (N,6) fitted aero forces/moments
    r2: np.ndarray           # (6,) per-output R^2 of the aero fit to the residual
    cost: float              # weighted residual sum of squares / N


def fit_aero(fd, y_act, m, I, d0, wind=(0., 0., 0.), phi=0., weights=None):
    """Least-squares fit of the aero model to the actuation residual.

    fd:      FlightData
    y_act:   (N,6) actuation model prediction (NaN columns are excluded from the fit)
    weights: (6,) row weights; default 1/std of each residual row.
    """
    wind = np.asarray(wind, dtype=float)
    d0 = np.asarray(d0, dtype=float)
    nu = fd.y_obs() - y_act
    FM_res = np.hstack((m * nu[:, :3], nu[:, 3:] @ np.asarray(I).T))

    rows = np.all(np.isfinite(FM_res), axis=0)
    if weights is None:
        sd = np.zeros(6)
        sd[rows] = np.std(FM_res[:, rows], axis=0)
        weights = np.where(sd > 0, 1. / np.where(sd > 0, sd, 1.), 0.)
    weights = np.asarray(weights, dtype=float)

    A = aero_regressors(fd.vB(wind), fd.O, fd.d, d0, phi=phi)
    Aw = (A[:, rows] * weights[rows][np.newaxis, :, np.newaxis]).reshape(-1, A.shape[2])
    yw = (FM_res[:, rows] * weights[rows][np.newaxis]).reshape(-1)

    # parameters without excitation (e.g. force-only parameters when only moments are modelled)
    ident = np.linalg.norm(Aw, axis=0) > 0
    theta = np.full(A.shape[2], np.nan)
    theta_std = np.full(A.shape[2], np.nan)
    sol, _, _, _ = np.linalg.lstsq(Aw[:, ident], yw, rcond=None)
    theta[ident] = sol

    e = yw - Aw[:, ident] @ sol
    dof = max(len(yw) - ident.sum(), 1)
    s2 = e @ e / dof
    try:
        cov = s2 * np.linalg.inv(Aw[:, ident].T @ Aw[:, ident])
        theta_std[ident] = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        pass

    th0 = np.nan_to_num(theta)
    FM_hat = A @ th0
    FM_hat[:, ~rows] = np.nan
    r2s = np.array([r2(FM_res[:, i], FM_hat[:, i]) if rows[i] else np.nan for i in range(6)])

    return AeroFitResult(theta=theta, theta_std=theta_std, Phi=phi_matrix(th0), phi=phi, wind=wind,
                         d0=d0, rows=rows, weights=weights, FM_res=FM_res, FM_hat=FM_hat, r2=r2s,
                         cost=float(e @ e) / fd.N)


def fit_aero_outer(fd, y_act, m, I, d0, wind0=(0., 0.), phi0=0., fit_wind=False, fit_phi=False,
                   wind_bound=10., phi_bound=(0., 2.)):
    """Optionally optimize the horizontal wind and/or phi (nonlinear) around fit_aero (linear).

    The row weights are fixed from the zero-wind, phi0 fit so the cost is comparable.
    """
    base = fit_aero(fd, y_act, m, I, d0, wind=(*wind0, 0.), phi=phi0)
    if not (fit_wind or fit_phi):
        return base

    def unpack(z):
        k = 0
        wind = np.array([*wind0, 0.])
        phi = phi0
        if fit_wind:
            wind[:2] = z[0:2]
            k = 2
        if fit_phi:
            phi = z[k]
        return wind, phi

    def cost(z):
        wind, phi = unpack(z)
        return fit_aero(fd, y_act, m, I, d0, wind=wind, phi=phi, weights=base.weights).cost

    z0, bounds = [], []
    if fit_wind:
        z0 += list(wind0)
        bounds += [(-wind_bound, wind_bound)] * 2
    if fit_phi:
        z0 += [phi0]
        bounds += [phi_bound]
    opt = minimize(cost, np.array(z0, dtype=float), method="Powell", bounds=bounds,
                   options={"maxiter": 50, "xtol": 1e-3, "ftol": 1e-5})
    wind, phi = unpack(opt.x)
    print(f"outer optimization: wind = {wind[:2]} m/s, phi = {phi:.4g}, cost {base.cost:.4g} -> {opt.fun:.4g}")
    return fit_aero(fd, y_act, m, I, d0, wind=wind, phi=phi, weights=base.weights)


def format_scientific(num):
    if not np.isfinite(num) or abs(num) < 1e-12:
        return "         0"
    return f"+{num:.3e}" if num >= 0 else f"{num:.3e}"


def print_result(res):
    print("\nAero parameters (nominal 1-sigma, assumes white residuals):")
    for name, th, sd in zip(PARAM_NAMES, res.theta, res.theta_std):
        if np.isfinite(th):
            print(f"  {name:5s} = {th:+.3e} +- {sd:.1e}")
        else:
            print(f"  {name:5s} =  not identifiable (output not modelled)")
    labels = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
    print("R^2 of aero fit to residual: " + ", ".join(
        f"{l} {v:.3f}" if np.isfinite(v) else f"{l} -" for l, v in zip(labels, res.r2)))
    print(f"wind = [{res.wind[0]:.3f}, {res.wind[1]:.3f}] m/s, d0 = {res.d0}")
    print("\n# paste-ready for TailsitterPhi.setPhiModel(phi, Phi)")
    print(f"phi = {res.phi:.3e}")
    print("Phi = np.array([")
    for row in res.Phi:
        print("    [" + ", ".join(format_scientific(v) for v in row) + "],")
    print("], dtype=np.float32)")
    print(f"# elevon freestream: My += -eta*w*Ce_m*(dd0+dd1), Mz += -eta*w*Ce_n*(dd0-dd1), dd = d - d0")
    print(f"Ce_m, Ce_n = {format_scientific(res.theta[12])}, {format_scientific(res.theta[13])}")
    ev = np.linalg.eigvalsh(res.Phi)
    print(f"eigenvalues of Phi: {np.array2string(ev, precision=3)}  ({'positive definite' if ev.min() > 0 else 'NOT positive definite'})")
