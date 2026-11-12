"""Library port of the moment RLS models of rlsPrototypingTailsitter.py.

Preprocessing, regressors, initial covariances and forgetting factors are copied from
rlsPrototypingTailsitter.py (which is left unchanged). The per-sample RLS recursion is the
same as estimators.RLS.update(), but only the parameter history is kept (estimators.RLS logs
P at every sample, which gets large with 11 models).

All models predict the body rate derivative only (3 outputs); there is no force model.

Regressor families (columns):
  'phi9' (21): 0:3 inertia ratios | 3:6 w^2 | 6:8 w^2 d | 8 wdot | 9:12 elevon rates (zeroed)
               | 12:16 C_mv | 16:19 rate damping diag | 19:21 rate damping cross
  'act'  (24): 0:3 inertia ratios | 3:21 per-motor w^2, w^2 d, (wdot zeroed) for p, q, r
               | 21:24 rate damping diag
Aerodynamic columns (stripped for the actuation model): phi9 12:21, act 21:24.
"""

import numpy as np
from tqdm import tqdm

from indiflight_log_tools.signal_tools import Signal

from .actuation import ActuationModel

FAMILY_N = {'phi9': 21, 'act': 24}
FAMILY_AERO_COLS = {'phi9': list(range(12, 21)), 'act': list(range(21, 24))}

_PHI9_COV = 1e-11 * np.diag([1e9, 1e9, 1e9,  # inertia terms
                             1, 1, 1,  # Cw2
                             1e-1, 1e-1,  # Cw2d (y, z only)
                             1e+2,  # Cwdot (z only)
                             1e+6, 1e+6,  # Cddot (y, z only)
                             1e+4,  # Cddotdot (y only)
                             1e9, 1e9, 1e9, 1e9,  # C_p_vy, C_q_vx, C_q_vz, C_r_vy
                             1e9, 1e9, 1e9, 1e9, 1e9,  # rate damping diagonal, and C_r_wx
                             ])
_ACT_COV = 1e-12*np.diag([1e11, 1e11, 1e11,    1, 1, 1e3,  1, 1, 1e3,    1, 1, 1e3,  1, 1, 1e3,    1, 1, 1e3,  1, 1, 1e3,    1e11, 1e11, 1e11])
FAMILY_COV = {'phi9': _PHI9_COV, 'act': _ACT_COV}
FORGETTING = 0.9999

# name: (family, columns zeroed in the regressor, title as in rlsPrototypingTailsitter.py;
#        act_* titles repeat the phi9 ones there, the names tell them apart)
MODELS = {
    'phi9':          ('phi9', [],                                                 "RLS Moments -- Inertias, Actuators and 9-param Phi"),
    'phi9_noI':      ('phi9', [0, 1, 2],                                          "RLS Moments -- Actuators and 9-param Phi"),
    'phi7_noI':      ('phi9', [0, 1, 2, 19, 20],                                  "RLS Moments -- Actuators and 7-param Phi"),
    'phi3_noI':      ('phi9', [0, 1, 2, 19, 20, 12, 13, 14, 15],                  "RLS Moments -- Actuators and 3-param Phi"),
    'noPhi_noI':     ('phi9', [0, 1, 2, 19, 20, 12, 13, 14, 15, 16, 17, 18],      "RLS Moments -- Actuators only"),
    'phi7':          ('phi9', [19, 20],                                           "RLS Moments -- Inertias, Actuators and 7-param Phi"),
    'phi3':          ('phi9', [19, 20, 12, 13, 14, 15],                           "RLS Moments -- Inertias, Actuators and 3-param Phi"),
    'noPhi':         ('phi9', [19, 20, 12, 13, 14, 15, 16, 17, 18],               "RLS Moments -- Inertias and Actuators only"),
    'act_phi3_I':    ('act',  [],                                                 "RLS Moments -- Inertias, Actuators and 3-param Phi"),
    'act_phi3_noI':  ('act',  [0, 1, 2],                                          "RLS Moments -- Actuators and 3-param Phi"),
    'act_noPhi_noI': ('act',  [0, 1, 2, 21, 22, 23],                              "RLS Moments -- Actuators only"),
}


def prepare_data(data, freq_hz=15, order=2):
    """Preprocessing of rlsPrototypingTailsitter.py on a (cropped) log DataFrame."""
    N = data.shape[0]
    n = 2
    t_raw = data["timeS"].to_numpy()
    O_raw = data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
    w_raw = data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
    d_raw = data[[f'servo_feedback[{i}]' for i in range(2)]].to_numpy()
    q_raw = data[[f"quat[{i}]" for i in range(4)]].to_numpy()
    v_raw = data[[f"localVel[{i}]" for i in range(3)]].to_numpy()

    t = np.linspace(t_raw[0], t_raw[-1], N)
    Of = Signal(t_raw, O_raw, rebase=t).filter("lowpass", order, freq_hz)
    wf = Signal(t_raw, w_raw, rebase=t).filter("lowpass", order, freq_hz)
    df = Signal(t_raw, d_raw, rebase=t).filter("lowpass", order, freq_hz)
    qf = Signal(t_raw, q_raw, rebase=t).filter("lowpass", order, freq_hz)
    vf = Signal(t_raw, v_raw, rebase=t).filter("lowpass", order, freq_hz)

    import quaternion
    qfy_norm_inv = quaternion.as_quat_array((qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
    R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)
    v_B = (R_IB @ vf.y[:, :, np.newaxis]).squeeze()

    return dict(t=t, O=Of.y, Odot=Of.dot().y, w=wf.y, wdot=wf.dot().y, d=df.y, vB=v_B,
                signals=dict(Of=Of, wf=wf, df=df, qf=qf, vf=vf))


def regressors(family, O, w, wdot, d, vB=None, eta=None):
    """Regressor matrices (N,3,n) of rlsPrototypingTailsitter.py, vectorized.

    vB/eta are only needed for the aero columns; if None, the aero columns are zero.
    eta = |vB| in the original script (phi = 0).
    """
    N = O.shape[0]
    w2 = w**2
    w2d = w2 * d
    A = np.zeros((N, 3, FAMILY_N[family]))
    # inertia_regs = -diag([q r, r p, p q])
    A[:, 0, 0] = -O[:, 1] * O[:, 2]
    A[:, 1, 1] = -O[:, 2] * O[:, 0]
    A[:, 2, 2] = -O[:, 0] * O[:, 1]
    if vB is not None and eta is None:
        eta = np.linalg.norm(vB, axis=1)

    if family == 'phi9':
        A[:, 0, 3] = w2[:, 0] - w2[:, 1]
        A[:, 1, 4] = w2[:, 0] + w2[:, 1]
        A[:, 2, 5] = w2[:, 0] - w2[:, 1]
        A[:, 1, 6] = w2d[:, 0] + w2d[:, 1]
        A[:, 2, 7] = w2d[:, 0] - w2d[:, 1]
        A[:, 2, 8] = wdot[:, 0] - wdot[:, 1]
        # 9:12 elevon rate/acc terms are multiplied by 0 in the original
        if vB is not None:
            A[:, 0, 12] = -eta * vB[:, 1]  # C_p_vy
            A[:, 1, 13] = -eta * vB[:, 0]  # C_q_vx
            A[:, 1, 14] = -eta * vB[:, 2]  # C_q_vz
            A[:, 2, 15] = -eta * vB[:, 1]  # C_r_vy
            for i in range(3):
                A[:, i, 16 + i] = -eta * O[:, i]  # C_m_w
            A[:, 2, 19] = -eta * O[:, 0]  # C_r_wx
            A[:, 0, 20] = -eta * O[:, 2]  # C_p_wz
    elif family == 'act':
        # |  w1*w1  |  w1*w1*delta1  |  (wdot1 zeroed)  |  w2*w2  |  w2*w2*delta2  |  (wdot2 zeroed)  |
        for ax in range(3):
            c = 3 + 6 * ax
            A[:, ax, c + 0] = w2[:, 0]
            A[:, ax, c + 1] = w2d[:, 0]
            A[:, ax, c + 3] = w2[:, 1]
            A[:, ax, c + 4] = w2d[:, 1]
        if vB is not None:
            for i in range(3):
                A[:, i, 21 + i] = -eta * O[:, i]
    else:
        raise ValueError(f"unknown family {family}")
    return A


class LeanRLS:
    """Same recursion as estimators.RLS.update() (theta, P only; theta history kept)."""

    def __init__(self, n, d, P0, forgetting):
        self.theta = np.zeros((n, 1))
        self.P = np.array(P0, dtype=float)
        self.lam = forgetting
        self.d = d
        self.theta_h = []

    def update(self, A, y):
        e = y[:, np.newaxis] - A @ self.theta
        M = self.lam * np.eye(self.d) + A @ self.P @ A.T
        K = (self.P @ A.T) @ np.linalg.inv(M)
        self.theta += K @ e
        self.P = (self.P - K @ A @ self.P) / self.lam
        self.theta_h.append(self.theta[:, 0].copy())


def make_full_estimator(name):
    """estimators.RLS configured as in rlsPrototypingTailsitter.py (keeps full history, for plots)."""
    from estimators import RLS
    family, _, title = MODELS[name]
    est = RLS(FAMILY_N[family], 3, gamma=1e-11, forgetting=FORGETTING)
    est.setTitle(title)
    est.setParameters(np.zeros(FAMILY_N[family]))
    est.setCovariance(FAMILY_COV[family])
    return est


def run(prep, model_names=None, progress=True, full=False):
    """Run the selected moment RLS models over prepared data.

    full=False: returns {name: LeanRLS} (theta history only, as an array).
    full=True:  returns {name: estimators.RLS} with complete history, for plotParameters().
    """
    model_names = list(MODELS) if model_names is None else list(model_names)
    families = sorted({MODELS[n][0] for n in model_names})
    A_full = {f: regressors(f, prep['O'], prep['w'], prep['wdot'], prep['d'], vB=prep['vB']) for f in families}

    estimators = {}
    masks = {}
    for name in model_names:
        family, zero_cols, _ = MODELS[name]
        estimators[name] = (make_full_estimator(name) if full
                            else LeanRLS(FAMILY_N[family], 3, FAMILY_COV[family], FORGETTING))
        mask = np.ones(FAMILY_N[family])
        mask[zero_cols] = 0.
        masks[name] = mask

    y = prep['Odot']
    t = prep['t']
    for k in tqdm(range(len(y)), disable=not progress, desc="offline RLS"):
        for name, est in estimators.items():
            A = A_full[MODELS[name][0]][k] * masks[name]
            if full:
                est.newSample(A, y[k], float(t[k]))
                est.update()
            else:
                est.update(A, y[k])

    if not full:
        for est in estimators.values():
            est.theta_h = np.array(est.theta_h)
    return estimators


def rmse(prep, name, theta, strip_aero=False):
    """RMSE (rad/s^2) of a model with parameters theta over the prepared data.

    strip_aero: evaluate without the aero columns (the actuation model used downstream).
    """
    family, zero_cols, _ = MODELS[name]
    A = regressors(family, prep['O'], prep['w'], prep['wdot'], prep['d'], vB=None if strip_aero else prep['vB'])
    A[:, :, zero_cols] = 0.
    e = A @ theta - prep['Odot']
    return float(np.sqrt(np.mean(np.sum(e**2, axis=1))))


class OfflineRLSActuationModel(ActuationModel):
    """Final parameters of one offline moment RLS model, with the aero columns stripped.

    Predicts only Odot (forces are NaN). Inertia-ratio columns (gyroscopic terms) are kept for
    the models that have them.
    """

    def __init__(self, model_name, theta, name=None, info=None):
        self.model_name = model_name
        self.family, self.zero_cols, self.title = MODELS[model_name]
        self.theta = np.asarray(theta, dtype=float).copy()
        self.name = name or f"offline RLS '{model_name}'"
        self.info = info or {}

    def describe(self):
        aero = FAMILY_AERO_COLS[self.family]
        return (f"{self.name}: {self.title}\n"
                f"  theta = {np.array2string(self.theta, precision=3, max_line_width=150)}\n"
                f"  stripped aero columns {aero[0]}:{aero[-1] + 1}; predicts moments only")

    def predict(self, fd):
        A = regressors(self.family, fd.O, fd.w, fd.wdot, fd.d, vB=None)  # aero columns stay 0
        A[:, :, self.zero_cols] = 0.
        Odot = A @ self.theta
        return np.hstack((np.full((fd.N, 3), np.nan), Odot))
