"""Library port of tailsitterGlobalPhi.py: global LBFGS fit of a phi-theory tailsitter model.

The model, data preprocessing, fit, and the parameter extraction/printing are copied from
tailsitterGlobalPhi.py (which is left unchanged). Differences:
  - the xy target rotation test (ROTATE_XY_TARGETS) is off by default (xy_rotation=None),
  - GlobalPhiTailsitter.forward() takes include_aero, and
  - GlobalPhiActuationModel evaluates the fitted actuation part (aero stripped) in numpy on
    any FlightData.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from scipy.constants import g as GRAVITY
from scipy.signal import savgol_filter, butter, sosfilt, sosfilt_zi

from .actuation import ActuationModel


def quaternion_rotate(q, v, inverse=False):
    # helper function to rotate a vector with a quaternion
    w, x, y, z = (-1 if inverse else 1) * q[0, :], q[1, :], q[2, :], q[3, :]
    vx, vy, vz = v[0, :], v[1, :], v[2, :]

    tx =  w * vx + y * vz - z * vy
    ty =  w * vy + z * vx - x * vz
    tz =  w * vz + x * vy - y * vx
    tw = -x * vx - y * vy - z * vz

    # Compute result quaternion (q * v * q_conj)
    rx = tw * -x + tx *  w + ty * -z - tz * -y
    ry = tw * -y + ty *  w + tz * -x - tx * -z
    rz = tw * -z + tz *  w + tx * -y - ty * -x

    return torch.vstack([rx, ry, rz])


def inertiaFromPendulumPeriod(P, R, m):
    # physical pendulum with centroid R from the rotation point and mass m, period P.
    # T = 2pi * sqrt(I/(mgR)), and I_axle = I + m R^2
    Iaxle = (m*GRAVITY*R*P*P) / (4*np.pi*np.pi)
    return Iaxle - m*R*R


ACTUATION_COEFFS = [
    'cxww',         'cxd', 'cxdd',
    'cyww',         'cyd', 'cydd',
    'czww',         'czd2',
    'clww',
    'cmww',         'cmd',  'cmdd', 'cmddd',
    'cnww', 'cnwd', 'cnd',  'cndd',
]
PHI_COEFFS = [
    'cxu',
    'cyu',  'cyv',
    'czu',  'czv',  'czw',
            'clv',        'clp',
    'cmu',         'cmw',        'cmq',
            'cnv',        'cnp',        'cnr',
]
# lower-triangle position of each phi coefficient
PHI_INDEX = {
    'cxu': (0, 0), 'cyu': (1, 0), 'cyv': (1, 1), 'czu': (2, 0), 'czv': (2, 1), 'czw': (2, 2),
    'clv': (3, 1), 'clp': (3, 3), 'cmu': (4, 0), 'cmw': (4, 2), 'cmq': (4, 4),
    'cnv': (5, 1), 'cnp': (5, 3), 'cnr': (5, 5),
}

# regressor layout of x and its scaling
#            body vel    body rate   rate deriv  prop speed  prop accel  elevon   elev vel  elev acc
X_LAYOUT = ['vx', 'vy', 'vz', 'Ox', 'Oy', 'Oz', 'Odx', 'Ody', 'Odz', 'w1', 'w2', 'w1d', 'w2d',
            'd1', 'd2', 'd1d', 'd2d', 'd1dd', 'd2dd']
SCALE = np.array([1,1,1, 1,1,1, 1,1,1, 1e-3,1e-3, 5e-2,5e-2, 1,1, .1,.1, .01,.01], dtype=np.float32)

DEFAULT_EXCLUDE = ['czd2', 'cndd', 'cmww']


class GlobalPhiTailsitter(nn.Module):
    """Tailsitter class of tailsitterGlobalPhi.py, with forward(x, include_aero)."""

    def __init__(self):
        super().__init__()

        self.r = nn.Parameter(torch.zeros(3, 1))  # IMU offset
        self.d0 = nn.Parameter(torch.zeros(2, 1)) # 0-force elevon angle
        self.sqrtphi = nn.Parameter(1*torch.ones(1))

        for name in ACTUATION_COEFFS + PHI_COEFFS:
            setattr(self, name, nn.Parameter(torch.zeros(1)))

        # measured inertia by oscillation period
        self.m = 0.564
        Ixx = inertiaFromPendulumPeriod(P=4.538/5, R=85e-3, m=self.m)  # roll
        Iyy = inertiaFromPendulumPeriod(P=3.315/5, R=79e-3, m=self.m)  # pitch
        Izz = inertiaFromPendulumPeriod(P=4.534/5, R=131e-3, m=self.m) # yaw
        Inp = np.diag([Ixx, Iyy, Izz]).astype(np.float32) # (5.73, 1.35, 5.43) e-3 kg m^2

        self.I = torch.tensor( Inp )
        self.Iinv = torch.tensor( np.linalg.inv( Inp ) )

    def phi_matrix_torch(self):
        PHI = torch.zeros((6,6))
        for name, (i, j) in PHI_INDEX.items():
            PHI[i, j] = getattr(self, name)
        return PHI + PHI.T - torch.diag(torch.diagonal(PHI))  # make symmetric

    def forward(self, x, include_aero=True):
        vx, vy, vz,  Ox, Oy, Oz,   Odx, Ody, Odz,      w1, w2, w1d, w2d,           d1, d2,       d1d, d2d,       d1dd, d2dd =  x

        v  = torch.vstack((vx, vy, vz))
        O  = torch.vstack((Ox, Oy, Oz))
        Od = torch.vstack((Odx, Ody, Odz))

        eta = torch.sqrt(torch.sum(v**2, axis=0) + self.sqrtphi**2 * torch.sum(O**2, axis=0))

        # REGRESSOR primitives
        ww = torch.stack([w1*w1, w2*w2])                              # prop speeds ** 2
        wwDd = ww * ( (torch.stack([d1, d2]) - self.d0) )             # prop speeds ** 2 * (elevon angles - zero-moment angle)
        d2abs = torch.sin( d1 ).abs() + torch.sin( d2 ).abs()         # for reduction of the prop thrust

        # AERO MODEL using phi-theory
        PHI = self.phi_matrix_torch()
        eta_B = torch.vstack([vx, vy, vz, Ox, Oy, Oz])
        screw = - eta * ( PHI @ eta_B )
        if not include_aero:
            screw = torch.zeros_like(screw)

        # ACTUATION MODEL
        #      prop thrust contribution         prop rate contribution           elevon contribution           elev rate contrib              elev acc contrib
        screw[0] +=  self.cxww * (ww[0] + ww[1])  +  0                        +  self.cxd * (wwDd[0] + wwDd[1])  +  self.cxdd * (d1d + d2d)
        screw[1] +=  self.cyww * (ww[0] + ww[1])  +  0                        +  self.cyd * (wwDd[0] + wwDd[1])  +  self.cydd * (d1d + d2d)
        screw[2] +=  self.czww * (ww[0] + ww[1])  +  0                        +  self.czd2 * d2abs               +  0
        screw[3] +=  self.clww * (ww[0] - ww[1])  +  0                        +  0                               +  0
        screw[4] +=  self.cmww * (ww[0] + ww[1])  +  0                        +  self.cmd * (wwDd[0] + wwDd[1])  +  self.cmdd * (d1d + d2d)  +   self.cmddd * (d1dd + d2dd)
        screw[5] +=  self.cnww * (ww[0] - ww[1])  +  self.cnwd * (w1d - w2d)  +  self.cnd * (wwDd[0] - wwDd[1])  +  self.cndd * (d1d - d2d)

        f = screw[:3]
        m = screw[3:]

        # KINETICS
        a_modelled = f / self.m  +  Od.cross(self.r, dim=0)  +  O.cross(O.cross(self.r, dim=0), dim=0)
        dO_modelled = self.Iinv @ ( m  -  O.cross(self.I @ O, dim=0) )

        return torch.concat([a_modelled, dO_modelled], dim=0)


def prepare_data(data, use_acc_smooth=True, order=2, freq=10, servo1_sign=1., xy_rotation=None, device="cpu"):
    """Preprocessing of tailsitterGlobalPhi.py.

    data: (cropped) log DataFrame.
    xy_rotation: None, or an angle in rad to rotate the xy force targets and body velocity by
                 (the ROTATE_XY_TARGETS test of the original script).
    Returns a dict with x (scaled regressors, 19xN tensor), y_true (6xN tensor), t, series, ...
    """
    data = data.copy()
    t = data['timeS']
    mean_dt = t.diff().mean()
    data['servo_feedback[1]'] *= servo1_sign  # set to -1 for old datafiles

    acc_key = 'accSmooth' if use_acc_smooth else 'accADCafterRpm'
    series = {'localVel': range(3), 'localQuat': range(4), acc_key: range(3),
              'gyroADCafterRpm': range(3), 'omegaUnfiltered': range(2), 'servo_feedback': range(2),
              'motor': range(2), 'u': range(4)}
    butter_sos = butter(order, Wn=freq, btype='lowpass', output='sos', fs=1/mean_dt)
    # with accSmooth, all other signals get an additional causal filter to match its onboard filtering
    extra_sos = butter(order, Wn=50, btype='lowpass', output='sos', fs=1/mean_dt)
    for key, val in series.items():
        tmp = data[[f'{key}[{i}]' for i in val]].to_numpy(dtype=np.float32).T
        zi = sosfilt_zi(butter_sos)
        zis = np.repeat(zi, len(val), axis=0) * tmp[:, 0, np.newaxis]
        tmp_f = sosfilt(butter_sos, tmp, zi=zis[np.newaxis])[0]
        if use_acc_smooth and key != acc_key:
            zi = sosfilt_zi(extra_sos)
            zis = np.repeat(zi, len(val), axis=0) * tmp_f[:, 0, np.newaxis]
            tmp_f = sosfilt(extra_sos, tmp_f, zi=zis[np.newaxis])[0]
        tmp_f = torch.tensor( tmp_f, dtype=torch.float32 ).to(device=device)
        tmp_fd  = torch.tensor( savgol_filter(tmp_f, window_length=25, polyorder=3, deriv=1, delta=mean_dt) ).to(device=device)
        tmp_fdd = torch.tensor( savgol_filter(tmp_f, window_length=25, polyorder=3, deriv=2, delta=mean_dt) ).to(device=device)
        series[key] = [tmp_f, tmp_fd, tmp_fdd]

    v_true = series['localVel'][0]
    q_true = series['localQuat'][0]
    a_true = series[acc_key][0]
    O_true = series['gyroADCafterRpm'][0]
    Od_true = series['gyroADCafterRpm'][1]
    w_true = series['omegaUnfiltered'][0]
    wd_true = series['omegaUnfiltered'][1]
    d_true = series['servo_feedback'][0]
    dd_true = series['servo_feedback'][1]
    ddd_true = series['servo_feedback'][2]

    v_body_true = quaternion_rotate(q_true, v_true, inverse=True)

    if xy_rotation is not None:
        c, s_ = np.cos(xy_rotation), np.sin(xy_rotation)
        a_true = torch.vstack(( c*a_true[0] - s_*a_true[1],
                                s_*a_true[0] + c*a_true[1],
                                a_true[2] ))
        v_body_true = torch.vstack(( c*v_body_true[0] - s_*v_body_true[1],
                                     s_*v_body_true[0] + c*v_body_true[1],
                                     v_body_true[2] ))

    x = torch.vstack([ v_body_true, O_true, Od_true, w_true, wd_true, d_true, dd_true, ddd_true ])
    x = x * SCALE[:, np.newaxis]
    y_true = torch.concat((a_true, Od_true))

    return dict(x=x, y_true=y_true, t=t.to_numpy(), mean_dt=mean_dt, series=series,
                v_body_true=v_body_true, n_samples=len(data))


def weighted_mse_loss(pred, true, weight):
    return torch.sum( (weight * (pred - true)) ** 2)


def fit(x, y_true, n_samples, exclude=DEFAULT_EXCLUDE, epochs=100, lr=5e-1, device="cpu", verbose=True):
    """LBFGS fit as in tailsitterGlobalPhi.py. Returns the fitted GlobalPhiTailsitter."""
    model = GlobalPhiTailsitter()
    model.to(device=device)
    for par in exclude:
        model.get_parameter(par).requires_grad = False

    #                       accelerations   gyro derivatives
    weights = torch.tensor([[0.1, 0.1, 0.1, 0.02, 0.02, 0.02]]).T.to(device=device) / 6 / n_samples**0.5 * 10
    optimizer = optim.LBFGS(model.parameters(), lr=lr)

    for epoch in range(epochs):
        def closure():
            optimizer.zero_grad()
            y_pred = model(x)
            loss = weighted_mse_loss(y_pred, y_true, weights)
            loss.backward()
            return loss

        optimizer.step(closure)
        if verbose:
            print(f'Model Epoch {epoch}: Loss = {closure().item():.5f}')

    return model


def phi_matrix(model):
    PHI = np.zeros((6,6), dtype=np.float64)
    for name, (i, j) in PHI_INDEX.items():
        PHI[i, j] = getattr(model, name).item()
    return PHI + PHI.T - np.diag(np.diagonal(PHI))


def format_scientific(num):
    if abs(num) < 1e-9:
        return "         0"
    elif num >= 0:
        return f"+{num:.3e}"
    else:
        return f"{num:.3e}"


def extract_parameters(model, dz=-0.07):
    """Physical parameters as computed and printed by tailsitterGlobalPhi.py (for TailsitterPhi)."""
    c = {name: getattr(model, name).item() for name in ACTUATION_COEFFS}
    PHI = phi_matrix(model)

    cd = np.array([ c['cxd'], 0, 0, 0, c['cmd'], c['cnd'] ]) * SCALE[9]**2 * SCALE[13]
    cdd = np.array([ c['cxdd'], 0, 0, 0, c['cmdd'], c['cndd'] ]) * SCALE[15]
    cddd = np.array([ 0, 0, 0, 0, c['cmddd'], 0 ]) * SCALE[17]
    d0 = model.d0.detach().cpu().numpy().squeeze()

    k_tot = np.sqrt(c['czww']**2 + c['cxww']**2) * SCALE[9]**2
    axis = np.array([ c['cxww'], 0, c['czww'] ])
    axis /= np.linalg.norm(axis)

    # motor position: choose ny = 0 and dz, solve for dx, dy, cm
    #  Clww / k_tot = +nz dy + cm nx
    #  Cmww / k_tot = -nz dx + nx dz
    #  Cnww / k_tot = -nx dy + cm nz
    nx, nz = axis[0], axis[2]
    A = np.array([[  0, +nz, +nx],
                  [-nz,   0,   0],
                  [  0, -nx, +nz]], dtype=np.float32)
    b = np.array([ c['clww']*SCALE[9]**2 / k_tot,
                   c['cmww']*SCALE[9]**2 / k_tot - nx*dz,
                   c['cnww']*SCALE[9]**2 / k_tot ])
    dx, dy, cmotor = np.linalg.solve(A, b)
    Imotor = np.abs(c['cnwd'] * SCALE[11])

    return dict(m=model.m, I=model.I.numpy().astype(float), phi=model.sqrtphi.item()**2, Phi=PHI,
                cd=cd, cdd=cdd, cddd=cddd, d0=d0, k=k_tot, axis=axis, X_motor=np.array([dx, dy, dz]),
                cmotor=cmotor, Imotor=Imotor, r_imu=model.r.detach().cpu().numpy().flatten())


def print_parameters(p):
    """Paste-ready printout (same format as tailsitterGlobalPhi.py)."""
    print(f"m = {p['m']:.3e}  # mass in kg")
    print(f"I = np.diag([{p['I'][0,0]:.3e}, {p['I'][1,1]:.3e}, {p['I'][2,2]:.3e}])  # inertia matrix in kg m^2")
    print()
    print(f"phi = {p['phi']:.3e}")
    print("Phi = np.array([")
    for row in p['Phi']:
        print("[" + ", ".join(format_scientific(val) for val in row) + "],")
    print("], dtype=np.float32)")
    print()
    for key in ['cd', 'cdd', 'cddd', 'd0']:
        print(f"{key} = np.array([{', '.join(format_scientific(val) for val in p[key])}], dtype=np.float32)")
    print()
    print(f"k = {p['k']:.3e} N/(rad/s)^2")
    X = p['X_motor']
    print(f"X_motor = np.array([{X[0]:.3e}, {X[1]:.3e}, {X[2]:.3e}], dtype=np.float32)")
    a = p['axis']
    print(f"axis = np.array([{a[0]:.3e}, {a[1]:.3e}, {a[2]:.3e}], dtype=np.float32)")
    print(f"cmotor = {p['cmotor']:.3e}  # motor torque coefficient")
    print(f"I_motor = {p['Imotor']:.3e}  # motor rotational inertia")
    print(f"r_IMU = np.array([{', '.join(format_scientific(val) for val in p['r_imu'])}], dtype=np.float32)")


class GlobalPhiActuationModel(ActuationModel):
    """Actuation part of a fitted GlobalPhiTailsitter: forward(x, include_aero=False).

    Keeps the full actuation model of the fit: rotor/elevon terms, IMU offset r, mass, inertia
    and gyroscopic term. Only the phi-theory screw -eta Phi eta_B is removed.
    """

    def __init__(self, model, name="global phi fit", info=None):
        self.name = name
        self.model = model
        self.m = float(model.m)
        self.I = model.I.numpy().astype(float)
        self.d0 = model.d0.detach().cpu().numpy().squeeze().astype(float)
        self.r_imu = model.r.detach().cpu().numpy().flatten().astype(float)
        self.stripped_phi = (model.sqrtphi.item()**2, phi_matrix(model))
        self.parameters = extract_parameters(model)
        self.info = info or {}

    def describe(self):
        return (f"{self.name}\n  m = {self.m:.4g} kg, I = diag({', '.join(f'{v:.3e}' for v in np.diag(self.I))})\n"
                f"  d0 = {self.d0}, r_imu = {self.r_imu}")

    def predict(self, fd):
        # same regressor layout as prepare_data (X_LAYOUT); airspeed is irrelevant without aero
        x = np.hstack((np.zeros((fd.N, 3)), fd.O, fd.Odot, fd.w, fd.wdot, fd.d, fd.dd, fd.ddd)).T
        x = torch.tensor(x * SCALE[:, np.newaxis], dtype=torch.float32)
        with torch.no_grad():
            return self.model(x, include_aero=False).numpy().T.astype(float)


def fit_global_phi(data, name="global phi fit", epochs=100, use_acc_smooth=True, verbose=True):
    """Fit the global phi model to a (cropped) log DataFrame; return (GlobalPhiActuationModel, torch model, prep)."""
    prep = prepare_data(data, use_acc_smooth=use_acc_smooth)
    model = fit(prep['x'], prep['y_true'], prep['n_samples'], epochs=epochs, verbose=verbose)
    return GlobalPhiActuationModel(model, name=name), model, prep
