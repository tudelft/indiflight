"""Log loading and signal preprocessing shared by the tailsitter identification tools."""

from dataclasses import dataclass, field

import numpy as np

from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal


def load_log(path, log_id=1, reset_time=False, crop=None, auto_range=False, auto_range_margin=0.0):
    """Load a log and optionally crop it.

    crop:        (start, end) in seconds (log time, i.e. after reset_time is applied)
    auto_range:  crop to the span in which any motor is commanded (see find_motor_active_range)

    Returns (log, data), where data is the cropped DataFrame. log.data is left uncropped.
    """
    log = IndiflightLog(path, logId=log_id, resetTime=reset_time)
    data = log.data
    if crop is not None:
        data, _ = log.crop(crop[0], crop[1])
    if auto_range:
        rng = find_motor_active_range(data, margin=auto_range_margin)
        if rng is None:
            print(f"auto range: no motor activity found in {path}, using full range")
        else:
            print(f"auto range: motors active from {rng[0]:.2f} s to {rng[1]:.2f} s")
            t = data['timeS']
            data = data[(t >= rng[0]) & (t <= rng[1])]
    if len(data) < 10:
        raise ValueError(f"Too few samples ({len(data)}) left in {path} after cropping")
    return log, data


def find_motor_active_range(data, margin=0.0, n_motors=2):
    """Time span [first, last] in which any of the first n_motors motor[i] commands is > 0.

    Same activity criterion as analyseOnboardId.py (which only crops the start, at first - 2 s).
    margin is added on both sides. Returns None if motors were never active.
    """
    cols = [f'motor[{i}]' for i in range(n_motors) if f'motor[{i}]' in data.columns]
    if not cols:
        return None
    active = (data[cols] > 0).any(axis=1).to_numpy()
    if not active.any():
        return None
    t = data['timeS'].to_numpy()
    return t[active][0] - margin, t[active][-1] + margin


def quat_rotate(q, v):
    """Rotate vectors v (N,3) by unit quaternions q (N,4, scalar first): v' = q v q*.

    Vectorized version of PyNDIflight.helpers.quatRotate.
    """
    qv = q[:, 1:]
    tmp = 2. * np.cross(qv, v)
    return v + q[:, :1] * tmp + np.cross(qv, tmp)


def quat_conj(q):
    return q * np.array([1., -1., -1., -1.])


@dataclass
class FlightData:
    """Uniformly resampled and filtered flight signals.

    All arrays have N rows. Units: SI (rad, rad/s, m/s, N/kg).
    acc is the accelerometer specific force at the IMU location, body frame.
    q is the attitude quaternion (scalar first) rotating body to inertial vectors.
    """
    name: str
    t: np.ndarray
    O: np.ndarray       # body rates (N,3)
    Odot: np.ndarray    # body rate derivatives (N,3)
    acc: np.ndarray     # specific force at IMU (N,3)
    w: np.ndarray       # rotor speeds (N,2)
    wdot: np.ndarray    # (N,2)
    d: np.ndarray       # elevon angles (N,2)
    dd: np.ndarray      # (N,2)
    ddd: np.ndarray     # (N,2)
    q: np.ndarray       # attitude quaternion (N,4)
    vI: np.ndarray      # inertial velocity (N,3)
    info: dict = field(default_factory=dict)

    @property
    def N(self):
        return len(self.t)

    def vB(self, wind=None):
        """Body-frame airspeed, assuming the inertial wind vector `wind` (3,) (default: no wind)."""
        v = self.vI if wind is None else self.vI - np.asarray(wind, dtype=float)[np.newaxis]
        return quat_rotate(quat_conj(self.q), v)

    def y_obs(self):
        """Observed outputs [acc (3), Odot (3)] as (N,6)."""
        return np.hstack((self.acc, self.Odot))


def prepare_flight_data(data, name="", filter_hz=5., order=2, zero_phase=False,
                        vel_source='vel', acc_source='accADCafterRpm', quat_source='quat'):
    """Resample a log DataFrame to a uniform grid, low-pass filter and differentiate.

    Defaults match the original lsPrototypingTailsitterFF.py: 2nd order causal Butterworth at
    5 Hz on everything, derivatives by finite differences of the filtered signals.
    zero_phase uses filtfilt instead (no group delay).
    """
    t_raw = data['timeS'].to_numpy()
    N = len(t_raw)
    t = np.linspace(t_raw[0], t_raw[-1], N)

    def col(prefix, n):
        names = [f'{prefix}[{i}]' for i in range(n)]
        missing = [c for c in names if c not in data.columns]
        if missing:
            raise KeyError(f"Log has no columns {missing}")
        return data[names].to_numpy(dtype=float)

    def filt(raw):
        s = Signal(t_raw, raw, rebase=t)
        return s.filtfilt('lowpass', order, filter_hz) if zero_phase else s.filter('lowpass', order, filter_hz)

    O = filt(col('gyroADCafterRpm', 3))
    a = filt(col(acc_source, 3))
    w = filt(col('omegaUnfiltered', 2))
    d = filt(col('servo_feedback', 2))
    q = filt(col(quat_source, 4))
    v = filt(col(vel_source, 3))

    qy = q.y / np.linalg.norm(q.y, axis=1, keepdims=True)

    return FlightData(
        name=name, t=t,
        O=O.y, Odot=O.dot().y, acc=a.y,
        w=w.y, wdot=w.dot().y,
        d=d.y, dd=d.dot().y, ddd=d.dot().dot().y,
        q=qy, vI=v.y,
        info=dict(filter_hz=filter_hz, order=order, zero_phase=zero_phase,
                  vel_source=vel_source, acc_source=acc_source, quat_source=quat_source),
    )
