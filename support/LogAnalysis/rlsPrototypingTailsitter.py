from indiflight_log_tools import IndiflightLog
from handy_signal_tools import Signal
from estimators import LMS, RLS, EMWV, RLS_fortescue
from plotting import FlightPlotter, Viewport, SysIdPlotter, BlittedCursor

import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
plt.close('all')

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Perform offline estimator prototyping on a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

fplt = FlightPlotter(log.data, name=f"{args.name} -- Flight Data")

pplt = Viewport(log.data, follow=False, name=f"{args.name} -- Flight Data")
fplt.connect_viewport(pplt)

# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
# splt.connect_viewport(pplt)

# cursor = BlittedCursor(fplt.all_axes, sharex=True)
# cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)


#%% load data for offline estimator

N = log.data.shape[0]
iend = log.data.index[-1]
n = 2

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
dm_raw = log.data[[f"motor[{i}]" for i in range(n)]]         .to_numpy()
d_raw = log.data[[f'servo_feedback[{i}]' for i in range(2)]].to_numpy()
q_raw = log.data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
dt = np.mean(np.diff(t))

O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
dm = Signal(t_raw, dm_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)
q = Signal(t_raw, q_raw, rebase=t)

order = 2
freq_hz = 15
Of = O.filter("lowpass", order, freq_hz)
af = a.filter("lowpass", order, freq_hz)
wf = w.filter("lowpass", order, freq_hz)
dmf = dm.filter("lowpass", order, freq_hz)
df = d.filter("lowpass", order, freq_hz)
qf = q.filter("lowpass", order, freq_hz)

import quaternion
qfy_norm = quaternion.as_quat_array( qf.y / np.linalg.norm(qf.y, axis=1, keepdims=True))
qfy_norm_inv = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
R_BI = quaternion.as_rotation_matrix(qfy_norm)
R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)

#%% do moments first

Oyz = Of.y[:, 1] * Of.y[:, 2]
Ozx = Of.y[:, 2] * Of.y[:, 0]
Oxy = Of.y[:, 0] * Of.y[:, 1]
OI = np.array([np.diag([Oxy[i], Oyz[i], Ozx[i]]) for i in range(Oyz.shape[0])])

w2 = wf.y*wf.y
wdot = wf.dot().y
w2d = w2 * df.y
ddot = df.dot().y
ddotdot = df.dot().dot().y

w2a = np.repeat(w2[:,0] + w2[:,1], 3).reshape(-1, 3)
w2t = np.array([w2[:,0] - w2[:,1], w2[:,0] + w2[:,1], w2[:,0] - w2[:,1]]).T
wdott = np.array([wdot[:,0] - wdot[:,1], wdot[:,0] + wdot[:,1], wdot[:,0] - wdot[:,1]]).T
w2dt = np.array([w2d[:,0] - w2d[:,1], w2d[:,0] + w2d[:,1], w2d[:,0] - w2d[:,1]]).T
ddott = np.array([ddot[:,0] - ddot[:,1], ddot[:,0] + ddot[:,1], ddot[:,0] - ddot[:,1]]).T
ddotdott = np.array([ddotdot[:,0] - ddotdot[:,1], ddotdot[:,0] + ddotdot[:,1], ddotdot[:,0] - ddotdot[:,1]]).T


rls_m_act = RLS(9, 3, gamma=1e-11, forgetting=0.9999)
rls_m_act.setTitle("RLS Moments -- Actuators Only")
rls_m_act.setParameters([0, 0, 0, 0, 0, 0, 0, 0, 0])
rls_m_act.setCovariance(1e-11 * np.diag([1, 1, 1, # Cw2
                                         1e-6, 1e-6, # Cw2d (y, z only)
                                         1e+2, # Cwdot (z only)
                                         1e+1, 1e+1, # Cddot (y, z only)
                                         1e-1 # Cddotdot (y only)
                                         ]))

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

for w2_ai, w2_ti, w2_d_ti, wdot_ti, ddot_ti, ddotdot_ti, Odoti in tqdm(zip(w2a, w2t, w2dt, wdott, ddott, ddotdott, Of.dot().y), total=len(w2a)):
    # first try: actuayors only, negelct aerodynamics
    # T = Cw2 * w2ti  +  Cw2d * w2dti  +  Cwdot * wdotti  +  Cddot * ddti  +  Cddotdot * dddti
    A = np.zeros((3, 9))
    A[0:3, :3] = np.diag(w2_ti[0:3])
    A[1:3, 3:5] = np.diag(w2_d_ti[1:3])
    A[2:3, 5:6] = np.diag(wdot_ti[2:3])
    A[1:3, 6:8] = np.diag(ddot_ti[1:3])
    A[1:2, 8:] = np.diag(ddotdot_ti[1:2])
    y = Odoti

    rls_m_act.newSample(A, y); rls_m_act.update()

text = np.zeros(len(t)+1)
text[1:] = t
text[0] = t[0]-(t[1]-t[0])

rls_m_act.plotParameters(timeMs=text, parGroups=[[0,1,2], [3,4], [5], [6,7], [8]], sharey=False, zoomy=False)

# asdf = BlittedCursor(rlsIMU_f.all_axes + rlsIMU_f8.all_axes + rlsIMU_m.all_axes, sharex=True)
# cursor = BlittedCursor(fplt.all_axes, sharex=True)
asdf = BlittedCursor(rls_m_act.all_axes + fplt.all_axes, sharex=True)

# ferr = emwv.plotParameters(timeMs=text, sharey=False, zoomy=False)

plt.show()
