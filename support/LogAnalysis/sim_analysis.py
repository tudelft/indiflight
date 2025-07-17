from indiflight_log_tools import IndiflightLog
from handy_signal_tools import Signal
from estimators import LMS, RLS, EMWV, RLS_fortescue
from plotting import FlightPlotter, Viewport, BlittedCursor

import numpy as np
from tqdm import tqdm

from scipy.spatial.transform import Rotation as R

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, ArgumentTypeError

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("log", type=str, metavar="LOG.bbl", help="Path to the log file to analyze.")
parser.add_argument("--id", required=False, type=int, default=1, metavar="ID",
                    help="Log ID to use for the analysis. Default is 1.")
parser.add_argument("--reset-time", required=False, action="store_true", help="Reset the time in the log to start at 0.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")

args = parser.parse_args()

log = IndiflightLog(args.log, logId=args.id, resetTime=args.reset_time)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

# extract filename from log path
log_name = args.log.split("/")[-1].split(".")[0]

import matplotlib.pyplot as plt
plt.close('all')

fplt = FlightPlotter(log.data, name=log_name)
pplt = Viewport(log.data, follow=False, name=log_name)
aplt = Viewport(log.data, follow=True, name=log_name)
fplt.connect_viewport(pplt)
fplt.connect_viewport(aplt)


#%% extract data from log
N = log.data.shape[0]
t_raw = log.data["timeS"].to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(4)]].to_numpy()
d_raw = log.data[[f"servo_feedback[{i}]" for i in range(2)]].to_numpy()
v_raw = log.data[[f"vel[{i}]" for i in range(3)]].to_numpy()  # velocity data, if available
q_raw = log.data[[f"quat[{i}]" for i in range(4)]].to_numpy()  # velocity data, if available

t = np.linspace(t_raw[0], t_raw[-1], N)
O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)
v = Signal(t_raw, v_raw, rebase=t)
q = Signal(t_raw, q_raw, rebase=t)

order = 2
freq_hz = 15
Of = O.filtfilt("lowpass", order, freq_hz)
Ofd = Of.dot()  # derivative of filtered gyro data
af = a.filtfilt("lowpass", order, freq_hz)
wf = w.filtfilt("lowpass", order, freq_hz)
df = d.filtfilt("lowpass", order, freq_hz)
dfD = df.diff()
vf = v.filtfilt("lowpass", order, freq_hz)
qf = q.filtfilt("lowpass", order, freq_hz)

# rls = RLS(n=8, d=6, gamma=1e2, forgetting=0.9999)
# for i in tqdm(range(N), desc="Fitting RLS model"):
#     rotation = R.from_quat(qf.y[i, [1,2,3,0]])
#     # apply inverse to get body velocity
#     body_velocity = rotation.inv().apply(vf.y[i])
#     eta = np.linalg.norm(body_velocity)
#     if eta < 1:
#         continue  # skip if velocity is too low to avoid numerical issues
# 
#     eta_b = np.concatenate((body_velocity, Of.y[i]))
#     vx, vy, vz, wx, wy, wz = eta_b
#     A = -eta * np.array([
#         [vx,  0,  0, vz,  0,  0,  0,  0],
#         [ 0, vy,  0,  0,  0,  0,  0,  0],
#         [ 0,  0, vz, vx,  0,  0,  0,  0],
#         [ 0,  0,  0,  0, wx,  0,  0, wz],
#         [ 0,  0,  0,  0,  0, wy,  0,  0],
#         [ 0,  0,  0,  0,  0,  0, wz, wx],
#     ])
#     y = np.concatenate((af.y[i], Ofd.y[i]))
#     rls.newSample(A, y); rls.update()

rls = RLS(n=20, d=6, gamma=1e0, forgetting=0.9999)
for i in tqdm(range(N), desc="Fitting RLS model"):
    rotation = R.from_quat(qf.y[i, [1,2,3,0]])
    # apply inverse to get body velocity
    body_velocity = rotation.inv().apply(vf.y[i])
    eta = np.linalg.norm(body_velocity)
    # if eta < 1:
    #     continue  # skip if velocity is too low to avoid numerical issues

    eta_b = np.concatenate((body_velocity, Of.y[i]))
    vx, vy, vz, Ox, Oy, Oz = eta_b
    w1, w2 = wf.y[i, :2] / 1e3
    d1, d2 = df.y[i, :2] / 1e3
    ww1 = w1 * w1
    ww2 = w2 * w2
    ww1d1 = ww1 * d1
    ww1d2 = ww1 * d2
    ww2d1 = ww2 * d1
    ww2d2 = ww2 * d2
    A = -eta * np.array([
        [vx,  0,  0, vz,  0,  0,  0,  0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,      0,     0],
        [ 0, vy,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,      0,     0],
        [ 0,  0, vz, vx,  0,  0,  0,  0, ww1, ww2,   0,   0,     0,     0,     0,     0,     0,     0,      0,     0],
        [ 0,  0,  0,  0, Ox,  0,  0, Oz,   0,   0, ww1, ww2,     0,     0,     0,     0,     0,     0,      0,     0],
        [ 0,  0,  0,  0,  0, Oy,  0,  0,   0,   0,   0,   0, ww1d1, 0*ww1d2, 0*ww2d1, ww2d2,     0,     0,      0,     0],
        [ 0,  0,  0,  0,  0,  0, Oz, Ox,   0,   0,   0,   0,     0,     0,     0,     0, ww1d1, 0*ww1d2, 0*ww2d1, ww2d2],
    ])
    y = np.concatenate((af.y[i], Ofd.y[i]))
    rls.newSample(A, y); rls.update()

#%% plotting

text = np.zeros(len(t)+1)
text[1:] = t
text[0] = t[0]-(t[1]-t[0])

frls = rls.plotParameters(timeMs=text,
                          parGroups=[[0,1,2,3], [4,5,6,7], [8,9], [10,11], [12,13,14,15], [16,17,18,19]],
                          sharey=False,
                          zoomy=False)
frls.show()

cursor = BlittedCursor(fplt.all_axes + rls.all_axes, sharex=True)

