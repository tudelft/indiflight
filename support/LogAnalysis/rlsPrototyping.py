from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import LMS, RLS, EWMV, RLS_fortescue
from pyFlightPlotter import BlittedCursor
from indiflightPlotter import IndiflightPlotter, IndiflightSysIdPlotter, IndiflightViewport

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

# log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-05-02/btfl_004_adaptationInflightCrashAfterDisturbance.bbl",
#                     logId=3,
#                     resetTime=True)
# log.data, _ = log.crop(4.643, +np.inf)
# log.data, _ = log.crop(62., 200.)

# log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-10-09_TII_Wednesday/LOG00109_LongOval_140perc.BFL",
#                     logId=1,
#                     resetTime=True)
# log.data, _ = log.crop(18., 26.)


# fplt = FlightPlotter(log.data, name=f"{args.name} -- Flight Data")
# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
# pplt = Viewport(log.data, follow=False, name=f"{args.name} -- Onboard ID Analysis")
# 
# fplt.connect_viewport(pplt)
# splt.connect_viewport(pplt)

# cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)


#%% load data for offline estimator

N = log.data.shape[0]
iend = log.data.index[-1]
n = 4

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(4)]].to_numpy()
d_raw = log.data[[f"motor[{i}]" for i in range(4)]]          .to_numpy()
q_raw = log.data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
dt = np.mean(np.diff(t))

O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)
q = Signal(t_raw, q_raw, rebase=t)

order = 2
freq_hz = 15
Of = O.filtfilt("lowpass", order, freq_hz)
af = a.filtfilt("lowpass", order, freq_hz)
wf = w.filtfilt("lowpass", order, freq_hz)
df = d.filtfilt("lowpass", order, freq_hz)
qf = q.filtfilt("lowpass", order, freq_hz)

import quaternion
qfy_inv_norm = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
R_I_to_B = quaternion.as_rotation_matrix(qfy_inv_norm)

Aa = 2 * wf.y * wf.diff().y
Ad = wf.dot().diff().y
Odiff = Of.diff()
yO = Of.dot().diff().y
ya = af.diff().y

absAa = wf.y * wf.y
absAd = wf.dot().y
absyO = Of.dot().y
absya = af.y

# Aa = 2 * wf.y * wf.dot().y
# yO = Of.dot(order=2).y
# Aa = wf.y**2
# yO = Of.dot().y

wmax = (80000/60*2*np.pi)
thrustfx = 1e-2*1000  / wmax**2
rollfx = 1e-1*6000. / wmax**2
pitchfx = 1e-1*6000. / wmax**2
yawfx = 1e-1*500. / wmax**2

# rls_fx = RLS(4, 1, gamma=1e-11, forgetting=0.9999)
# rls_fx.setTitle("Fx")
# rls_fx.setParameters([0,0,0,0])

rls_fy = RLS(4, 1, gamma=1e-11, forgetting=0.9999)
rls_fy.setTitle("Fy")
rls_fy.setParameters([0,0,0,0])

rls8_fy = RLS(8, 1, gamma=1e-11, forgetting=0.9999)
rls8_fy.setTitle("Fy 8 reg")
rls8_fy.setParameters([0,0,0,0,0,0,0,0])

rlsIMU_f_diff = RLS_fortescue(3+3*4, 3, gamma=1e-11, forgetting_base=0.995, N0=500.)
rlsIMU_f_diff.setTitle("IMU F but differential")
rlsIMU_f_diff.setParameters([0, 0, 0,
                             0, 0, 0, 0,
                             0, 0, 0, 0,
                             0, 0, 0, 0])
rlsIMU_f_diff.setCovariance(np.diag([1e-3, 1e-3, 1e-3,
                                     1e-12, 1e-12, 1e-12, 1e-12,
                                     1e-12, 1e-12, 1e-12, 1e-12,
                                     1e-12, 1e-12, 1e-12, 1e-12]))

rlsIMU_fv = RLS_fortescue(3+3*4+1, 3, gamma=1e-11, forgetting_base=0.995, N0=500.)
rlsIMU_fv.setTitle("IMU F with v")
rlsIMU_fv.setParameters([0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0])
rlsIMU_fv.setCovariance(np.diag([1e-3, 1e-3, 1e-3,
                                1e-12, 1e-12, 1e-12, 1e-12,
                                1e-12, 1e-12, 1e-12, 1e-12,
                                1e-12, 1e-12, 1e-12, 1e-12,
                                1e-1]))

rlsIMU_f = RLS_fortescue(3+3*4, 3, gamma=1e-11, forgetting_base=0.995, N0=500.)
rlsIMU_f.setTitle("IMU F")
rlsIMU_f.setParameters([0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0])
rlsIMU_f.setCovariance(np.diag([1e-3, 1e-3, 1e-3,
                                1e-12, 1e-12, 1e-12, 1e-12,
                                1e-12, 1e-12, 1e-12, 1e-12,
                                1e-12, 1e-12, 1e-12, 1e-12]))

rlsIMU_f8 = RLS_fortescue(3+3*8, 3, gamma=1e-11, forgetting_base=0.995, N0=500.)
rlsIMU_f8.setTitle("IMU F8")
rlsIMU_f8.setParameters([0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0])
rlsIMU_f8.setCovariance(np.diag([1e-3, 1e-3, 1e-3,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8]))


rlsIMU_m = RLS_fortescue(3+3*8, 3, gamma=1e-11, forgetting_base=0.995, N0=500.)
rlsIMU_m.setTitle("IMU M")
rlsIMU_m.setParameters([0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0])
rlsIMU_m.setCovariance(np.diag([1e-3, 1e-3, 1e-3,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8,
                                1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-8, 1e-8, 1e-8]))


# rls_fz = RLS(4, 1, gamma=1e-11, forgetting=0.999)
# rls_fz.setTitle("RLS Fz")
# rls_fz.setParameters([-thrustfx, -thrustfx, -thrustfx, -thrustfx])
# 
# rls_fort_fz = RLS_fortescue(4, 1, gamma=1e-11, forgetting_base=0.999, N0=1.)
# rls_fort_fz.setTitle("RLS fortescue Fz")
# rls_fort_fz.setParameters([-thrustfx, -thrustfx, -thrustfx, -thrustfx])
# 
# lms_fz = LMS(4, 1, mu=1e-13)
# lms_fz.setTitle("LMS Fz")
# lms_fz.setParameters([-thrustfx, -thrustfx, -thrustfx, -thrustfx])

# rls_roll = RLS(4, 1, gamma=1e-11, forgetting=0.9999)
# rls_roll.setTitle("Roll")
# rls_roll.setParameters([-rollfx, -rollfx, -rollfx, -rollfx])

rls_pitch = RLS(4, 1, gamma=1e-14, forgetting=0.992)
rls_pitch.setTitle("RLS Pitch")
rls_pitch.setParameters([-pitchfx, pitchfx, -pitchfx, pitchfx])

rls_fort_pitch = RLS_fortescue(4, 1, gamma=1e-14, forgetting_base=0.995, N0=500.)
rls_fort_pitch.setTitle("RLS Fortescue Pitch")
rls_fort_pitch.setParameters([-pitchfx, pitchfx, -pitchfx, pitchfx])

lms_pitch = LMS(4, 1, mu=1e-13)
lms_pitch.setTitle("LMS Pitch")
lms_pitch.setParameters([-pitchfx, pitchfx, -pitchfx, pitchfx])

# rls_yaw = RLS(4, 1, gamma=1e-11, forgetting=0.9999)
# rls_yaw.setTitle("Yaw")
# rls_yaw.setParameters([yawfx, -yawfx, yawfx, -yawfx])

rls_yaw = RLS(8, 1, gamma=1e-10, forgetting=0.992)
rls_yaw.setTitle("RLS Yaw")
# rls_yaw.setParameters([yawfx, -yawfx, yawfx, -yawfx])

rls_fort_yaw = RLS_fortescue(8, 1, gamma=1e-10, forgetting_base=0.995, N0=500.)
rls_fort_yaw.setTitle("RLS Fortescue Yaw")
# rls_fort_yaw.setParameters([yawfx, -yawfx, yawfx, -yawfx])

lms_yaw = LMS(8, 1, mu=1e-13)
lms_yaw.setTitle("LMS Yaw")
# lms_yaw.setParameters([yawfx, -yawfx, yawfx, -yawfx])

ewmv = EWMV(forgetting=0.5)
ewmv.setParameters([0., 0.])

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

for Aai, Adi, yOi, yai, Oi, Odiffi, ti in tqdm(zip(Aa, Ad, yO, ya, Of.y, Odiff.y, t), total=len(yO)):
    Ad = np.concatenate((Aai, Adi))
    rls_fy.newSample(Aai, yai[1], ti); rls_fy.update()
    rls8_fy.newSample(Ad, yai[1], ti); rls8_fy.update()

    AIMU_diff = np.empty((3, 3+3*4))
    AIMU_diff[:, :3] = skew(Odiffi) @ skew(Oi) + skew(Oi) @ skew(Odiffi)
    AIMU_diff[:, 3:15] = np.kron(np.eye(3), Aai)
    rlsIMU_f_diff.newSample(AIMU_diff, yai, ti); rlsIMU_f_diff.update()

    # rls_fz.newSample(A, yai[2]); rls_fz.update()
    # rls_fort_fz.newSample(A, yai[2]); rls_fort_fz.update()
    # lms_fz.newSample(A, yai[2]); lms_fz.update()

    rls_pitch.newSample(Aai, yOi[1], ti); rls_pitch.update()
    rls_fort_pitch.newSample(Aai, yOi[1], ti); rls_fort_pitch.update()
    lms_pitch.newSample(Aai, yOi[1], ti); lms_pitch.update()

    rls_yaw.newSample(Ad, yOi[2], ti); rls_yaw.update()
    rls_fort_yaw.newSample(Ad, yOi[2], ti); rls_fort_yaw.update()
    lms_yaw.newSample(Ad, yOi[2], ti); lms_yaw.update()

# repeat for absolute regression
delta_v = 0
for Aai, Adi, yOi, yai, Oi, Ri, ti in tqdm(zip(absAa, absAd, absyO, absya, Of.y, R_I_to_B, t), total=len(absyO)):
    # calculate vertical speed increment since throw
    delta_v += dt * np.linalg.inv(Ri) @ yai
    # vzi = 

    a_app_mtx = skew(Oi) @ skew(Oi) + skew(yOi)
    AIMU = np.empty((3, 3+3*4))
    AIMU[:, :3] = a_app_mtx
    AIMU[:, 3:15] = np.kron(np.eye(3), Aai)

    AIMUv = np.empty((3, 3+3*4+1))
    AIMUv[:, :15] = AIMU[:, :15]
    AIMUv[:, 15] = Ri[:, 2]

    AIMUv2 = np.empty((3, 3+3*4+2))
    AIMUv2[:, :15] = AIMU[:, :15]
    AIMUv2[:, 15] = Ri[:, 2]
    AIMUv2[:, 16] = Ri[:, 2]

    Ad = np.concatenate((Aai, Adi))
    AIMU8 = np.empty((3, 3+3*8))
    AIMU8[:, :3] = a_app_mtx
    AIMU8[:, 3:] = np.kron(np.eye(3), Ad)

    rlsIMU_f.newSample(AIMU, yai, ti); rlsIMU_f.update()
    rlsIMU_fv.newSample(AIMUv, yai, ti); rlsIMU_fv.update()
    rlsIMU_f8.newSample(AIMU8, yai, ti); rlsIMU_f8.update()

    AIMU_m = np.empty((3, 3+3*8))
    Ireg_mtx = -np.diag([Oi[1]*Oi[2], Oi[0]*Oi[2], Oi[0]*Oi[1]])
    AIMU_m[:, :3] = Ireg_mtx
    AIMU_m[:, 3:] = np.kron(np.eye(3), Ad)

    rlsIMU_m.newSample(AIMU_m, yOi, ti); rlsIMU_m.update()


# e = np.array(rls.predictOnline()).squeeze()[1:] - yO[:, 0]
# e_sig = Signal(t, e)
# eHP = e_sig.filter('highpass', 1, 5).y

# for out in tqdm(yO[:, 1].squeeze()):
#     emwv.newSample([0, 0], out)
#     emwv.update()

# fig, ax = plt.subplots(1,1)
# ax.plot(t, e)
# ax.plot(t, eHP.squeeze())
# ax.legend(["pure", "highpass"])
# fig.show()

# all_rls = [rls_fx, rls_fy, rls_fz, rls_roll, rls_pitch, rls_yaw]
# for rls in all_rls:
#     rls.plotParameters(timeMs=text, sharey=True, zoomy=False)

# rls_fy.plotParameters(timeMs=text, parGroups=[[0,1,2,3]], sharey=False, zoomy=False)
# rls8_fy.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)

# rls_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)
# rls_fort_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)
# lms_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)

# rls_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)
# rls_fort_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)
# lms_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)

# rls_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)
# rls_fort_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)
# lms_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)

rlsIMU_f.plotParameters(parGroups=[[0,1,2], [3,4,5,6], [7,8,9,10], [11,12,13,14]], sharey=False, zoomy=False)
rlsIMU_f_diff.plotParameters(parGroups=[[0,1,2], [3,4,5,6], [7,8,9,10], [11,12,13,14]], sharey=False, zoomy=False)
rlsIMU_fv.plotParameters(parGroups=[[0,1,2], [3,4,5,6], [7,8,9,10], [11,12,13,14], [15]], sharey=False, zoomy=False)
rlsIMU_f8.plotParameters(parGroups=[[0,1,2], range(3, 7), range(7, 11), range(11, 15), range(15, 19), range(19, 23), range(23, 27)], sharey=False, zoomy=False)
rlsIMU_m.plotParameters(parGroups=[[0,1,2], range(3, 7), range(7, 11), range(11, 15), range(15, 19), range(19, 23), range(23, 27)], sharey=False, zoomy=False)

# asdf = BlittedCursor(rlsIMU_f.all_axes + rlsIMU_f8.all_axes + rlsIMU_m.all_axes, sharex=True)
asdf = BlittedCursor(rlsIMU_f.all_axes, sharex=True)

# ferr = emwv.plotParameters(timeMs=text, sharey=False, zoomy=False)

plt.show()

# %%
