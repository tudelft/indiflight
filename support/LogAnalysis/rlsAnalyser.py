from indiflight_log_tools import IndiflightLog
from handy_signal_tools import Signal
from estimators import LMS, RLS, EMWV, RLS_fortescue
from plotting import FlightPlotter, Viewport

import numpy as np
from tqdm import tqdm

log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-05-02/btfl_004_adaptationInflightCrashAfterDisturbance.bbl",
                    logId=3,
                    resetTime=True)
log.data, _ = log.crop(4.643, +np.inf)
# log.data, _ = log.crop(62., 200.)

# log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-10-09_TII_Wednesday/LOG00109_LongOval_140perc.BFL",
#                     logId=1,
#                     resetTime=True)
# log.data, _ = log.crop(18., 26.)


import matplotlib.pyplot as plt
plt.close('all')

fplt = FlightPlotter(log.data, name="Disturbance Flight")
pplt = Viewport(log.data, follow=False, name="Disturbance Flight")
aplt = Viewport(log.data, follow=True, name="Disturbance Flight")
fplt.add_callback('motion_notify_event', pplt.update)
fplt.add_callback('motion_notify_event', aplt.update)

# fplt.show()
# aplt.fig.show()

N = log.data.shape[0]

t_raw = log.data["timeS"]                                    .to_numpy()
O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(4)]].to_numpy()
d_raw = log.data[[f"motor[{i}]" for i in range(4)]]          .to_numpy()

t = np.linspace(t_raw[0], t_raw[-1], N)
O = Signal(t_raw, O_raw, rebase=t)
a = Signal(t_raw, a_raw, rebase=t)
w = Signal(t_raw, w_raw, rebase=t)
d = Signal(t_raw, d_raw, rebase=t)

order = 2
freq_hz = 15
Of = O.filtfilt("lowpass", order, freq_hz)
af = a.filtfilt("lowpass", order, freq_hz)
wf = w.filtfilt("lowpass", order, freq_hz)
df = d.filtfilt("lowpass", order, freq_hz)

Aa = 2 * wf.y * wf.diff().y
Ad = wf.dot().diff().y
yO = Of.dot().diff().y
ya = af.diff().y

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
# 
# rls_fy = RLS(4, 1, gamma=1e-11, forgetting=0.9999)
# rls_fy.setTitle("Fy")
# rls_fy.setParameters([0,0,0,0])

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




emwv = EMWV(forgetting=0.5)
emwv.setParameters([0., 0.])

for Ai, Adi, yOi, yai in tqdm(zip(Aa, Ad, yO, ya), total=len(yO)):
    # rls_fz.newSample(A, yai[2]); rls_fz.update()
    # rls_fort_fz.newSample(A, yai[2]); rls_fort_fz.update()
    # lms_fz.newSample(A, yai[2]); lms_fz.update()

    rls_pitch.newSample(Ai, yOi[1]); rls_pitch.update()
    rls_fort_pitch.newSample(Ai, yOi[1]); rls_fort_pitch.update()
    lms_pitch.newSample(Ai, yOi[1]); lms_pitch.update()

    Ayaw = np.concatenate((Ai, Adi))
    rls_yaw.newSample(Ayaw, yOi[2]); rls_yaw.update()
    rls_fort_yaw.newSample(Ayaw, yOi[2]); rls_fort_yaw.update()
    lms_yaw.newSample(Ayaw, yOi[2]); lms_yaw.update()


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

text = np.zeros(len(t)+1)
text[1:] = t
text[0] = t[0]-(t[1]-t[0])

# all_rls = [rls_fx, rls_fy, rls_fz, rls_roll, rls_pitch, rls_yaw]
# for rls in all_rls:
#     rls.plotParameters(timeMs=text, sharey=True, zoomy=False)

# rls_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)
# rls_fort_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)
# lms_fz.plotParameters(timeMs=text, sharey=True, zoomy=False)

rls_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)
rls_fort_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)
lms_pitch.plotParameters(timeMs=text, sharey=True, zoomy=False)

rls_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)
rls_fort_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)
lms_yaw.plotParameters(timeMs=text, parGroups=[[0,1,2,3], [4,5,6,7]], sharey=False, zoomy=False)

# ferr = emwv.plotParameters(timeMs=text, sharey=False, zoomy=False)

# plt.show()