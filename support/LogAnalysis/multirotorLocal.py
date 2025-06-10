#!/usr/bin/env python3

from indiflight_log_tools import IndiflightLog
from matplotlib import pyplot as plt
import numpy as np

from handy_signal_tools import Signal


log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-05-01/tinyWhoopSysId2.bbl",
                    timeRange=(2000, 18000), resetTime=True)
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
freq_hz = 20
Of = O.filtfilt("lowpass", order, freq_hz)
af = a.filtfilt("lowpass", order, freq_hz)
wf = w.filtfilt("lowpass", order, freq_hz)
df = d.filtfilt("lowpass", order, freq_hz)


#%% effectiveness

# model:
#   \Delta a_i             =  \sum{ c_ij  2\omega_j \Delta \omega_j }_j
#   \Delta \dot{\Omega_i}  =  \sum{ c_ij  2\omega_j \Delta \omega_j }_j

Aa = 2 * wf.y * wf.diff().y
ya = af.diff().y
Xa, Ra, rka, _ = np.linalg.lstsq(Aa, ya)
Xa[:, :2] = 0.
Xa[:, 2] = Xa[:, 2].mean()

AO = np.empty((N, 8))
AO[:, :4] = Aa
AO[:, 4:] = wf.dot().diff().y
yO = Of.dot().diff().y
XO, RO, rkO, _ = np.linalg.lstsq(AO, yO)

#%% motor model

# model:
#   tau * w_dot = ws - w
#   ws = wmax  (k d  +  (1-k) sqrt(d))  +  widle
#   ws = wmax*k * d  +  wmax*(1-k) * sqrt(d)  +  widle
#   ws = a * d  +  b * sqrt(d)  +  widle
#   w  =  ws  -  tau * w_dot
#   w  =  a*d + b*sqrt(d) + widle - tau * w_dot

Am = np.empty((N, 4, 4))
Am[:, 0]  =  df.y
Am[:, 1]  =  np.sqrt(df.y)
Am[:, 2]  =  1.
Am[:, 3]  =  -wf.dot().y

ym = wf.y

wmax = np.empty((4,))
kappa = np.empty((4,))
widle = np.empty((4,))
tau = np.empty((4,))
for i in range(4):
    Xm, _, _, _ = np.linalg.lstsq(Am[:, :, i], ym[:, i])
    a, b, widle[i], tau[i] = Xm
    # wmax*k = a
    # wmax*(1-k) = b
    # a + b = wmax
    # k = a / wmax
    wmax[i] = a+b
    kappa[i] = a / (a+b)

print()
print(f"----- Motor Model found -----")
print()
print(f"set indi_act_time_constant_ms = {','.join([str(int(np.round(x))) for x in 1e3*tau])}")
print(f"set indi_act_max_rpm = {         ','.join([str(int(np.round(x))) for x in wmax/(2*np.pi)*60])}")
print(f"set indi_act_hover_rpm = {       ','.join([str(int(np.round(x))) for x in 0.5*wmax/(2*np.pi)*60])}") # todo: bad approx, shouldnt matter
print(f"set indi_act_nonlinearity = {    ','.join([str(int(np.round(x))) for x in 100.*kappa])}")
print()


#%% scaled fx model

wmax2 = wmax**2
G1 = np.empty((6, 4))
G1[:] = np.nan

G1[:2, :] = 0.
G1[ 2, :] = (Xa[:, 2] @ wmax2) / 4
G1[3:, :] = XO[:4, :].T * wmax2

G2 = XO[4:, :].T
G2[:2, :] = 0.

# convert to indiflight units
G1_int = np.zeros_like(G1, dtype=np.int16)
G1_int[:3, :] = G1[:3, :] * 100
G1_int[3:, :] = G1[3:, :] * 10

G2_int = np.zeros_like(G2, dtype=np.int16)
G2_int[:] = G2 * 1e5

print()
print(f"----- Effectiveness G1/G2 found -----")
print()
print(f"set indi_act_g1_fx = {   ','.join([str(x) for x in G1_int[0, :]])}")
print(f"set indi_act_g1_fy = {   ','.join([str(x) for x in G1_int[1, :]])}")
print(f"set indi_act_g1_fz = {   ','.join([str(x) for x in G1_int[2, :]])}")
print(f"set indi_act_g1_roll = { ','.join([str(x) for x in G1_int[3, :]])}")
print(f"set indi_act_g1_pitch = {','.join([str(x) for x in G1_int[4, :]])}")
print(f"set indi_act_g1_yaw = {  ','.join([str(x) for x in G1_int[5, :]])}")
print()
print(f"set indi_act_g2_roll = { ','.join([str(x) for x in G2_int[0, :]])}")
print(f"set indi_act_g2_pitch = {','.join([str(x) for x in G2_int[1, :]])}")
print(f"set indi_act_g2_yaw = {  ','.join([str(x) for x in G2_int[2, :]])}")
print()


#%% plotting

plt.close('all')
fig, axs = plt.subplots(6, 2, sharex=True)

AXES = ["fx", "fy", "fz", "mx", "my", "mz"]
for i, ax in enumerate(AXES):
    if i < 3:
        axs[i, 0].plot(t, ya[:, i], label=f"{ax} targets")
        axs[i, 0].plot(t, Aa @ Xa[:, i], label=f"{ax} model")
        axs[i, 0].set_ylabel("$\Delta m\ s^{-2}$")
    else:
        axs[i, 0].plot(t, yO[:, i-3], label=f"{ax} targets")
        axs[i, 0].plot(t, AO @ XO[:, i-3], label=f"{ax} model")
        axs[i, 0].set_ylabel("$\Delta rad\ s^{-2}$")

MOTORS = ["1", "2", "3", "4"]
for i, m in enumerate(MOTORS):
    axs[i, 1].plot(t, w_raw[:, i], label=f"motor {m}")
    axs[i, 1].plot(t, wf.y [:, i], label=f"motor {m} filtered")
    axs[i, 1].set_ylabel("$rad\ s^{-1}$")

axs[-1, 0].set_xlabel("Time [s]")
axs[-1, 1].set_xlabel("Time [s]")

for ax in axs.flatten():
    ax.legend()

fig.show()
