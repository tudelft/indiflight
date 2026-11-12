#%% 
from indiflight_log_tools import IndiflightLog
from pyFlightPlotter import BlittedCursor

from matplotlib import pyplot as plt
import numpy as np

import torch

from scipy.signal import butter, sosfiltfilt

# model, preprocessing, fit and parameter extraction live in tailsitter_id.global_phi
from tailsitter_id import global_phi as gp
from tailsitter_id.global_phi import format_scientific

#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")  # LBGFS is faster on cpu


#%% DATA loading

#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00043.BFL", resetTime=True, timeRange=(10000, 50000))  # first dataset
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00063.BFL", resetTime=True, timeRange=(8000, 91000))   # after tuning, higher speeds
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-02-21/LOG00066_simplifiedDynFx.BFL", resetTime=True, timeRange=(9000, 50000))   # dynamic fx, hover
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-02-21/LOG00079.BFL", resetTime=True, timeRange=(6587, 50000))   # dynamic fx, hover
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-11-14/LOG00230_fastSysId.BFL", resetTime=False, timeRange=(98000, 175000))   # dynamic fx, hover
log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2026-10-29/LOG00035_autoId.BFL", resetTime=False, timeRange=(53500, 114000))   # dynamic fx, hover

# test option for the Phi_vv z-rotation: rotate the xy specific force targets and body velocity by a random but reproducible angle
ROTATE_XY_TARGETS = True
RANDOMIZE_ROTATION = True  # True: seeded random angle (skips the simulator verification); False: fixed test angle below
ROTATION_SEED = 0
rot_angle = None
if ROTATE_XY_TARGETS:
    rot_angle = np.random.default_rng(ROTATION_SEED).uniform(-np.pi, np.pi) if RANDOMIZE_ROTATION else np.pi/2
    print(f"Rotated xy force targets and body velocity by {np.degrees(rot_angle):.3f} deg about z (seed {ROTATION_SEED})")

# filtering, derivatives, body velocity, (rotated) targets and scaled regressors x
USE_ACC_SMOOTH = True  # True: use onboard-filtered accSmooth instead of accADCafterRpm
prep = gp.prepare_data(log.data, use_acc_smooth=USE_ACC_SMOOTH,
                       servo1_sign=1, # set to -1 for old datafiles
                       xy_rotation=rot_angle, device=device)
t, mean_dt, series = prep['t'], prep['mean_dt'], prep['series']
x, y_true, v_body_true = prep['x'], prep['y_true'], prep['v_body_true']
O_true = series['gyroADCafterRpm'][0]
w_true = series['omegaUnfiltered'][0]
d_true, dd_true, ddd_true = series['servo_feedback']
scale = gp.SCALE

#%% FIT (LBFGS, 100 epochs, weighted MSE)

# ablations: keep parameters at their initial value
#exclude = ['czd2', 'cndd', 'cxdd', 'cmww']
exclude = ['czd2', 'cndd', 'cmww']
model = gp.fit(x, y_true, prep['n_samples'], exclude=exclude, epochs=100, lr=5e-1, device=device)


#%% PLOTS
import matplotlib
matplotlib.use('QtAgg')

fig, axs = plt.subplots(6, 3)
fig.suptitle("DarkO Phi-model (based on motor speeds, elevon deflection, and speed)")
fig.subplots_adjust(left=0.086, bottom=0.11, right=0.952, top=0.907)

# model
IMU_LABELS = ["a_x\ (N/kg)", "a_y\ (N/kg)", "a_z\ (N/kg)", "\dot\Omega_x\ (rad/s^2)", "\dot\Omega_y\ (rad/s^2)", "\dot\Omega_z\ (rad/s^2)"]
for i in range(6):
    axs[i, 0].plot(t, y_true[i].cpu(), '-')
    axs[i, 0].plot(t, model.forward(x).detach().cpu()[i], '--')
    axs[i, 0].set_ylabel(f"${IMU_LABELS[i]}$")

axs[0, 0].legend(["Measured", "Modelled"])
[ax.set_ylim((-10, 10)) for ax in axs[0:2, 0]]
axs[2, 0].set_ylim((-20, 0))
[ax.set_ylim((-50, +50)) for ax in axs[3:, 0]]

# state and inputs
VEL_LABELS = ["v_{xy}^B\ (m/s)", "v_z^B\ (m/s)"]
for i in range(2):
    axs[0, 1].plot(t, v_body_true[i].cpu().T)
    axs[0, 1].set_ylim((-6, 6))
axs[0, 1].set_ylabel(f"${VEL_LABELS[0]}$")
axs[1, 1].plot(t, v_body_true[2].cpu().T)
axs[1, 1].set_ylabel(f"${VEL_LABELS[1]}$")
axs[1, 1].set_ylim((-4, 4))

for i in range(2):
    axs[2, 1].plot(t, series['u'][0][2+i].T, label=f"u servo")
    axs[2, 1].set_ylabel("u servo")
    axs[3, 1].plot(t, 180/np.pi*d_true[i].cpu().T, label=f"Elevon {i+1}")
    axs[3, 1].set_ylabel("$\delta\ (deg)$")
    axs[4, 1].plot(t, 180/np.pi*dd_true[i].cpu().T, label=f"Elevon {i+1}")
    axs[4, 1].set_ylabel("$\dot\delta\ (deg/s)$")
    axs[5, 1].plot(t, 180/np.pi*ddd_true[i].cpu().T, label=f"Elevon {i+1}")
    axs[5, 1].set_ylabel("$\ddot\delta\ (deg/s^2)$")
[ax.legend() for ax in axs[2:6, 1]]
[ax.legend() for ax in axs[2:6, 1]]

for i in range(3):
    axs[0, 2].plot(t, O_true[i].cpu().T, label=f"Gyro {i+1}")
    axs[0, 2].set_ylabel("$\Omega\ (rad/s)$")
[ax.legend() for ax in axs[2:6, 1]]
[ax.legend() for ax in axs[2:6, 1]]

axs[1, 2].plot(t, 180/np.pi * np.arctan2(v_body_true[0].cpu(), -v_body_true[2].cpu()), label=f"AOA")

axs[2, 2].plot(t, w_true.cpu().T, label=f"Motor {i+1}")
axs[2, 2].set_ylabel("$\omega\ (rad/s)$")


# dress up and plot
[ax.grid(True) for ax in axs.flatten()]
[ax.set_xlabel("Time [s]") for ax in axs[-1, :]]

bc = BlittedCursor(axs.flatten(), sharex=True)
fig.show()

#%% TROUBLESHOOTING PLOT: outputs only, unfiltered / 5Hz zero-phase filtered / filtered error
cutoff = 5
sos_filt = butter(2, Wn=cutoff, btype='lowpass', output='sos', fs=1/mean_dt)
y_meas_np = y_true.cpu().numpy()
y_model_np = model.forward(x).detach().cpu().numpy()
y_meas_filt = sosfiltfilt(sos_filt, y_meas_np, axis=1)
y_model_filt = sosfiltfilt(sos_filt, y_model_np, axis=1)

fig2, axs2 = plt.subplots(6, 3, sharex=True)
fig2.suptitle(f"Phi-model outputs: raw | {cutoff} Hz zero-phase filtered | filtered error (measured - modelled)")
for i in range(6):
    axs2[i, 0].plot(t, y_meas_np[i], '-')
    axs2[i, 0].plot(t, y_model_np[i], '--')
    axs2[i, 0].set_ylabel(f"${IMU_LABELS[i]}$")
    axs2[i, 1].plot(t, y_meas_filt[i], '-')
    axs2[i, 1].plot(t, y_model_filt[i], '--')
    axs2[i, 2].plot(t, y_meas_filt[i] - y_model_filt[i], '-')

axs2[0, 0].legend(["Measured", "Modelled"])
axs2[0, 1].legend([f"Measured ({cutoff}Hz)", f"Modelled ({cutoff}Hz)"])
axs2[0, 2].legend([f"Error ({cutoff}Hz)"])
for col in range(2):
    [ax.set_ylim((-7.5, 7.5)) for ax in axs2[0:2, col]]
    axs2[2, col].set_ylim((-20, 0))
    [ax.set_ylim((-25, +25)) for ax in axs2[3:, col]]
[ax.grid(True) for ax in axs2.flatten()]
[ax.set_xlabel("Time [s]") for ax in axs2[-1, :]]
fig2.subplots_adjust(left=0.086, bottom=0.11, right=0.952, top=0.907)
bc2 = BlittedCursor(axs2.flatten(), sharex=True)
fig2.show()

test = 1


# analyse PHI matrix to see if it is positive definite
PHI = gp.phi_matrix(model)
eigenvalues = np.linalg.eigvals(PHI)

#%% output the identified model, rounded for easy copy-pasting into the simulator
p = gp.extract_parameters(model)
print()
print()
gp.print_parameters(p)
print()
print("Eigenvalues of PHI matrix:", eigenvalues)
print()

np.set_printoptions(linewidth=200, precision=3, suppress=False)
for name, M in {
    "Phi_vv (3x3, symmetric)": PHI[:3, :3],
    "Phi_OO (3x3, symmetric)": PHI[3:, 3:],
    "Phi_vO (3x3 off-diagonal block; Phi_Ov = Phi_vO^T)": PHI[:3, 3:],
}.items():
    print(f"\n{name} =")
    print(M)
print("\nFull Phi (6x6, symmetric) =")
print(np.array2string(PHI, formatter={"float_kind": lambda x: f"{x:+.3e}"}))
print(f"symmetric: {np.allclose(PHI, PHI.T)}")
print()

#%% rotate about z so that the xy coupling Phi_vv[0,1] vanishes
# the xy block of Phi_vv is symmetric 2x2, so its eigenvectors give the principal axes
lam, V = np.linalg.eigh(PHI[:2, :2])
V = V[:, ::-1]                      # order by descending eigenvalue, x-like axis first
if V[0, 0] < 0: V[:, 0] *= -1       # make the first axis point along +x-ish ...
V[:, 1] = np.array([-V[1, 0], V[0, 0]])  # ... and the second 90 deg ccw, so det(R) = +1
psi = np.arctan2(V[1, 0], V[0, 0])  # rotation angle of the new axes w.r.t. the old ones (about z)

Rz = np.eye(3)
Rz[:2, :2] = V
R6 = np.kron(np.eye(2), Rz)         # velocities and body rates rotate identically
PHI_rot = R6.T @ PHI @ R6           # coefficients expressed in the rotated frame

print(f"Rotation about z: psi = {np.degrees(psi):.3f} deg")
if ROTATE_XY_TARGETS:
    # rotation-covariance check: rotating the xy targets and body velocity by rot_angle should rotate
    # the principal axes of Phi_vv by the same angle (a principal axis is a line, so wrap into +-90 deg)
    psi_err = np.degrees(psi - rot_angle)
    print(f"applied rot_angle = {np.degrees(rot_angle):.3f} deg; psi - rot_angle = {psi_err:.3f} deg")
print("Rz =")
print(np.array2string(Rz, formatter={"float_kind": lambda x: f"{x:+.4f}"}))
print("Rotated Phi (6x6, symmetric) =")
print(np.array2string(PHI_rot, formatter={"float_kind": lambda x: f"{x:+.3e}"}))
print(f"rotated Phi_vv[0,1] = {PHI_rot[0,1]:.3e}")
print(f"eigenvalues unchanged: {np.allclose(np.sort(np.linalg.eigvals(PHI_rot).real), np.sort(eigenvalues.real))}")
print()

cd, cdd, cddd, d0 = p['cd'], p['cdd'], p['cddd'], p['d0']
k_tot, axis, cmotor, Imotor = p['k'], p['axis'], p['cmotor'], p['Imotor']
dx, dy, dz = p['X_motor']


#%% verification with pyIndiflight simulator

# the simulator comparison assumes the extracted actuation model (thrust axis, motor position) is
# expressed in the airframe frame, which no longer holds after a randomized xy rotation
if ROTATE_XY_TARGETS and RANDOMIZE_ROTATION:
    print("Skipping pyIndiflight simulator verification: the xy targets and velocity were rotated by a randomized angle.")
else:
    test_cases = [
        {'name': 'off',       'w': [0., 0.]      , 'd': [0., 0.]       , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'hover',     'w': [1500., 1500.], 'd': d0             , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'motors',    'w': [1500., 1500.], 'd': [0., 0.]       , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'motorHigh', 'w': [2500., 2500.], 'd': [0., 0.]       , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'roll',      'w': [1500.,    0.], 'd': [0., 0.]       , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'pureRoll',  'w': [1500.,    0.], 'd': d0             , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'hover',     'w': [1500., 1500.], 'd': d0             , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'pitch',     'w': [1500., 1500.], 'd': d0+[0.1, 0.1]  , 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'yaw',       'w': [1500., 1500.], 'd': d0+[-0.1, +0.1], 'velI': [0., 0., 0.], 'OB': [0., 0., 0.]},
        {'name': 'throw',     'w': [0., 0.]      , 'd': d0             , 'velI': [3., 3., -10.], 'OB': [4., 5., 6.]},
    ]

    from tailsitter_id.actuation import import_pyndiflight
    crafts, helpers = import_pyndiflight()
    TailsitterPhi, quatRotate = crafts.TailsitterPhi, helpers.quatRotate


    model.r.requires_grad = False
    model.r *= 0. # pretend IMU is at CG for simulator comparison
    test_x = torch.zeros((19,1))

    for case in test_cases:
        tail = TailsitterPhi()
        tail.setRotor(0, X=[dx, +dy, dz], ax=axis, k=k_tot, cm=-cmotor, wmax=3000, tau=0.03, kESC=0.5, I=Imotor)
        tail.setRotor(1, X=[dx, -dy, dz], ax=axis, k=k_tot, cm=+cmotor, wmax=3000, tau=0.03, kESC=0.5, I=Imotor)
        tail.setInertia(model.m, np.array(model.I))
        tail.setPhiModel(model.sqrtphi.item()**2, PHI)
        tail.setElevonModel(cd, cdd, cddd, d0)

        tail.r_w[:] = case['w']
        tail.r_wdot[:] = [0., 0.]
        tail.r_tau[:] = np.inf
        tail.s_d[:] = case['d']
        tail.s_dd[:] = [0., 0.]
        tail.s_D *= 0.
        tail.s_P *= 0.
        tail.setPose(np.array([0., 0., -10.]), np.array([1., 0., 0., 0.]))
        tail.setTwist(np.array(case['velI']), np.array(case['OB']))
        tail.vB = quatRotate(tail.qInv, tail.vI)

        tail.tick(1e-9)

        ysim = np.concatenate((tail.fspB, tail.ODotB))

        test_x[0:3] = torch.tensor(case['velI']).unsqueeze(1)
        test_x[3:6] = torch.tensor(case['OB']).unsqueeze(1)
        test_x[9:11] = torch.tensor(case['w']).unsqueeze(1)
        test_x[13:15] = torch.tensor(case['d']).unsqueeze(1)

        ymodel = model.forward( test_x * scale[:, np.newaxis] ).detach().cpu().numpy().squeeze()
        e = ymodel - ysim

        if e.dot(e) < 1e-6:
            print(f"Test case '{case['name']}' passed.")
        else:
            print(f"Test case '{case['name']}' FAILED with error norm {np.linalg.norm(e)}.")
            print(f"w: {case['w']}, d: {case['d']}  =>  fspB sim: {ysim[:3]}, ODotB sim: {ysim[3:6]} => fspB model: {ymodel[:3]}, ODotB model: {ymodel[3:6]}")
