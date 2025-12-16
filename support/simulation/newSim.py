
import numpy as np
from tqdm import tqdm
import threading

from PyNDIflight.crafts import TailsitterPhi, IMU
from PyNDIflight.interfaces import visApp, visData
from PyNDIflight.sim import Sim

import matplotlib.pyplot as plt
from pyFlightPlotter import Quadrotor, Tailsitter, BlittedCursor

from IPython import get_ipython

tail = TailsitterPhi()
tail.setInertia(m=0.5, I=np.diag([6e-3, 2e-3, 6.5e-3]))

phi = 3.370e-01
Phi = np.array([
    [+2.748e-01,          0, +3.666e-02,          0, -1.479e-04,          0],
    [         0, +3.128e-02,          0, +1.425e-03,          0, -4.223e-03],
    [+3.666e-02,          0, +3.950e-02,          0, +1.490e-04,          0],
    [         0, +1.425e-03,          0, +8.233e-04,          0, +7.577e-04],
    [-1.479e-04,          0, +1.490e-04,          0, +8.688e-04,          0],
    [         0, -4.223e-03,          0, +7.577e-04,          0, +3.282e-03],
], dtype=np.float32)
cd   = np.array([-6.241e-07,          0,          0,          0, -2.601e-08, -7.293e-08], dtype=np.float32)
cdd  = np.array([         0,          0,          0,          0, -1.956e-03,          0], dtype=np.float32)
cddd = np.array([         0,          0,          0,          0, -2.961e-05,          0], dtype=np.float32)
d0   = np.array([-3.665e-01, -1.602e-01], dtype=np.float32)

tail.setPhiModel(phi, Phi)
tail.setElevonModel(cd, cdd, cddd, d0)

tail.setRotor(0, X=[1.124e-02, -1.260e-01, -7.000e-02], ax=[1.585e-01, 0.000e+00, -9.874e-01], k=1.254e-06, cm=-1.856e-03, wmax=3000., tau=0.03, kESC=0.5, I=3.336e-6) # RR
tail.setRotor(1, X=[1.124e-02, +1.260e-01, -7.000e-02], ax=[1.585e-01, 0.000e+00, -9.874e-01], k=1.254e-06, cm=+1.856e-03, wmax=3000., tau=0.03, kESC=0.5, I=3.336e-6) # FR

imu = IMU(tail, r=[-3.580e-02, -3.827e-03, +5.630e-03], qBody=[0.707, 0., 0.707, 0.], accStd=0.08, gyroStd=0.08)

tail.setPose(x=[0, -3.5, -0.1], q=[0.707, 0., 0., 0.707])
tail.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

sim = Sim(tail, imu, mocap=None, hil=None, sil=None)

print("\n\n##########################################\n")
print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
visThread.start( )

tail.throw(height=10.,
           wB=[0., -4., 0.],
           vHorz=[0., 3.],
           at_time=0.,
           )

dt = 0.000125 # 8kHz
T = 3.
dt_rt = 1*dt

from support.LogAnalysis.exciteGen import ExcitationGenerator, Transformations

t = np.linspace(0, 1, 1001)
v = lambda t: np.cos(4*np.pi*(1-t)*(1-t))
eg = ExcitationGenerator(t)
eg.add_library_function(v)
eg.add_library_function(Transformations.scale(v, 0.85**1))
eg.add_library_function(Transformations.scale(v, 0.85**2))
eg.add_library_function(Transformations.scale(v, 0.85**3))

eg.add_actuator(type='independent', lb=+0.25, ub=+0.70)
eg.add_actuator(type='independent', lb=+0.25, ub=+0.70)
eg.add_actuator(type='dependent',   lb=-0.60, ub=+0.60, dependent_on=0)
eg.add_actuator(type='dependent',   lb=-0.60, ub=+0.60, dependent_on=1)

eg.generate()


from scipy import interpolate
from indiflight_log_tools.signal_tools import Signal
T_excite = 0.5
T_cooldown = 0.2
t_start_excite = 0.5

dt_id = 0.002
div_id = int(dt_id / dt)
n_id = int(T_excite / dt_id) - 1

t_raw  = np.zeros((n_id))
O_raw  = np.zeros((n_id, 3))
w_raw  = np.zeros((n_id, 2))
d_raw  = np.zeros((n_id, 2))
u_raw  = np.zeros((n_id, 4))
dm_raw = np.zeros((n_id, 2))
q_raw  = np.zeros((n_id, 4))
v_raw  = np.zeros((n_id, 3))

tau_excite = np.arange(0., T_excite, dt)
Uint = interpolate.interp1d(eg.t, eg.U, axis=1)(tau_excite / T_excite)
UintS = Signal(tau_excite, Uint.T)

Ufmotor = UintS.filter(type='lowpass', order=1, cutoff_hz=1/(0.03*2*np.pi)).y
Ufservo = UintS.filter(type='lowpass', order=2, cutoff_hz=1/(0.04*2*np.pi)).y



j = 0
e = 0
for i in tqdm(range(int(T / dt)), target_looptime=dt_rt):
    # interpolate eg.U at current sim time, and then set r_u, s_u
    u = np.zeros((4,))
    t_excitation = sim.t - t_start_excite
    if t_excitation >= 0. and t_excitation <= T_excite:
        u[:] = interpolate.interp1d(eg.t, eg.U, axis=1)(t_excitation / T_excite)

        # uservo = interpolate.interp1d(eg.t, eg.U, axis=1)((t_excitation + 0.04) / T_excite)
        # u[2:] = uservo[2:]

        u[:2] = Ufservo[e, :2]
        u[2:] = Ufmotor[e, 2:]
        e += 1


        if i % div_id == 0: # 500Hz 
            t_raw[j] = sim.t
            O_raw[j] = tail.OB
            w_raw[j] = tail.r_w
            d_raw[j] = tail.s_d
            q_raw[j] = tail.q
            v_raw[j] = tail.vB
            j += 1
    if t_excitation > T_excite+T_cooldown:
        break

    tail.r_u[:] = u[:2]
    tail.s_u[:] = u[2:]
    tail.s_u[1] = tail.s_u[1]  # right elevon is negative deflection
    sim.tick(dt)

visThread.join(0.1)
del(visApp)


boolarr = t_raw > 0.
t = t_raw[boolarr].copy()
O = Signal(t, O_raw[boolarr])
w = Signal(t, w_raw[boolarr])
d = Signal(t, d_raw[boolarr])
q = Signal(t, q_raw[boolarr])
v = Signal(t, v_raw[boolarr])

order = 2
freq_hz = 15.

Of = O.filter(type='lowpass', order=order, cutoff_hz=freq_hz)
wf = w.filter(type='lowpass', order=order, cutoff_hz=freq_hz)
df = d.filter(type='lowpass', order=order, cutoff_hz=freq_hz)
qf = q.filter(type='lowpass', order=order, cutoff_hz=freq_hz)
vf = v.filter(type='lowpass', order=order, cutoff_hz=freq_hz)

import quaternion
qfy_norm = quaternion.as_quat_array( qf.y / np.linalg.norm(qf.y, axis=1, keepdims=True))
qfy_norm_inv = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
R_BI = quaternion.as_rotation_matrix(qfy_norm)
R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)
v_B = (R_IB @ vf.y[:, :, np.newaxis]).squeeze()

v_norm = np.linalg.norm(v_B, axis=1)
O_norm = np.linalg.norm(Of.y, axis=1)

eta_B = np.hstack((v_B, Of.y))
phi = 1.0
eta = np.sqrt(0*v_norm**2 + phi * O_norm**2)


from support.LogAnalysis.estimators import RLS, LS

estimators = []

rls_all = RLS(26, 3, gamma=1e-11, forgetting=0.9999)
rls_all.name = "rls_all"
rls_all.setCovariance(1e-12*np.diag([1, 1, 1e8,  1, 1, 1e8,    1, 1, 1e8, 1e7,  1, 1, 1e8, 1e7,    1, 1, 1e8,  1, 1, 1e8,    1e9, 1e9, 1e9, 1e8, 1e8, 1e8]))
estimators.append(rls_all)

ls_all = LS(26, 3)
ls_all.name = "ls_all"
estimators.append(ls_all)

A_hist = []
count = 0
for i, ti in enumerate(t):
    w2i = wf.y[i]**2
    w2di = w2i * df.y[i]
    wdi = wf.dot(order=1).y[i]
    di = df.y[i]
    ddi = df.dot(order=1).y[i]
    dddi = df.dot(order=2).y[i]
    vi = v_B[i]
    Oi = Of.y[i]
    Odi = Of.dot(order=1).y[i]
    etai = eta[i]
    etaBi = eta_B[i]

    y = Odi

    A = np.zeros((3, 26))
    A[0, 0:3]   = [w2i[0], w2i[0] * di[0], wdi[0]]
    A[0, 3:6]   = [w2i[1], w2i[1] * di[1], wdi[1]]
    A[1, 6:10]  = [w2i[0], w2i[0] * di[0], ddi[0], dddi[0]]
    A[1, 10:14] = [w2i[1], w2i[1] * di[1], ddi[1], dddi[1]]
    A[2, 14:17] = [w2i[0], w2i[0] * di[0], wdi[0]]
    A[2, 17:20] = [w2i[1], w2i[1] * di[1], wdi[1]]
    A[:, 20:23] = - np.array([Oi[1]*Oi[2], Oi[0]*Oi[2], Oi[0]*Oi[1]])
    A[:, 23:26] = - etai * np.diag(etaBi[3:]) # C_m_w
    A_hist.append(A)

    rls_all.newSample(A, Odi, ti)
    rls_all.update()

    ls_all.newSample(A, Odi, ti)

    count += 1
    if count > 150 and count % 10 == 0:
        ls_all.update()


parGroups = [[0,3], [1,4], [2,5],   [6,10], [7,11], [8,12], [9,13],   [14,17], [15,18], [16,19], [20,21,22], [23,24,25]]
parGroupNames = ["$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                 "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\delta}, q}$", "$C_{\\ddot{\\delta}, q}$",
                 "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                 "$C_{m\\sigma}$",
                 "$C_{m\\omega diag}$"]


all_axes = []
for est in estimators:
    est.plotParameters(parGroups=parGroups, parGroupNames=parGroupNames,
                       sharey=False, zoomy=False)

    # est.f.savefig(f"tailsitter_phi_estimator_{est.name}.png", dpi=300)
    # est.f.show()
    all_axes.extend(est.all_axes)


bc = BlittedCursor(all_axes, sharex=True)

ipython = get_ipython()
if ipython is not None:
    ipython.run_line_magic('matplotlib', 'qt')
    plt.show()
