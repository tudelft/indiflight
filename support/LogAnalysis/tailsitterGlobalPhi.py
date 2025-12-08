#%% 
from indiflight_log_tools import IndiflightLog
from indiflightPlotter import IndiflightPlotter, IndiflightViewport
from pyFlightPlotter import BlittedCursor

from matplotlib import pyplot as plt
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim

from scipy.signal import savgol_filter, butter, sosfilt, sosfilt_zi

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

#%% DEFINE model
#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")  # LBGFS is faster on cpu

class Tailsitter(nn.Module):
    def __init__(self):
        super(Tailsitter, self).__init__()

        self.r = nn.Parameter(torch.zeros(3, 1))  # IMU offset
        self.d0 = nn.Parameter(torch.zeros(2, 1)) # 0-force elevon angle
        self.sqrtphi = nn.Parameter(1*torch.ones(1))
        actuation_coeffs = [
            'cxww',         'cxd', 'cxdd', 

            'czww',         'czd2',
            'clww',
            'cmww',         'cmd',  'cmdd', 'cmddd',
            'cnww', 'cnwd', 'cnd',  'cndd',
        ]
        phi_coeffs = [
            'cxu',
                    'cyv',
            'czu',         'czw',
                    'clv',        'clp',
            'cmu',         'cmw',        'cmq',
                    'cnv',        'cnp',        'cnr',
        ]

        coefficients = actuation_coeffs + phi_coeffs
        for name in coefficients:
            setattr(self, name, nn.Parameter(torch.zeros(1)))

        self.m = 0.5
        Inp = np.diag([6e-3, 2e-3, 6.5e-3]).astype(np.float32)
        #self.m = 1.
        #Inp = np.diag([1., 1., 1.]).astype(np.float32)
        self.I = torch.tensor( Inp )
        self.Iinv = torch.tensor( np.linalg.inv( Inp ) )

    def forward(self, x):
        # STATE
        # body vel    body rate   body rate deriv    prop speeds (derivative)   elevon angles   elevon vel   elevon accelerations
        vx, vy, vz,  Ox, Oy, Oz,   Odx, Ody, Odz,      w1, w2, w1d, w2d,           d1, d2,       d1d, d2d,       d1dd, d2dd =  x

        v  = torch.vstack((vx, vy, vz))
        O  = torch.vstack((Ox, Oy, Oz))
        Od = torch.vstack((Odx, Ody, Odz))

        eta = torch.sqrt(torch.sum(v**2, axis=0) + self.sqrtphi**2 * torch.sum(O**2, axis=0))

        # REGRESSOR primitives
        ww = torch.stack([w1*w1, w2*w2])                              # prop speeds ** 2
        # wwDd = ww * torch.sin( (torch.stack([d1, d2]) ) )          # prop speeds ** 2 * sin ( elevon angles )
        # wwDd = ww * torch.sin( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles - zero-force angle )
        # wwDd = ww * ( (torch.stack([d1, d2]) ) )          # prop speeds ** 2 * sin ( elevon angles )
        wwDd = ww * ( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles - zero-moment angle)
        d2abs = torch.sin( d1 ).abs() + torch.sin( d2 ).abs()         # for reduction of the prop thrust

        # AERO MODEL using phi-theory
        # build lower triangular 6x6 phi-theoretic coefficient matrix without preallocation
        # PHI = torch.tensor([
        #     [self.cxu,     0,          0,          0,              0,            0         ],
        #     [0,            self.cyv,   0,          0,              0,            0         ],
        #     [self.czu,     0,          self.czw,   0,              0,            0         ],
        #     [0,            self.clv,   0,          self.clp,       0,            0         ],
        #     [self.cmu,     0,          self.cmw,   0,              self.cmq,     0         ],
        #     [0,            self.cnv,   0,          self.cnp,       0,            self.cnr  ]
        # ])

        # explicitely writing tensor doesnt work, need to preallocate and assign
        PHI = torch.zeros((6,6))
        PHI[0,0] = self.cxu
        PHI[1,1] = self.cyv
        PHI[2,0] = self.czu
        PHI[2,2] = self.czw

        PHI[3,1] = self.clv
        PHI[3,3] = self.clp
        PHI[4,0] = self.cmu
        PHI[4,2] = self.cmw
        PHI[4,4] = self.cmq
        PHI[5,1] = self.cnv
        PHI[5,3] = self.cnp
        PHI[5,5] = self.cnr

        PHI = PHI + PHI.T - torch.diag(torch.diagonal(PHI))  # make symmetric
        eta_B = torch.vstack([
            vx,
            vy,
            vz,
            Ox,
            Oy,
            Oz
        ])
        screw = - eta * ( PHI @ eta_B )

        # ACTUATION MODEL
        #      prop thrust contribution         prop rate contribution           elevon contribution           elev rate contrib              elev acc contrib
        screw[0] +=  self.cxww * (ww[0] + ww[1])  +  0                        +  self.cxd * (wwDd[0] + wwDd[1])  +  self.cxdd * (d1d + d2d)
        screw[1] +=  0                            +  0                        +  0                               +  0
        screw[2] +=  self.czww * (ww[0] + ww[1])  +  0                        +  self.czd2 * d2abs               +  0
        screw[3] +=  self.clww * (ww[0] - ww[1])  +  0                        +  0                               +  0
        screw[4] +=  self.cmww * (ww[0] + ww[1])  +  0                        +  self.cmd * (wwDd[0] + wwDd[1])  +  self.cmdd * (d1d + d2d)  +   self.cmddd * (d1dd + d2dd)
        screw[5] +=  self.cnww * (ww[0] - ww[1])  +  self.cnwd * (w1d - w2d)  +  self.cnd * (wwDd[0] - wwDd[1])  +  self.cndd * (d1d - d2d)

        f = screw[:3]
        m = screw[3:]

        # KINETICS (todo: use measured dOdt, for IMU offset contribution to fIMU?)
        a_modelled = f / self.m  +  Od.cross(self.r, dim=0)  +  O.cross(O.cross(self.r, dim=0), dim=0)
        dO_modelled = self.Iinv @ ( m  -  O.cross(self.I @ O, dim=0) )

        return torch.concat([a_modelled, dO_modelled], dim=0)

model = Tailsitter()
model.to(device=device)

# ablations: keep parameters at their initial value from __init__
exclude = ['czd2', 'cndd', 'cxdd', 'cmww']
for par in exclude:
    model.get_parameter(par).requires_grad = False


#%% DATA loading

#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00043.BFL", resetTime=True, timeRange=(10000, 50000))  # first dataset
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00063.BFL", resetTime=True, timeRange=(8000, 91000))   # after tuning, higher speeds
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-02-21/LOG00066_simplifiedDynFx.BFL", resetTime=True, timeRange=(9000, 50000))   # dynamic fx, hover
#log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-02-21/LOG00079.BFL", resetTime=True, timeRange=(6587, 50000))   # dynamic fx, hover
log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2025-11-14/LOG00230_fastSysId.BFL", resetTime=False, timeRange=(98000, 175000))   # dynamic fx, hover

# preproc columns that are not in indiflight log tools yet
data = log.data
t = data['timeS']
mean_dt = t.diff().mean()
#data[[f'extAtt[{i}]' for i in range(4)]] /= 8.128
#data[[f'localQuat[{i}]' for i in range(4)]] /= 8128.
#data[[f'localVel[{i}]' for i in range(3)]] /= 100.
#data[[f'servo_feedback[{i}]' for i in range(4)]] *= np.pi/180 / 100
data['servo_feedback[1]'] *= 1 # set to -1 for old datafiles

# convert columns to tensors
series = {'localVel': range(3), 'localQuat': range(4), 'accADCafterRpm': range(3),
          'gyroADCafterRpm': range(3), 'omegaUnfiltered': range(2), 'servo_feedback': range(2),
          'motor': range(2), 'u': range(4)}
# series = {'extVel': range(3), 'extAtt': range(4), 'accADCafterRpm': range(3),
#           'gyroADCafterRpm': range(3), 'omegaUnfiltered': range(2), 'servo_feedback': range(2),
#           'motor': range(4)}
order = 2
freq = 20
butter_sos = butter(order, Wn=freq, btype='lowpass', output='sos', fs=1/mean_dt)
for key, val in series.items():
    tmp = data[[f'{key}[{i}]' for i in val]].to_numpy(dtype=np.float32).T
    #tmp_f   = torch.tensor( savgol_filter(tmp, window_length=25, polyorder=3, deriv=0, delta=mean_dt) ).to(device=device)
    zi = sosfilt_zi(butter_sos)
    zis = np.repeat(zi, len(val), axis=0) * tmp[:, 0, np.newaxis]
    tmp_f = torch.tensor( sosfilt(butter_sos, tmp, zi=zis[np.newaxis])[0], dtype=torch.float32 ).to(device=device)
    tmp_fd  = torch.tensor( savgol_filter(tmp_f, window_length=25, polyorder=3, deriv=1, delta=mean_dt) ).to(device=device)
    tmp_fdd = torch.tensor( savgol_filter(tmp_f, window_length=25, polyorder=3, deriv=2, delta=mean_dt) ).to(device=device)
    series[key] = [tmp_f, tmp_fd, tmp_fdd]

# v_true = series['extVel'][0]
# q_true = series['extAtt'][0]
v_true = series['localVel'][0]
q_true = series['localQuat'][0]
a_true = series['accADCafterRpm'][0]
O_true = series['gyroADCafterRpm'][0]
Od_true = series['gyroADCafterRpm'][1]
w_true = series['omegaUnfiltered'][0]
wd_true = series['omegaUnfiltered'][1]
d_true = series['servo_feedback'][0]
dd_true = series['servo_feedback'][1]
ddd_true = series['servo_feedback'][2]
del_12 = series['motor'][0][:2]

# get velocity in body frame for drag model
v_body_true  = quaternion_rotate(q_true, v_true, inverse=True)

# regressors and targets
scale = np.array([ 1,1,1, 1,1,1, 1,1,1, 1e-3,1e-3, 5e-2,5e-2, 1,1, .1,.1, .01,.01 ], dtype=np.float32)
x = torch.vstack([ v_body_true, O_true, Od_true, w_true, wd_true, d_true, dd_true, ddd_true ])
x = x * scale[:, np.newaxis] # make regressors roughtly unit
y_true = torch.concat((a_true, Od_true))

# loss function           accelerations   gyro derivatives
weights = torch.tensor([[0.1, 0.1, 0.1, 0.02, 0.02, 0.02]]).T.to(device=device) / 6 / len(data)**0.5 * 10
def weighted_mse_loss(pred, true, weight):
    return torch.sum( (weight * (pred - true)) ** 2)

optimizer = optim.LBFGS(model.parameters(), lr=5e-1); epochs = 100
#optimizer = optim.AdamW(model.parameters(), lr=2e-1); epochs = 2000

for epoch in range(epochs):
    def closure():
        optimizer.zero_grad()
        y_pred = model(x)
        loss = weighted_mse_loss(y_pred, y_true, weights)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 1 == 0:
        print(f'Model Epoch {epoch}: Loss = {closure().item():.5f}')


#%% PLOTS
import matplotlib
matplotlib.use('tkagg')

fig, axs = plt.subplots(6, 3)
fig.suptitle("DarkO Phi-model (based on motor speeds, elevon deflection, and speed)")
fig.subplots_adjust(left=0.086, bottom=0.11, right=0.952, top=0.907)

# model
IMU_LABELS = ["a_x\ (N/kg)", "a_y\ (N/kg)", "a_z\ (N/kg)", "\dot\Omega_x\ (rad/s^2)", "\dot\Omega_y\ (rad/s^2)", "\dot\Omega_z\ (rad/s^2)"]
for i in range(6):
    axs[i, 0].plot(t, y_true[i].cpu(), '-')
    axs[i, 0].plot(t, model.forward(x).detach().cpu()[i], '--')
    axs[i, 0].set_ylabel(f"${IMU_LABELS[i]}$")

axs[0, 0].legend(["Raw Measured", "Modelled"])
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

test = 1

# analyse PHI matrix to see if it is positive definite

PHI = np.zeros((6,6), dtype=np.float64)
PHI[0,0] = model.cxu.item()
PHI[1,1] = model.cyv.item()
PHI[2,0] = model.czu.item()
PHI[2,2] = model.czw.item()
PHI[3,1] = model.clv.item()
PHI[3,3] = model.clp.item()
PHI[4,0] = model.cmu.item()
PHI[4,2] = model.cmw.item()
PHI[4,4] = model.cmq.item()
PHI[5,1] = model.cnv.item()
PHI[5,3] = model.cnp.item()
PHI[5,5] = model.cnr.item()

PHI = PHI + PHI.T - np.diag(np.diagonal(PHI))  # make symmetric
eigenvalues = np.linalg.eigvals(PHI)

#%% output the PHI matrix
# rounded to 3 significant digits in scientific format for easy copy-pasting into simulator
# also, keep zeros as 0, and not 0.000e+00
def format_scientific(num):
    if abs(num) < 1e-9:
        return "         0"
    elif num >= 0:
        return f"+{num:.3e}"
    else:
        return f"{num:.3e}"


print(f"phi = {model.sqrtphi.item()**2:.3e}")
print(f"Phi = np.array([")
for row in PHI:
    formatted_row = [format_scientific(val) for val in row]
    print("[" + ", ".join(formatted_row) + "],")
print("], dtype=np.float32)")

cd = np.array([ model.cxd.item(), 0, 0, 0, model.cmd.item(), model.cnd.item() ])
cd *= scale[9]**2 * scale[13]

cdd = np.array([ model.cxdd.item(), 0, 0, 0, model.cmdd.item(), model.cndd.item() ])
cdd *= scale[15]

cddd = np.array([ 0, 0, 0, 0, model.cmddd.item(), 0 ])
cddd *= scale[17]

# output in scientific format with 3 significant digits
print(f"cd = np.array([{', '.join(format_scientific(val) for val in cd)}], dtype=np.float32)")
print(f"cdd = np.array([{', '.join(format_scientific(val) for val in cdd)}], dtype=np.float32)")
print(f"cddd = np.array([{', '.join(format_scientific(val) for val in cddd)}], dtype=np.float32)")
d0 = model.d0.detach().cpu().numpy().squeeze()
print(f"d0 = np.array([{', '.join(format_scientific(val) for val in d0)}], dtype=np.float32)")

# motor max thrust and location
k_tot = np.sqrt(model.czww.item()**2 + model.cxww.item()**2)
k_tot *= scale[9]**2
print(f"k = {k_tot:.3e} N/(rad/s)^2")

axis = np.array([ model.cxww.item(), 0, model.czww.item() ])
axis /= np.linalg.norm(axis)
print(f"axis = np.array([{axis[0]:.3e}, {axis[1]:.3e}, {axis[2]:.3e}], dtype=np.float32)")

# motor position n := axis
# 
#  Clww / k_tot = +nz dy + cm nx
#  Cmww / k_tot = -nz dx + nx dz
#  Cnww / k_tot = -nx dy + cm nz
#
# nx, nz known. choose ny = 0, and dz = -0.07. solve for dx, dy, cm
#
dz = -0.07
nx = axis[0]
nz = axis[2]

A = np.array([[  0, +nz, +nx],
              [-nz,   0,   0],
              [  0, -nx, +nz]], dtype=np.float32)
b = np.array([ model.clww.item()*scale[9]**2 / k_tot,
               model.cmww.item()*scale[9]**2 / k_tot - nx*dz,
               model.cnww.item()*scale[9]**2 / k_tot ])
dx, dy, cmotor = np.linalg.solve(A, b)

# L = model.cmww / k
# dy = model.clww.item()*scale[9]**2 / (k_tot*axis[2]) # meter offset of motor in y
# dx = (-model.cmww.item()*scale[9]**2 / k_tot + axis[0]*dz) / axis[2]
print(f"r_motor = np.array([{dx:.3e}, {dy:.3e}, {dz:.3e}], dtype=np.float32)")

# motor moment coefficient
# cmotor = model.cnww.item() * scale[9]**2 / k_tot
print(f"cmotor = {cmotor:.3e}  # motor torque coefficient")

Imotor = np.abs(model.cnwd.item() * scale[11])
print(f"I_motor = {Imotor:.3e}  # motor rotational inertia")


# imu offset
print(f"r_IMU = np.array([{', '.join(format_scientific(val) for val in model.r.detach().cpu().numpy().flatten())}], dtype=np.float32)")

print()
print("Eigenvalues of PHI matrix:", eigenvalues)


#%% verification with pyIndiflight simulator

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

# add __file__/../simulation/PyNDIflight to path for pyNDIflight import
import os
import sys
sys.path.append( os.path.abspath( os.path.join( os.path.dirname(__file__), '..', 'simulation' ) ) )

from PyNDIflight.crafts import TailsitterPhi
from PyNDIflight.helpers import quatRotate


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
