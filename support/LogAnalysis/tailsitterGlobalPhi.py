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
        self.phi = nn.Parameter(torch.ones(1))
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

        eta = torch.sqrt(torch.sum(v**2, axis=0) + self.phi**2 * torch.sum(O**2, axis=0))

        # REGRESSOR primitives
        ww = torch.stack([w1*w1, w2*w2])                              # prop speeds ** 2
        #wwDd0 = ww * torch.sin( (torch.stack([d1, d2]) ) )          # prop speeds ** 2 * sin ( elevon angles )
        #wwDd = ww * torch.sin( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles - zero-force angle )
        wwDd0 = ww * ( (torch.stack([d1, d2]) ) )          # prop speeds ** 2 * sin ( elevon angles )
        wwDd = ww * ( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles )
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
        screw[0] +=  self.cxww * (ww[0] + ww[1])  +  0                        +  self.cxd * (wwDd0[0] + wwDd0[1])  +  self.cxdd * (d1d + d2d)
        screw[1] +=  0                            +  0                        +  0                               +  0
        screw[2] +=  self.czww * (ww[0] + ww[1])  +  0                        +  self.czd2 * d2abs               +  0
        screw[3] +=  self.clww * (ww[0] - ww[1])  +  0                        +  0                               +  0
        screw[4] +=  self.cmww * (ww[0] + ww[1])  +  0                        +  self.cmd * (wwDd0[0] + wwDd0[1])  +  self.cmdd * (d1d + d2d)  +   self.cmddd * (d1dd + d2dd)
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
exclude = ['czd2', 'cmddd', 'cndd', 'cxdd']
for par in exclude:
    model.get_parameter(par).requires_grad = False


class Motor(nn.Module):
    def __init__(self):
        super(Motor, self).__init__()

        self.tau = nn.Parameter(0.1*torch.ones(1))
        self.idle = nn.Parameter(0*torch.ones(1))
        self.max = nn.Parameter(1000*torch.ones(1))
        self.k = nn.Parameter(0.0*torch.ones(1))

    def forward(self, delta, wd):
        return self.max * (self.k * delta + (1-self.k) * delta**0.5) + self.idle - self.tau * wd


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

#%% MODEL optimisation
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


#%% MOTOR optimization for Motor 1 only for now
# motor = Motor()
# motor.to(device=device)
# 
# # ablations: keep parameters at their initial value from __init__
# motor.get_parameter('idle').requires_grad = False
# 
# # regressors and targets
# x1, x2 = del_12[0], wd_true[0]
# y_motor_true = w_true[0]
# 
# # loss
# def mse_loss(pred, true):
#     return torch.sum((pred - true)**2)
# 
# optimizer = optim.LBFGS(motor.parameters(), lr=2e-1); epochs = 100
# 
# for epoch in range(epochs):
#     def closure():
#         optimizer.zero_grad()
#         y_pred = motor(x1, x2)
#         loss = mse_loss(y_pred, y_motor_true)
#         loss.backward()
#         return loss
# 
#     optimizer.step(closure)
# 
#     if epoch % 10 == 0:
#         print(f'Motor Epoch {epoch}: Loss = {closure().item():.5f}')


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

# d0 for elevon yaw, also 3 significant digits
d0 = model.d0.detach().cpu().numpy().squeeze()
print(f"d0_elevon_yaw = np.array([{', '.join(format_scientific(val) for val in d0)}], dtype=np.float32)")

print(f"self.phi = {model.phi.item():.3e}")
print(f"self.Phi = np.array([")
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
print(f"self.cd = np.array([{', '.join(format_scientific(val) for val in cd)}], dtype=np.float32)")
print(f"self.cdd = np.array([{', '.join(format_scientific(val) for val in cdd)}], dtype=np.float32)")
print(f"self.cddd = np.array([{', '.join(format_scientific(val) for val in cddd)}], dtype=np.float32)")

# motor max thrust and location
k_tot = np.sqrt(model.czww.item()**2 + model.cxww.item()**2)
k_tot *= scale[9]**2
print(f"k = {k_tot:.3e} N/(rad/s)^2")

axis = np.array([ model.cxww.item(), 0, model.czww.item() ])
axis /= np.linalg.norm(axis)
print(f"axis = np.array([{axis[0]:.3e}, {axis[1]:.3e}, {axis[2]:.3e}], dtype=np.float32)")

dz = -0.07
# L = model.cmww / k
dy = model.clww.item()*scale[9]**2 / (k_tot*axis[2]) # meter offset of motor in y
dx = (-model.cmww.item()*scale[9]**2 / k_tot + axis[0]*dz) / axis[2]
print(f"r_motor = np.array([{dx:.3e}, {dy:.3e}, {dz:.3e}], dtype=np.float32)")

# motor moment coefficient
cmotor = model.cnww.item() * scale[9]**2 / k_tot
print(f"cmotor = {cmotor:.3e}  # motor torque coefficient")

Imotor = np.abs(model.cnwd.item() * scale[11])
print(f"I_motor = {Imotor:.3e}  # motor rotational inertia")


# imu offset
print(f"r_IMU = np.array([{', '.join(format_scientific(val) for val in model.r.detach().cpu().numpy().flatten())}], dtype=np.float32)")

# actuation_coeffs = [
#     'cxww',         'cxd', 'cxdd', 
# 
#     'czww',         'czd2',
#     'clww',
#     'cmww',         'cmd',  'cmdd', 'cmddd',
#     'cnww', 'cnwd', 'cnd',  'cndd',
# ]

print()
print("Eigenvalues of PHI matrix:", eigenvalues)



#%% TRIM
# trim condition / initial guesses. False means not optimized
#v_trim   = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body speed
#O_trim   = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body rates
#Od_trim  = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body rate derivatives
#w_trim   = nn.Parameter(torch.ones (2, 1), requires_grad=True)  # motor speeds
#wd_trim  = nn.Parameter(torch.zeros(2, 1), requires_grad=False) # motor rate
#d_trim   = nn.Parameter(torch.zeros(2, 1), requires_grad=True)  # elevon angle
#dd_trim  = nn.Parameter(torch.zeros(2, 1), requires_grad=True)  # elevon angle first derivative
#ddd_trim = nn.Parameter(torch.zeros(2, 1), requires_grad=False) # elevon angle second derivative
#trim_pars = [v_trim, O_trim, Od_trim, w_trim, wd_trim, d_trim, dd_trim, ddd_trim]
#
## dont allow model parameters to change, and set IMU offset to zero for the jacobians to make more sense
#from copy import deepcopy
#model_to_trim = deepcopy(model)
#model_to_trim.requires_grad_(False)
#model_to_trim.r.set_(torch.zeros((3,1)))
#
## define loss as  norm(dOdt)**2  +  (norm(f)**2 - G**2) ** 2
#def trim_loss(output):
#    return torch.sum(output[3:, 0] ** 2)  +  ( torch.sum(output[:3, 0] ** 2) - 9.81**2 ) ** 2
#
## optimize!
#optimizer = optim.LBFGS(trim_pars, lr=1e-1)
#for epoch in range(100):
#    def closure():
#        optimizer.zero_grad()
#        v = model_to_trim(torch.concat(trim_pars))
#        loss = trim_loss(v)
#        loss.backward()
#        return loss
#
#    optimizer.step(closure)
#
#    if epoch % 10 == 0:
#        print(f'Trimming epoch {epoch+1}: Loss = {closure().item():.5f}')
#
#
## %% Hover jacobian
#
## turn on automatic gradient computation for all inputs
##with torch.no_grad():
#    #w_trim.set_(torch.tensor([[1, 1.]]).T)
#    #d_trim.set_(torch.tensor([[0.8, 0.8]]).T)
#wd_trim.requires_grad = True
#dd_trim.requires_grad = True
#ddd_trim.requires_grad = True
#
## calculate 
#y = model_to_trim(torch.concat(trim_pars))
#
## assemble jacobians 
#G1 = torch.zeros((6, 4))
#G2 = torch.zeros((6, 4))
#G3 = torch.zeros((6, 4))
#for i in range(6):
#    output = torch.zeros((6, 1))
#    output[i, 0] = 1.
#
#    G1[i, 0:2] = 1/(1000**2 * 2*w_trim.T) * torch.autograd.grad(y, w_trim, grad_outputs=output, retain_graph=True)[0].T
#    G1[i, 2:4] = torch.autograd.grad(y, d_trim, grad_outputs=output, retain_graph=True)[0].T
#    G2[i, 0:2] = 1/20 * torch.autograd.grad(y, wd_trim, grad_outputs=output, retain_graph=True)[0].T
#    G2[i, 2:4] = 1/10 * torch.autograd.grad(y, dd_trim, grad_outputs=output, retain_graph=True)[0].T
#    G3[i, 2:4] = 1/100 * torch.autograd.grad(y, ddd_trim, grad_outputs=output, retain_graph=True)[0].T
#
##M = np.zeros((6,6), dtype=np.float32)
##M[0,0] = M[1,1] = M[2,2] = model.m
##M[3:, 3:] = model.I
##Minv = torch.tensor( np.linalg.inv(M) )
##G1 = Minv @ G1
##G2 = Minv @ G2
##G3 = Minv @ G3
#
#G1_indi = G1.detach().numpy().copy()
#G2_indi = G2.detach().numpy().copy()
#G3_indi = G3.detach().numpy().copy()
#
## to u-units
#G1_indi[:, :2] *= float(motor.max[0]) ** 2 # omega**2 = omega_max**2 * u
#G1_indi[:, 2:] *= 100*np.pi / 180   # u is in hectodegrees (...)
#G1_indi[:,  3] *= -1                # I have no idea why
#G1_indi[5, :2] *= -1                # I have no idea why
#G2_indi[5, :2] *= -1                # I have no idea why
#G2_indi[5,  2] *= 100*np.pi / 180   # u is in hectodefgrees
#G3_indi[:,  3] *= -1                # I have no idea why
#
## to integers
#G1_indi[:3, :] *= 100
#G1_indi[3:, :] *= 10
#G2_indi        *= 1e5
#G3_indi        *= 1e3
#
#print("\n===== G1 =====")
#print(G1_indi.round().astype(np.int16))
#
#print("\n===== G2 =====")
#print(G2_indi.round().astype(np.int16))
#
#print("\n===== G3 =====")
#print(G3_indi.round().astype(np.int16))
#
#print("\n===== TAILSITTER parameters =====")
#
#d0 = np.array([
#    float(model.d0[0,0]),
#    float(model.d0[1,0]),
#])
##iM = np.array([model.m, model.m, model.m, model.I[0,0], model.I[1,1], model.I[2,2]])
#cv = np.array([
#    float(model.cxv[0]),
#    float(model.cyv[0]),
#    float(model.czv[0]),
#    float(0.),
#    float(model.cmx[0]),
#    float(0.),
#])
#cw = 1e-6 * np.array([
#    float(model.cxw[0]),
#    float(0.),
#    float(model.czw[0]),
#    float(model.clw[0]),
#    float(model.cmw[0]),
#    float(model.cnw[0]),
#])
#cO = 1.   * np.array([
#    float(0.),
#    float(0.),
#    float(0.),
#    float(model.clp[0]),
#    float(model.cmq[0]),
#    float(model.cnr[0]),
#])
#cd = 1e-6 * np.array([
#    float(model.cxd[0]),
#    float(0.),
#    float(0.),
#    float(0.),
#    float(model.cmd[0]),
#    float(model.cnd[0]),
#])
#cdd = 1e-1 * np.array([
#    float(model.cxdd[0]),
#    float(0.),
#    float(0.),
#    float(0.),
#    float(model.cmdd[0]),
#    float(model.cndd[0]),
#])
#cddd = 1e-2 * np.array([
#    float(0.),
#    float(0.),
#    float(0.),
#    float(0.),
#    float(model.cmddd[0]),
#    float(0.),
#])
#
#print(f"d0: {d0}")
#print(f"cv: {cv}")
#print(f"cw: {cw}")
#print(f"cO: {cO}")
#print(f"cd: {cd}")
#print(f"cdd: {cdd}")
#print(f"cddd: {cddd}")
#
#print()
#print(f"set indi_tails_use_scheduled = 1")
#print(f"set indi_tails_use_sine = 1")
#print()
#print(f"set indi_tails_d0 = {int(d0[0]*100*180/np.pi)}, {int(model.d0[1]*100*180/np.pi)}")
#print()
#print(f"set indi_tails_cxw = {int(cw[0]/model.m*1e9)}")
#print(f"set indi_tails_cyw = 0")
#print(f"set indi_tails_czw = {int(cw[2]/model.m*1e9)}")
#print(f"set indi_tails_clw = {int(cw[3]/model.I[0,0]*1e8)}")
#print(f"set indi_tails_cmw = {int(cw[4]/model.I[1,1]*1e8)}")
#print(f"set indi_tails_cnw = {int(cw[5]/model.I[2,2]*1e8)}")
#print()
#print(f"set indi_tails_cnwd = {int(model.cnwd[0]/model.I[2,2]*1e5)}")
#print()
#print(f"set indi_tails_cxd = {int(cd[0]/model.m*1e8)}")
#print(f"set indi_tails_cmd = {int(cd[4]/model.I[1,1]*1e8)}")
#print(f"set indi_tails_cnd = {int(cd[5]/model.I[2,2]*1e8)}")
#print()
#