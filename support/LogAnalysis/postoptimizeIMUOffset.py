import numpy as np
from scipy.optimize import least_squares

n = 6

#  N/kg / 1 * 100.  Roll, pitch is Nm / (kgm^2) / 1 * 10.  Yaw is Nm / (kgm^2) / 1 * 10
B1_fromThrow = np.array(
    [
        [2,-77,70,254,149,-112],
        [-130,154,104,6,-259,-98],
        [-1125,-419,3,137,-723,-1228],
        [-148,4548,4948,-267,-4809,-4308],
        [5126,2529,-2701,-5598,-2358,3298],
        [-456,412,-91,520,-299,-108],
    ], dtype=np.float64
)
B1_fromThrow[:3, :] /= 100.
B1_fromThrow[3:, :] /= 10.
B1 = B1_fromThrow

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

A = np.zeros((3*n, 3))
for motor in range(n):
    dw = B1[3:, motor];
    A[(motor*3):((motor+1)*3), :] = -skew(dw)

def residual(parameters):
    r = parameters[:3]
    Fz_gt = parameters[3]
    B1f_gt = np.zeros((3, 6))
    B1f_gt[2] = Fz_gt
    y = (B1f_gt - B1[:3]).flatten(order='F')
    return A @ r - y

fit = least_squares(residual, x0=np.array([0., 0., 0., -1.]))
r = fit.x[:3]
Fz_gt = fit.x[3]
print(f"r: {r}")
print(f"Fz_gt: {Fz_gt}")
print(f"Residual sum of squares: {2 * fit.cost}")

# final matrix
B1f_fit = np.zeros((3, 6))
r_test = r + 0.0
for motor in range(n):
    dw = B1[3:, motor];
    B1f_fit[:, motor] = B1[:3, motor] - np.cross(dw, r_test)

print(f"Best fit B1 force matrix:\n {B1f_fit.round(1)}")
