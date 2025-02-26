# Numba definitions to speed up common math operations used in PyNDIflight
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.


from numba import njit
import numpy as np

@njit("f4[::1](f4[::1], f4[::1])")
def cross(u, v):
    w = np.empty(3, dtype=np.float32)
    w[0] = u[1]*v[2] - u[2]*v[1]
    w[1] = u[2]*v[0] - u[0]*v[2]
    w[2] = u[0]*v[1] - u[1]*v[0]
    return w

@njit("f4[::1](f4[::1], f4[::1])")
def quatRotate(q, v):
    # crazy algorithm due to Fabian Giesen (A faster quaternion-vector multiplication)  # 15 multiplications, 15 additions
    # https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    # v' = v  +  q[0] * 2*cross(q[1:], v)  +  cross(q[1:], 2*cross(q[1:], v))
    tmp = (2. * cross(q[1:], v)).astype(np.float32)
    return v  +  q[0] * tmp  +  cross(q[1:], tmp)

@njit("f4[::1](f4[::1], f4[::1])")
def quaternionDerivative(q, w):
    wx, wy, wz = 0.5 * w
    qw, qx, qy, qz = q
    return np.array([ ( -wx*qx - wy*qy - wz*qz ),
                      (  wx*qw + wz*qy - wy*qz ),
                      (  wy*qw - wz*qx + wx*qz ),
                      (  wz*qw + wy*qx - wx*qy ) ], dtype=np.float32)

@njit("f4[::1](f4[::1], f4[::1], f4[:, ::1], f4[:, ::1])")
def angularRateDerivative(w, T, I, Iinv):
    return Iinv @ ( T  -  cross(w, I @ w) )

# torque from motors?
# Ti  =  ri x Fi  +  Ii omegaMotorDot_Body  +  Omega_Body x Ii omegaMotor_Body
@njit("f4[::1](f4, f4[::1], f4, f4, f4[::1])")
def rotatingMassTorques(Imotor, spinAxisBody, motorVelocity, motorAcceleration, bodyRates):
    L = Imotor * motorVelocity * spinAxisBody
    dLdt_body = Imotor * motorAcceleration * spinAxisBody

    return  dLdt_body  +  cross(bodyRates, L)

@njit("f4(f4,f4,f4,f4,f4)")
def motorModel(u, kESC, wmax, w, tau):
    wc = wmax * np.sqrt( kESC*u*u + (1 - kESC) * u )
    return ( wc - w ) / tau
