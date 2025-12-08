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


#import jax.numpy as jnp
#import jax
#
#jax.config.update('jax_platform_name', 'cpu')

from numba import njit
import numpy as np

@njit("f4[::1](f4[::1], f4[::1])")
def cross(u, v):
    w = np.empty(3, dtype=np.float32)
    w[0] = u[1]*v[2] - u[2]*v[1]
    w[1] = u[2]*v[0] - u[0]*v[2]
    w[2] = u[0]*v[1] - u[1]*v[0]
    return w

@njit("f4[:, ::1](f4[:, ::1], f4[:, ::1])")
def el_cross(u, v):
    w = np.empty(u.shape, dtype=np.float32)
    w[0, :] = u[1, :]*v[2, :] - u[2, :]*v[1, :]
    w[1, :] = u[2, :]*v[0, :] - u[0, :]*v[2, :]
    w[2, :] = u[0, :]*v[1, :] - u[1, :]*v[0, :]
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
def angularRateDerivative(OB, T, I, Iinv):
    return Iinv @ ( T  -  cross(OB, I @ OB) )

@njit("f4[::1](f4[::1],f4[::1],f4[::1],f4[::1],f4[::1])")
def motorModel(u, kESC, wmax, w, tau):
    wc = wmax * np.sqrt( kESC*u*u + (1 - kESC) * u )
    return ( wc - w ) / tau

# torque from motors?
# Ti  =  ri x Fi  +  Ii omegaMotorDot_Body  +  Omega_Body x Ii omegaMotor_Body
@njit("f4[::1](f4[::1], f4[:, ::1], f4[::1], f4[::1], f4[::1])")
def rotatingMassTorques(I, ax, w, wdot, OB):
    L = I * w * ax
    dLdt = I * wdot * ax

    Nr = len(I)
    OBs = np.empty((3, Nr), dtype=np.float32)
    for i in range(Nr):
        OBs[:, i] = OB

    return  np.sum(-dLdt  -  el_cross(OBs, L), axis=1)

@njit("f4[::1](f4[:, ::1],f4[:, ::1],f4[::1],f4[::1],f4[::1])")
def rotorForcesMoments(X, ax, w, k, cm):
    F = ax * k * w*w
    M = el_cross(X, F)  -  F * cm
    FM_B = np.empty((6, len(w)), dtype=np.float32)
    FM_B[:3, :] = F
    FM_B[3:, :] = M

    return np.sum(FM_B, axis=1)

@njit("f4[::1](f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1])")
def servoModel(u, d, dd, dmin, dmax, ddmin, ddmax, dddmin, dddmax, P, D):
    # rate/accel-limited second order model
    d_ref = np.clip(u, dmin, dmax)
    dd_ref = np.clip(P * (d_ref - d), ddmin, ddmax)
    return np.clip(D * (dd_ref - dd), dddmin, dddmax)

@njit("f4[::1](f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1],f4[::1])")
def wingElevonForcesMoments(vB, OB, w, d, dd, ddd, d0, cv, cvx, cO, cd, cdd, cddd):
    # regressors
    wwDd = w*w * np.sin( d - d0 )
    vB2 = np.abs(vB) * vB
    elev_sum = wwDd[0] + wwDd[1]
    elev_abs = abs(wwDd[0]) + abs(wwDd[1])
    elev_diff = wwDd[0] - wwDd[1]
    delev_sum = dd[0] + dd[1]
    delev_diff = dd[0] - dd[1]
    ddelev_sum = ddd[0] + ddd[1]
    ddelev_diff = ddd[0] - ddd[1]

    FM_B = np.empty((6,), dtype=np.float32)

    # AERO MODEL
    #               drag          rate damping
    FM_B[:3]  =  cv * vB2   +   cO[:3] * OB
    FM_B[3:]  =  cvx * vB2[0]   +   cO[3:] * OB

    # ACTUATION MODEL (elevons only)
    #              deflection              rate                      accel
    FM_B[0] += cd[0] * elev_sum    +   delev_sum * cdd[0]
    FM_B[1] += 0.
    FM_B[2] += cd[2] * elev_abs
    FM_B[3] += 0.
    FM_B[4] += cd[4] * elev_sum    +   delev_sum * cdd[4]    +   ddelev_sum * cddd[4]
    FM_B[5] += cd[5] * elev_diff   +   delev_diff * cdd[5]

    return FM_B

@njit("f4[::1](f4[::1],f4[::1],f4, f4[:, ::1])")
def phiTheoryForcesMoments(vB, OB, phi, Phi):
    V2 = np.dot(vB, vB)
    O2 = np.dot(OB, OB)

    eta = np.sqrt(V2 + phi*O2)
    etaB = np.hstack((vB, OB))

    return -eta * ( Phi @ etaB )

@njit("f4[::1](f4[::1],f4[::1],f4[::1],f4[::1], f4[::1], f4[::1], f4[::1], f4[::1])")
def elevonForcesMoments(w, d, dd, ddd, d0, cd, cdd, cddd):
    # regressors
    ww = w*w
    wwDd = ww * ( d - d0 )    # prop speeds ** 2 * sin ( elevon angles - zero-force angle )

    wwDd_diff = wwDd[0] - wwDd[1]
    dd_diff = dd[0] - dd[1]
    ddd_diff = ddd[0] - ddd[1]

    # elevon contribution
    FM_B = np.empty((6,), dtype=np.float32)

    # ACTUATION MODEL (elevons only)
    #              deflection              rate                      accel
    FM_B[0] = cd[0] * np.sum(wwDd)   +  cdd[0] * np.sum(dd)   +  cddd[0] * np.sum(ddd)
    FM_B[1] = 0.
    FM_B[2] = 0.
    FM_B[3] = 0.
    FM_B[4] = cd[4] * np.sum(wwDd)   +  cdd[4] * np.sum(dd)   +  cddd[4] * np.sum(ddd)
    FM_B[5] = cd[5] * wwDd_diff       +  cdd[5] * dd_diff      +  cddd[5] * ddd_diff

    return FM_B
