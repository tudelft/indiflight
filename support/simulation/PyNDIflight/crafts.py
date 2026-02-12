# Simplified multirotor flight dynamics and sensor models
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


import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.constants import g as GRAVITY

from .helpers import (
    cross,
    quatRotate,
    quaternionDerivative,
    angularRateDerivative,
    rotatingMassTorques,
    motorModel,
    rotorForcesMoments,
    servoModel,
    wingElevonForcesMoments,
    phiTheoryForcesMoments,
    elevonForcesMoments,
    )

class Rotor:
    def __init__(self, r=[0., 0., 0.], axis=[0., 0., -1.], wmax=4900., Tmax=4.5, kESC=0.5, cm=0.01, tau=0.02, Izz=1e-6, dir='rh'):
        self.r = np.asarray(r, dtype=np.float32)
        self.axis = np.asarray(axis, dtype=np.float32)
        self.axis /= np.linalg.norm(self.axis)
        self.wmax = wmax
        self.Tmax = Tmax
        self.kESC = kESC
        self.k = Tmax / self.wmax / self.wmax
        self.cm = cm
        self.tau = tau
        self.Izz = Izz
        if dir in ['rh', 'lh']:
            self.dir = -1. if dir=='lh' else +1.
        else:
            raise ValueError("dir must be one of 'rh', or 'lh'!")

        self.F = np.array([0., 0., 0.], dtype=np.float32)
        self.M = np.array([0., 0., 0.], dtype=np.float32)
        self.w = 0.

    def step(self, u, Omega, dt):
        wDot = motorModel( u, self.kESC, self.wmax, self.w, self.tau )
        self.w += dt * wDot
        self.F = self.axis * self.k * self.w*self.w
        self.M = cross(self.r, self.F)
        self.M -= self.dir * self.cm * self.F
        self.M -= rotatingMassTorques(self.Izz, self.axis*self.dir, self.w, wDot, Omega)


class Craft:
    def __init__(self, Nr=4, Ns=0):
        self.Nr = Nr
        self.Ns = Ns

        self.m = 1.
        self.I = np.eye(3, dtype=np.float32)
        self.Iinv = np.eye(3, dtype=np.float32)
        self.xI = np.array([0., 0., 0.], dtype=np.float32)
        self.vI = np.array([0., 0., 0.], dtype=np.float32)
        self.vB = np.array([0., 0., 0.], dtype=np.float32)
        self.fspB = np.array([0., 0., 0.], dtype=np.float32)
        self.q = np.array([1., 0., 0., 0.], dtype=np.float32)
        self.qInv = np.array([1., 0., 0., 0.], dtype=np.float32)
        self.ODotB = np.array([0., 0., 0.], dtype=np.float32)
        self.OB = np.array([0., 0., 0.], dtype=np.float32)

        self.throw_time = +np.inf
        self.throw_duration = 0.
        self.FthrowI = np.array([0., 0., 0.], dtype=np.float32)
        self.MthrowB = np.array([0., 0., 0.], dtype=np.float32)

        self.r_w = np.zeros((Nr,), dtype=np.float32)
        self.r_wdot = np.zeros((Nr,), dtype=np.float32)
        self.r_u = np.zeros((Nr,), dtype=np.float32)

        self.r_X = np.zeros((3, Nr), dtype=np.float32)
        self.r_ax = np.zeros((3, Nr), dtype=np.float32)
        self.r_k = 1e-6*np.ones((Nr,), dtype=np.float32)
        self.r_cm = 1e-2*np.ones((Nr,), dtype=np.float32)
        self.r_cm[:2:] *= -1.
        self.r_wmax = 1000*np.ones((Nr,), dtype=np.float32)
        self.r_tau = 0.02*np.ones((Nr,), dtype=np.float32)
        self.r_kESC = np.zeros((Nr,), dtype=np.float32)
        self.r_I = 1e-7*np.ones((Nr,), dtype=np.float32)

        self.s_d = np.zeros((Ns,), dtype=np.float32)
        self.s_u = np.zeros((Ns,), dtype=np.float32)

        self.FM_B = np.zeros((6,), dtype=np.float32)
        self.F_I = np.zeros((3,), dtype=np.float32)

    def __repr__(self):
        qWrong = np.zeros_like(self.q)
        qWrong[3] = self.q[0]
        qWrong[:3] = self.q[1:]
        rot = R.from_quat(qWrong)
        eulers = rot.as_euler('ZYX', degrees=True)
        return f"{self.__class__.__name__}( Nr={self.Nr}, Ns={self.Ns}, x={self.xI}m, v={self.vI}m/s, roll={eulers[2]}deg, pitch={eulers[1]}deg, yaw={eulers[0]}deg )"

    def throw(self, height=3.5, acc=45., wB=[0., 0., 0.], vHorz=[0., 0.], at_time=0.):
        force = self.m * ( acc + GRAVITY )

        # solve duration:
        # 
        # height = height after powered throw (sT) + altitude gained during coasting (sC)
        # sT = 0.5*a*t**2
        # vT = a*t
        # sC = 0.5*vT**2 / g,  becayse 0.5*vT**2 = g*sC
        # 
        # then, solve  height == sT + sC  for time
        self.throw_time = -at_time
        self.throw_duration = np.sqrt( 2. * height / (acc * (1. + acc / GRAVITY)) )

        self.FthrowI[:2] = self.m * np.asarray(vHorz) / self.throw_duration
        self.FthrowI[2] = -force
        self.MthrowB[:] = self.I @ ( wB / self.throw_duration )

    def setInertia(self, m, I):
        self.m = m
        self.I = I.astype(np.float32)
        self.Iinv = np.linalg.inv(I).astype(np.float32)

    def setRotor(self, i, X, ax=[0., 0., -1.], k=2e-7, cm=0.02, wmax=4000., tau=0.02, kESC=0.4, I=1e-7):
        self.r_X[:, i] = np.asarray(X, dtype=np.float32)
        self.r_ax[:, i] = np.asarray(ax, dtype=np.float32)
        self.r_k[i] = k
        self.r_cm[i] = cm
        self.r_wmax[i] = wmax
        self.r_tau[i] = tau
        self.r_kESC[i] = kESC
        self.r_I[i] = I

    def setPose(self, x=[0., 0., 0.], q=[1., 0., 0., 0.]):
        self.xI[:] = np.asarray(x, dtype=np.float32)
        self.q[:] = np.asarray(q, dtype=np.float32)

    def setTwist(self, v=[0., 0., 0.], w=[0., 0., 0.]):
        self.vI[:] = np.asarray(v, dtype=np.float32)
        self.OB[:] = np.asarray(w, dtype=np.float32)

    def setExternalForceInInertialFrame(self, F):
        self.FthrowI[:] = F

    def setExternalMomentInBodyFrame(self, M):
        self.MthrowB[:] = M

    def groundContact(self):
        z = self.xI[2]
        if z > 0.:
            # handle ground contact
            down = self.vI[2] > 0.
            self.F_I[2]  -= (1000 if down else 1000) * self.m * z
            self.F_I[:3] -= (100  if down else    1) * self.m * self.vI
            self.FM_B[3:] -= 1000 * self.I @ (np.sign(self.q[0]) * self.q[1:])
            self.FM_B[5] = 0.; # no yaw
            self.FM_B[3:] -= 100 * self.I @ self.OB

    def customPhysics(self, dt):
        # modify FM_B and/or F_I
        pass

    def tick(self, dt):
        self.FM_B[:] = 0.
        self.F_I[:] = 0.

        # step motors
        self.r_wdot[:] = motorModel(self.r_u, self.r_kESC, self.r_wmax, self.r_w, self.r_tau)
        self.r_w += dt * self.r_wdot

        # get rotor forces
        self.FM_B += rotorForcesMoments(self.r_X, self.r_ax, self.r_w, self.r_k, self.r_cm)
        self.FM_B[3:] += rotatingMassTorques(self.r_I,
                                             self.r_ax,
                                             np.sign(self.r_cm)*self.r_w,
                                             np.sign(self.r_cm)*self.r_wdot,
                                             self.OB)

        # ground contact
        self.groundContact()

        # throw forces
        self.throw_time += dt
        if self.throw_time > 0. and self.throw_time <= self.throw_duration:
            self.F_I += self.FthrowI
            self.FM_B[3:] += self.MthrowB

        # extra forces
        self.customPhysics(dt)

        # accumulate forces
        self.F_I += quatRotate(self.q, self.FM_B[:3])
        self.fspB[:] = quatRotate(self.qInv, self.F_I) / self.m

        # get ODotB and step rotational dynamics
        self.ODotB[:] = angularRateDerivative(self.OB, self.FM_B[3:], self.I, self.Iinv)
        # self.ODotB[:] = self.Iinv @ self.FM_B[3:]
        self.OB += dt * self.ODotB
        self.q  += dt * quaternionDerivative(self.q, self.OB)
        self.q[:] /= np.linalg.norm(self.q)
        self.qInv = self.q.copy()
        self.qInv[0] *= -1.
        self.vB = quatRotate(self.qInv, self.vI)

        # step position / velocity
        self.xI += dt * self.vI
        self.vI += dt * (self.F_I / self.m  +  np.array([0., 0., GRAVITY]))


class MultiRotor(Craft):
    def __init__(self, Nr=4):
        super().__init__(Nr=Nr)

    def calculateG1G2(self):
        # 2024-02-25 slightly nicer formulation for online learning (G2 not scaled with Tmax)
        # 
        #  let O = (Fx Fy Fz Mx My Mz)
        # idea: DeltaO = B1 * DeltaT  +  B2 * DeltaWdot
        # 
        # where B1 holds information about thrust axes and motor locations
        # and   B2 holds information about thrust axes and propeller inertia
        # 
        # using w = sqrt(T/k), first-order dynamics wdot = (w - w0)/tau and taylor 
        # expansion of the square root results in:
        # 
        #   DeltaO = B1 DeltaT  +  B2 / (2*w0*tau*k) * (DeltaT - DeltaTprev)
        #
        # Introduce the normalized unitless control U = T / Tmax
        #
        #   DeltaO = B1 Tmax DeltaU                +  B2 * Tmax / (2*tau*k*w0) * (DeltaU - DeltaUprev)
        #   DeltaO = B1 * k * omegaMax^2 * DeltaU  +  B2 * omegaMax^2 / (2*tau*w0) * (DeltaU - DeltaUprev)
        #
        # Introduce specific generalized forces A = (fx fy fz taux tauy tauz) with 
        # units (N/kg N/kg N/kg Nm/(kgm^2) Nm/(kgm^2) Nm/(kgm^2)) and
        #
        #   DeltaA = G1 DeltaU  +  G2 * omegaMax^2 / (2*tau*w0) * (DeltaU - DeltaUprev)
        #      where  G1   == (Minv B1) * k * omegaMax^2  , where (Minv B1 * k) can be learned online and then scaled with omegaMax^2 which is separetely learned online
        #        or   G1   == (Minv B1) * Tmax            , which seems more accurate, if available
        #      and    G2   == (Minv B2)                   , which can be learned online
        #      and    Minv == inv(diag(m,m,m,Ixx,Iyy,Izz)), called generalized mass matrix
        # 
        # this can later be inverted to compute DeltaU by solving:
        #
        #   DeltaA + G2n / w0 DeltaU_prev = ( G1 + G2n / w0 )  DeltaU
        #      where G2n = G2 * omegaMax^2 / (2*tau)
        #
        # or, assuming wdot feedback is available
        #
        #   DeltaA + G2 * wdot_prev = ( G1 + G2n / w0 ) DeltaU
        ##################
        N = len(self.rotors)

        B1 = np.zeros((6, N))
        B2 = np.zeros((6, N))
        for i, rotor in enumerate(self.rotors):
            # force contribution from thrust
            B1[:3, i] = rotor.axis

            # moment contribution from thrust
            # and moment contribution from rotor drag
            B1[3:, i] = cross(rotor.r, rotor.axis) \
                        -rotor.dir * rotor.cm * rotor.axis

            B1[:, i] *= rotor.Tmax

            # moment contribution from spinup
            B2[3:, i] = -rotor.dir * rotor.axis * rotor.Izz

        M = np.zeros((6,6))
        M[:3, :3] = self.m * np.eye(3)
        M[3:, 3:] = np.diag(np.diag(self.I)) # remove offdiagonal elements
        #M[3:, 3:] = self._I # isnt this more accurate?

        G1 = np.linalg.solve(M, B1)
        G2 = np.linalg.solve(M, B2)
        G2_scaler = np.array([0.5 * r.wmax**2 / (0.5*r.tau) for r in self.rotors])
        return G1, G2, G2_scaler

    def checkHover(self):
        G1, _, _ = self.calculateG1G2()
        Qr, Rr = np.linalg.qr(G1[3:, :].T, 'complete')
        if (len(self.rotors) < 4) or (np.abs(np.diag(Rr)) < 1e-3).any():
            print(f"\nWARNING: generated craft has no control over some rotation axis or axes.")
            return False
        else:
            Nr = Qr[:, 3:] # rotational nullspace
            A = Nr.T @ G1[:3, :].T @ G1[:3, :] @ Nr
            v, V = np.linalg.eig(A)
            # calculate most effeicient hover allocation with 1.1 thrust to weight margin
            ustar = ( 1.1 * 9.81 / np.sqrt(max(v)) ) * (Nr @ V[:, np.argmax(v)])
            if not ( ((ustar >= 0.) & (ustar <= 1.)).all() or ((ustar >= -1.) & (ustar <= 0.)).all()):
                print(f"\nWARNING: generated craft does not have enough thrust-to-weight to hover without rotation. Double check rotation directions")
                return False

        return True

    def customPhysics(self, dt):
        return super().customPhysics(dt)

import collections

class Tailsitter(Craft):
    def __init__(self):
        super().__init__(Nr=2, Ns=2)

#        if (self.Nr != 2) or (self.Ns != 2):
#            raise NotImplementedError("Must be 2 rotors and 2 servos")

        self.d0 = np.array([-0.3170, -0.1578], dtype=np.float32)

        # drag and rate damping
        self.cv = np.array([-0.7454, -0.1554, 0.], dtype=np.float32)
        self.cvx = np.array([0., 0.01388, 0.], dtype=np.float32)
        self.cO = np.array([0., 0., 0., -0.00865, -0.0100, -0.0211], dtype=np.float32)

        # elevon contribution
        self.cd = np.array([-2.14e-7, 0., 0., 0., -4.470e-8, -1.08e-7], dtype=np.float32)
        self.cdd = np.array([0., 0., 0., 0., 0., 0.], dtype=np.float32)
        self.cddd = np.array([0., 0., 0., 0., -3.702e-5, 0.], dtype=np.float32)

        # servo data/states
        self.s_u = np.zeros((self.Ns), dtype=np.float32) # input command: +1 equals +100 deg
        self.s_d = np.zeros((self.Ns), dtype=np.float32) # servo state in radians
        self.s_dd = np.zeros((self.Ns), dtype=np.float32)
        self.s_dmin   = -1.75*np.ones((self.Ns), dtype=np.float32)
        self.s_dmax   = +1.75*np.ones((self.Ns), dtype=np.float32)
        self.s_ddmin  = -11.*np.ones((self.Ns), dtype=np.float32)
        self.s_ddmax  = +11.*np.ones((self.Ns), dtype=np.float32)
        self.s_dddmin = -250.*np.ones((self.Ns), dtype=np.float32)
        self.s_dddmax = +250.*np.ones((self.Ns), dtype=np.float32)
        self.s_P  = +45.*np.ones((self.Ns), dtype=np.float32)
        self.s_D  = +80.*np.ones((self.Ns), dtype=np.float32)
        self.s_delay = 0.04
        self.s_u_buffer = collections.deque(maxlen=1000)

    def customPhysics(self, dt):
        self.s_u_buffer.append((dt, self.s_u.copy()))

        tac = 0.
        s_u = self.s_u_buffer[0][1]  # oldest element
        for i in range(len(self.s_u_buffer)-1, -1, -1):
            tac += self.s_u_buffer[i][0] # time
            if tac > self.s_delay:
                s_u = self.s_u_buffer[i][1]
                break

        s_ddd = servoModel(s_u * 100. * np.pi / 180.,
                           self.s_d, self.s_dd,
                           self.s_dmin, self.s_dmax,
                           self.s_ddmin, self.s_ddmax,
                           self.s_dddmin, self.s_dddmax,
                           self.s_P, self.s_D)

        self.s_dd += dt * s_ddd
        self.s_d += dt * self.s_dd

        self.FM_B += wingElevonForcesMoments(
            self.vB, self.OB,
            self.r_w, self.s_d, self.s_dd, s_ddd,
            self.d0,
            self.cv, self.cvx, self.cO,
            self.cd, self.cdd, self.cddd)

class TailsitterPhi(Craft):
    def __init__(self):
        super().__init__(Nr=2, Ns=2)

#        if (self.Nr != 2) or (self.Ns != 2):
#            raise NotImplementedError("Must be 2 rotors and 2 servos")

        # phi theory coefficients
        self.phi = 0.0
        self.Phi = np.zeros((6,6), dtype=np.float32)

        # elevon contribution
        self.cd = np.zeros((6,), dtype=np.float32)
        self.cdd = np.zeros((6,), dtype=np.float32)
        self.cddd = np.zeros((6,), dtype=np.float32)
        self.d0 = np.zeros((2,), dtype=np.float32)

        # servo data/states
        self.s_u = np.zeros((self.Ns), dtype=np.float32) # input command: +1 equals +100 deg
        self.s_d = np.zeros((self.Ns), dtype=np.float32) # servo state in radians
        self.s_dd = np.zeros((self.Ns), dtype=np.float32)
        self.s_dmin   = -1.75*np.ones((self.Ns), dtype=np.float32)
        self.s_dmax   = +1.75*np.ones((self.Ns), dtype=np.float32)
        self.s_ddmin  = -11.*np.ones((self.Ns), dtype=np.float32)
        self.s_ddmax  = +11.*np.ones((self.Ns), dtype=np.float32)
        self.s_dddmin = -250.*np.ones((self.Ns), dtype=np.float32)
        self.s_dddmax = +250.*np.ones((self.Ns), dtype=np.float32)
        self.s_P  = +45.*np.ones((self.Ns), dtype=np.float32)
        self.s_D  = +80.*np.ones((self.Ns), dtype=np.float32)
        self.s_delay = 0.03
        self.s_u_buffer = collections.deque(maxlen=1000)

    def setPhiModel(self, phi, Phi):
        self.phi = phi
        self.Phi[:] = np.asarray(Phi, dtype=np.float32)

    def setElevonModel(self, cd, cdd, cddd, d0):
        self.cd[:] = np.asarray(cd, dtype=np.float32)
        self.cdd[:] = np.asarray(cdd, dtype=np.float32)
        self.cddd[:] = np.asarray(cddd, dtype=np.float32)
        self.d0[:] = np.asarray(d0, dtype=np.float32)

    def customPhysics(self, dt):
        self.s_u_buffer.append((dt, self.s_u.copy()))

        tac = 0.
        s_u = self.s_u_buffer[0][1]  # oldest element
        for i in range(len(self.s_u_buffer)-1, -1, -1):
            tac += self.s_u_buffer[i][0] # time
            if tac > self.s_delay:
                s_u = self.s_u_buffer[i][1]
                break

        s_ddd = servoModel(s_u * 100. * np.pi / 180.,
                           self.s_d, self.s_dd,
                           self.s_dmin, self.s_dmax,
                           self.s_ddmin, self.s_ddmax,
                           self.s_dddmin, self.s_dddmax,
                           self.s_P, self.s_D)

        self.s_dd += dt * s_ddd
        self.s_d += dt * self.s_dd

        self.FM_B += phiTheoryForcesMoments(self.vB,
                                            self.OB,
                                            self.phi,
                                            self.Phi)
        self.FM_B += elevonForcesMoments(self.r_w, self.s_d, self.s_dd, s_ddd,
                                         self.d0,
                                         self.cd, self.cdd, self.cddd)



class IMU:
    def __init__(self, uav, r=[0., 0., 0.], qBody=[1., 0., 0., 0.], accBias=[0., 0., 0.], accStd=0.0, gyroBias=[0., 0., 0.], gyroStd=0.0):
        self.uav = uav

        self.r = np.asarray(r, dtype=np.float32)
        self.qInv = np.asarray(qBody, dtype=np.float32)
        self.qInv[0] *= -1.

        self.acc = np.zeros(3, dtype=np.float32)
        self.accBias = np.asarray(accBias, dtype=np.float32)
        self.accStd = accStd

        self.gyro = np.zeros(3, dtype=np.float32)
        self.gyroBias = np.asarray(gyroBias, dtype=np.float32)
        self.gyroStd = gyroStd

    def update(self):
        accAtImu = self.uav.fspB + cross(self.uav.ODotB, self.r) + cross(self.uav.OB, cross(self.uav.OB, self.r))
        self.acc[:] = quatRotate(self.qInv, accAtImu) + np.random.normal(self.accBias, self.accStd)
        self.gyro[:] = quatRotate(self.qInv, self.uav.OB) + np.random.normal(self.gyroBias, self.gyroStd)
