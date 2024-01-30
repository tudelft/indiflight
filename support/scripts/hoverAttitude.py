#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

N_ACT = 4
G = 9.81 # can be set to 1 (direction only) or 9.81 (correctly scaled alloc)
W = np.eye(N_ACT) # actuator weighing matrix in hover

#%% do stuff

# standard effectiveness matrix of a quadrotor
Bf = np.zeros((3, N_ACT))
Bf[2, :] = -5

Br = np.zeros((3, N_ACT))
Br[0, :] = [-1, -1, 1, 1]
Br[1, :] = [-1, 1, -1, 1]
Br[2, :] = [1, -1, -1, 1]

# rotate randomly
R = Rotation.random().as_matrix()
Bf = R @ Bf
Br = R @ Br

# get rotation nullspace
Nr = np.linalg.qr(Br.T, 'complete')[0][:, 3:]

# get eigenpairs
Q = Nr.T @ W @ Nr
A = Nr.T @ Bf.T @ Bf @ Nr
L, V = np.linalg.eig(-np.linalg.inv(A) @ Q)

A = np.random.random((4,4))
A = A.T @ A 

M = np.random.random((4,4))
M = M.T @ M

# https://people.inf.ethz.ch/arbenz/ewp/Lnotes/2010/chapter12.pdf
q = np.array([[0.5, 0.5, 0.5, 0.5]]).T
v = A @ q
u = M @ q
rho = (v.T @ q) / (u.T @ q)
g = (v - rho * u)
for i in range(10):
    if i == 0:
        p = -g
    else:
        p = -g + (g.T @ M @ g) / (gkm1.T @ M @ gkm1) * p

    gkm1 = g
    d = (p.T @ g) / (p.T @ (rho * M @ p - A @ p))
    q += d * p
    v = A @ q
    u = M @ q
    rho = (v.T @ q) / (u.T @ q)
    g = (v - rho * u)


# adjust eigenpairs for solutions
for i, v in enumerate(V.T):
    scale = np.sqrt( G**2 / ( v.T @ A @ v ) )
    eta = scale * v

    u = Nr @ eta
    print()
    print(f"--- Solution {i} ---")
    print(f"Hover distribution: {u}")
    print(f"Hover rotation: {Br @ u}")
    print(f"Hover thrust direction: {Bf @ u}")
    print(f"Local z from hover thrust: {(-Bf @ u) / np.linalg.norm(Bf @ u)}")
    print(f"Rotation times local z: {R @ np.array([0., 0., 1.])}")
    print(f"sqrt(u.T (Bf.T Bf) u): {np.sqrt(u.T @ Bf.T @ Bf @ u)}")
