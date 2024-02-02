#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

N_ACT = 6
G = 9.81 # can be set to 1 (direction only) or 9.81 (correctly scaled alloc)
W = np.eye(N_ACT) # actuator weighing matrix in hover

#%% do stuff

# standard effectiveness matrix of a quadrotor
Bf = np.zeros((3, N_ACT))
Bf[2, :] = -5

Br = np.zeros((3, N_ACT))
Br[0, :] = [-1, -1, 1, 1, 2, 3]
Br[1, :] = [-1, 1, -1, 1, 2, 3]
Br[2, :] = [1, -1, -1, 1, 2, 3]

# rotate randomly
R = Rotation.random().as_matrix()
Bf = R @ Bf
Br = R @ Br

# get rotation nullspace
Q = np.linalg.qr(Br.T, 'complete')[0]
Nr = Q[:, 3:]

# get ground truth eigenpairs
H = Nr.T @ W @ Nr
A = Nr.T @ Bf.T @ Bf @ Nr
L, V = np.linalg.eig(np.linalg.inv(H) @ A)
vTrue = V[:, np.abs(L).argmax()] # abs probably not necessary since H and A are pos def? lets keep it to guard against spurious complex numbers

# fast inverse square root aka Quake 3 algorithm
# https://www.youtube.com/watch?v=p8u_k2LIZyo
import struct
USE_PACK = False
def fisqrt(x):
    if USE_PACK:
        # pack and unpack to simulate reinterpret_cast using float precision
        # boring...
        ix = struct.unpack('<I', struct.pack('<f', x))[0]
        iy = 0x5F3759DF - (ix >> 1)
        y = struct.unpack('<f', struct.pack('<I', iy))[0]
    else:
        # direct bit manipulation to get binary of IEEE 754 floats
        # fun!
        e = 0x7F + int(np.log2(x))
        m = int((x / (2 << (e-0x80)) - 1) * (2 << 22))
        ix = (e << 23) + m
        iy = 0x5F3759DF - (ix >> 1)
        y = (1 + (iy & 0x7FFFFF) / (2 << 22)) * 2**((iy >> 23) - 0x7F)

    # newton iteration
    y *= 1.5 - (0.5 * x * y * y)
    #y *= 1.5 - (0.5 * x * y * y)
    return y

# power iteration, let's do one per estimation loop iteration (use warmstarting eventually!)
v = np.random.random(N_ACT - 3)
i = 1
while i > 0:
    Av = A @ v
    Av2 = Av.T @ Av
    if Av2 < 1e-7:
        # almost orthogonal to largest eigenvector, reset
        v = np.random.random(N_ACT - 3)
        continue
    v = Av * fisqrt(Av2)
    #v = Av * 1/np.sqrt(Av2)
    i -= 1

# adjust eigenpairs for solutions
#for i, v in enumerate(V.T):
for i in range(1):
    scale = np.sqrt( G**2 / ( v.T @ A @ v ) )
    eta = scale * v

    u = Nr @ eta
    if sum(u) < 0:
        # is this nice/sufficient?
        u = -u

    print()
    print(f"--- Solution {i} ---")
    print(f"True eig(A): {vTrue}")
    print(f"Power eig(A) with fisqrt: {v}")
    print(f"Hover allocation: {u}")
    print(f"Hover rotational acceleration: {Br @ u}")
    print(f"Hover thrust direction: {Bf @ u}")
    print(f"Local z from hover thrust: {(-Bf @ u) / np.linalg.norm(Bf @ u)}")
    print(f"Rotation times local z: {R @ np.array([0., 0., 1.])}")
    print(f"sqrt(u.T (Bf.T Bf) u): {np.sqrt(u.T @ Bf.T @ Bf @ u)}")
