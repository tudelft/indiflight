import numpy as np
from scipy.spatial.transform import Rotation

N_ACT = 4
G = 1 # can be set to 9.81 to get correct hover thrust settings
W = np.eye(N_ACT) # actuator weighing matrix in hover

#%% do stuff

# standard effectiveness matrix of a quadrotor
Bf = np.zeros((3, N_ACT))
Bf[2, :] = -1

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

# adjust eigenpairs for solutions
for i, v in enumerate(V.T):
    scale = np.sqrt( G / ( v.T @ A @ v ) )
    eta = scale * v

    u = Nr @ eta
    print()
    print(f"--- Solution {i} ---")
    print(f"Hover distribution: {u}")
    print(f"Hover rotation {Br @ u}")
    print(f"Hover thrust direction {Bf @ u}")
    print(f"Local z from hover thrust {(-Bf @ u) / np.linalg.norm(Bf @ u)}")
    print(f"Rotation times local z {R @ np.array([0., 0., 1.])}")
    print(f"u.T (Bf.T Bf) u: {u.T @ Bf.T @ Bf @ u}")
