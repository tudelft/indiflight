
import numpy as np
import matplotlib.pyplot as plt
from time import time

import pulp as pl

from plotting import BlittedCursor


# helper functions
def inner(u, v, t):
    return np.trapezoid(u(t) * v(t), t)

def normalized(v):
    return lambda t: v(t) / np.sqrt(inner(v, v, t))

# transformations
def T_scale(v, beta):
    return lambda t: v(t*beta)

def T_timeshift(v, tau):
    return lambda t: v(t+tau)

def T_ampshift(v, a):
    return lambda t: a+v(t)

def T_nextpower(v):
    return lambda t: v(t)**2



# t = np.linspace(0, 4*np.pi, 1001)
# u1 = np.sin(t)
# u2 = np.cos(t)
# 
# fig, ax = plt.subplots(3, 1)
# ax[0].plot(t, u1, label='u1')
# ax[0].plot(t, u2, label='u2')
# ax[1].plot(t, u1*u2, label='u1*u2')
# ax[2].plot(t, u2-u1, label='u1-u2')
# 
# fig.show()




#%% monomial

np.random.seed(42)  # for reproducibility
S = 101
prng = np.random.random(S*10)*2 - 1

# v = [lambda t: (1-t)**3]
# v = [lambda t: np.sinc(10*(t-0.2)) * np.log(10*(t-0.2)**2+1.5)]  # v1]
# v = [lambda t: np.sin(20*t)+np.cos(60*t) + 8]  # monomial function
v = [lambda t: np.cos(20*(1-t)*(1-t))]
# v = [lambda t: np.cos(30*t)]
# v = [lambda t: prng[(S*t).astype(int)]]  # random piecewise constant function
# v = [lambda t: (t <= 0.33) * 0 + (t > 0.33) * np.cos(40*(1.-t)*(1.-t))]  # piecewise linear function

ni = 2
nd = 2
K = ni + nd







start = time()

# v.append(T_timeshift(v[0], -0.08))  # v2
# v.append(T_timeshift(v[1], -0.08))  # v3
# v.append(T_timeshift(v[2], -0.08))  # v3
v.append(T_scale(v[0], 0.8))  # v2
v.append(T_scale(v[1], 0.8))  # v2
v.append(T_scale(v[2], 0.8))  # v2
# v.append(T_scale(v[0], 0.9**2))  # v2
# v.append(T_scale(v[0], 0.9**3))  # v2
# v.append(T_scale(v[0], 0.9**4))  # v2
# v.append(T_scale(v[0], 0.9**5))  # v2
# v.append(T_scale(v[1], 0.9))  # v3
# v.append(T_scale(v[2], 0.9))  # v3
# v.append(T_scale(v[3], 0.9))  # v3
# v.append(T_scale(v[4], 0.9))  # v3
# v.append(T_scale(v[5], 0.9))  # v3
# v.append(T_scale(v[6], 0.9))  # v3
# v.append(T_scale(v[7], 0.9))  # v3
# v.append(T_timeshift(v[0], 0.8))  # v2
# v.append(T_timeshift(v[1], 0.8))  # v3
# v.append(T_timeshift(v[2], 0.8))  # v3
# v.append(T_timeshift(v[3], 0.8))  # v3
# v.append(T_timeshift(v[4], 0.8))  # v3

# v.append(T_nextpower(v[0]))
# v.append(T_nextpower(v[1]))
# v.append(T_nextpower(v[2]))
# v.append(T_nextpower(v[3]))
# v.append(T_nextpower(v[4]))

# normalize v
# v = [normalized(v[i]) for i in range(len(v))]

t = np.linspace(0, 1, 1001)
p = np.eye(K)
g = np.eye(K)

# orthogonalize the functions using Gram-Schmidt process

w = []
wn = []
auto_inner = []
x = np.zeros((len(t), K))
y = np.zeros((len(t), K))
for k in range(K):
    for j in range(k):
        p[k, j] = inner(v[k], w[j], t) / auto_inner[j]

    # calc gamma matrix for  w = gamma @ v  instead of  w_k = v_k - sum(p[k, :k] @ w[:k])
    g[k] -= p[k, :k] @ g[:k]

    # evaluate  w_k = gamma_k @ v 
    w.append(lambda t, k=k:
        v[k](t) + np.sum([g[k, i] * v[i](t) for i in range(k)], axis=0)
    )

    # evaluate inner product for the next iteration
    auto_inner.append(inner(w[k], w[k], t))

    # evaluate the orthogonalized function at the time points
    x[:, k] = v[k](t)
    y[:, k] = w[k](t)

# normalize gamma, and evaluate the orthonormalized functions
g /= np.sqrt(auto_inner)[:, np.newaxis]
g /= np.max(np.abs(x @ g.T))
yn = x @ g.T

# end timer
end = time()
print(f"Time taken for orthogonalization: {end - start:.4f} seconds")



#%% evaluate calculate pairwise inner products
inner_products_dict = np.zeros((K, K))
inner_products_ortho = np.zeros((K, K))
inner_products_norm = np.zeros((K, K))
for k in range(K):
    for j in range(K):
        inner_products_dict[k, j] = np.trapezoid(x[:, k] * x[:, j], t)
        inner_products_ortho[k, j] = np.trapezoid(y[:, k] * y[:, j], t)
        inner_products_norm[k, j] = np.trapezoid(yn[:, k] * yn[:, j], t)

# print inner products in a matrix
print("Inner Products Matrix Dictionary:")
print(inner_products_dict.round(6))

print("Inner Products Matrix Orthogonalized:")
print(inner_products_ortho.round(6))

print("Inner Products Matrix Normalized Orthogonalized:")
print(inner_products_norm.round(6))

print("Gamma Matrix:")
print(g.round(6))

print("set ortho_gamma = ", end='')
for i in range(K):
    for j in range(i+1):
        print(f'{g[i, j]:.6f}', end=', ')
print()

#%% linear programming solver for scaling and shifting factors

def max_excitation(ulower, uupper, deltalower, deltaupper, F, G, integral_mode='equal'):
    n = len(ulower)
    assert len(uupper) == n
    assert len(deltalower) == n
    assert len(deltaupper) == n
    assert len(F) == n
    assert len(G) == n

    if integral_mode not in ['equal', 'zero']:
        raise ValueError("integral_mode must be 'equal' or 'zero'")

    c = [pl.LpVariable(f"c_{i}", lowBound=0, upBound=None) for i in range(n)]
    a = [pl.LpVariable(f"a_{i}", lowBound=None, upBound=None) for i in range(n)]

    # objective 
    prob = pl.LpProblem("MaximizeExcitation", pl.LpMaximize)
    prob += pl.lpSum(c)

    # constraints
    for i in range(n):
        prob += c[i] * ulower[i] + a[i] >= deltalower[i], f"MinAmplitude_{i+1}"
        prob += c[i] * uupper[i] + a[i] <= deltaupper[i], f"MaxAmplitude_{i+1}"
        if integral_mode == 'equal':
            for j in range(i):
                if i != j:
                    # these should generate all n choose 2 constraints
                    print(f"Adding equality constraint between {i} and {j}")
                    prob += c[i] * F[i] + a[i] * G[i] == c[j] * F[j] + a[j] * G[j], f"EqualIntegral_{i+1}_{j+1}"
        elif integral_mode == 'zero':
            print(f'Forcing integral to 0 for function {i}')
            prob += c[i] * F[i] + a[i] * G[i] == 0, f"ZeroIntegral_{i+1}"

    # solve and retrieve results
    prob.solve(pl.PULP_CBC_CMD(msg=1))

    cstar = np.array([pl.value(c[i]) for i in range(n)])
    astar = np.array([pl.value(a[i]) for i in range(n)])

    return cstar, astar


#%% evaluate integrals of the orthonormalized functions

F = np.trapezoid(yn, t, axis=0)
umax = np.max(yn, axis=0)
umin = np.min(yn, axis=0)

#%% run linear program to scaling and shifting to satisfy secondary objectives for indipendent actuators

T = 1
G = np.ones(ni) * T
deltalower = np.ones(ni) * 0.2
deltaupper = np.ones(ni) * 0.8

cstar, astar = max_excitation(umin[:ni], umax[:ni],
                              deltalower, deltaupper,
                              F[:ni], G,
                              integral_mode='equal')

print("Optimal Scaling Factors c* =", cstar)
print("Optimal Shifting Factors a* =", astar)

# assemble final functions for independent actuators
z = cstar * yn[:, :ni] + astar



#%% run linear program to generate v for dependent actuators

T = 1
c = pl.LpVariable(f"c", lowBound=0, upBound=None)
a = pl.LpVariable(f"a", lowBound=None, upBound=None)
deltalower = np.ones(nd) * (-0.8)
deltaupper = np.ones(nd) * (+0.8)

# get min and max of the dependent actuators, and integral z
ptilde = yn[:, ni:] / z
ptilde_min = np.min(ptilde, axis=0)
ptilde_max = np.max(ptilde, axis=0)

G = np.trapezoid(z, t, axis=0)


cstar2, astar2 = max_excitation(ptilde_min, ptilde_max,
                                deltalower, deltaupper,
                                F[ni:], G,
                                integral_mode='zero')

print("Optimal Scaling Factors c* =", cstar2)
print("Optimal Shifting Factors a* =", astar2)

zj = cstar2 * ptilde + astar2


# plot v in same figure
plt.figure(figsize=(10, 4))
plt.plot(t, z, label=[f'z{i+1}' for i in range(ni)], linestyle='--')
plt.plot(t, zj, label=[f'zj{i+1}' for i in range(nd)], linestyle='--')
plt.title('Scaled and Shifted Orthogonalized Functions')
plt.xlabel('t')
plt.ylabel('z(t)')
plt.legend()
plt.grid(True)

#%% plot
# plt.close('all')

f, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axs[0].plot(t, x, label=[f'v{i+1}' for i in range(K)])
axs[0].set_ylabel('Dictorionary Functions')
axs[1].plot(t, y, label=[f'w{i+1}' for i in range(K)])
axs[1].set_ylabel('Orthogonalized Functions')
axs[2].plot(t, yn, label=[f'wn{i+1}' for i in range(K)])
axs[2].set_ylabel('Orthonormalized Functions')


for ax in axs:
    ax.legend()
    ax.grid(True)

axs[-1].set_xlabel('t')

# add cursor
f.subplots_adjust(left=0.1, bottom=0.1)

from scipy.integrate import cumulative_trapezoid

# show time evolution of the inner products
fig, ax = plt.subplots(K, K, figsize=(10, 8), sharex=True, sharey=True)
for k in range(K):
    for j in range(K):
        cumsum = cumulative_trapezoid(yn[:, k] * yn[:, j], t, initial=0)
        ax[k, j].plot(t, cumsum, label=f'v{k+1} * v{j+1}')
        ax[k, j].set_title(f'Inner Product wn{k+1} * wn{j+1}')
        ax[k, j].grid(True)
        if k == K - 1:
            ax[k, j].set_xlabel('t')
        if j == 0:
            ax[k, j].set_ylabel('Inner Product')

all_axes = axs.flatten().tolist() + ax.flatten().tolist()
cursor = BlittedCursor(axs, sharex=True)
cursor2 = BlittedCursor(ax.flatten().tolist(), sharex=True)

# plt.show()