"""
exciteGen.py: Signal excitation generators
--------------------------------
Generates excitation signals for system identification
of systems with independent and dependent actuators.
"""

import numpy as np
from scipy.integrate import cumulative_trapezoid
import matplotlib.pyplot as plt
from time import time
import pulp as pl

# helper functions
def inner(u, v, t):
    return np.trapezoid(u * v, t)

def normalized(v):
    return lambda t: v(t) / np.sqrt(inner(v, v, t))

def solve_gram_schmidt(n, V, t):
    # solve gram schmidt process to find orthogonalized basis for v_i on [0, 1]
    p = np.eye(n)
    g = np.eye(n)
    B = np.empty_like(V)
    bb = np.zeros(n)
    for k in range(n):
        for j in range(k):
            p[k, j] = inner(V[k], B[j], t) / bb[j]

        # calc gamma matrix for  w = gamma @ v  instead of  w_k = v_k - sum(p[k, :k] @ w[:k])
        g[k] -= p[k, :k] @ g[:k]

        # evaluate  w_k = gamma_k @ v
        B[k] = V[k]
        if k > 0:
            B[k] += g[k, :k] @ V[:k]

        # evaluate norm2 for the next iteration
        bb[k] = inner(B[k], B[k], t)

        # check for linear dependence
        if bb[k] < 1e-3:
            raise ValueError(f"Generated function {k+1} are (too) linearly dependent ({bb[k]} < 1e-3). Try different transformations.")

    # finalize gamma matrix such that g @ V  are orthonormal basis functions
    g /= np.sqrt(bb)[:, np.newaxis]
    g /= np.max(np.abs(g @ V))

    # finalize orthonormal basis functions
    B = g @ V

    return B, g

def maximize_excitation(v_min, v_max, u_lb, u_ub, F, G, integral_mode='equal'):
    """
    Maximize excitation by construction excitation functions of the form

           u_i = c_i * v_i + a_i

    subject to linear constraints:
        1. u_i(t) in [u_lb[i], u_ub[i]]  for all t
        2. sum(c_i) is maximized
        3. c_i >= 0  for all i
        4. if integral_mode == 'zero':
            a) c_i F_i  +  a_i G_i  ==  0  for all i
        5. if integral_mode == 'equal':
            a) c_i F_i  +  a_i G_i  ==  c_j F_j  +  a_j G_j  for all i != j

    Note: if F_i == int(v_i(t) dt) and G_i == T, then 4 and 5 enforce integral constraints on u_i

    Parameters:
        v_min: list of minimum values for each basis function
        v_max: list of maximum values for each basis function
        u_lb: list of lower bounds for the excitation signal
        u_ub: list of upper bounds for the excitation signal
        F: additional constraints, see above (4 and 5)
        G: additional constraints, see above (4 and 5)
        integral_mode: 'equal' or 'zero', see above (4 and 5)
    """

    n = len(v_min)
    assert len(v_max) == n
    assert len(u_lb) == n
    assert len(u_ub) == n
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
        prob += c[i] * v_min[i] + a[i] >= u_lb[i], f"MinAmplitude_{i+1}"
        prob += c[i] * v_max[i] + a[i] <= u_ub[i], f"MaxAmplitude_{i+1}"
        if integral_mode == 'equal':
            for j in range(i):
                if i != j:
                    # these should generate all n choose 2 constraints
                    # print(f"Adding equality constraint between {i} and {j}")
                    prob += c[i] * F[i] + a[i] * G[i] == c[j] * F[j] + a[j] * G[j], f"EqualIntegral_{i+1}_{j+1}"
        elif integral_mode == 'zero':
            # print(f'Forcing integral to 0 for function {i}')
            prob += c[i] * F[i] + a[i] * G[i] == 0, f"ZeroIntegral_{i+1}"

    # solve and retrieve results
    # do not print, but fail if not optimal or infeasible
    prob.solve(pl.PULP_CBC_CMD(msg=0))
    if prob.status != pl.LpStatusOptimal:
        raise ValueError("Linear program did not find an optimal solution.")
    # alternative: print solver output, but do not fail
    # prob.solve(pl.PULP_CBC_CMD(msg=1))

    cstar = np.array([pl.value(c[i]) for i in range(n)])
    astar = np.array([pl.value(a[i]) for i in range(n)])

    return cstar, astar

class BaseFunctions(object):
    @staticmethod
    def polynomial(p):
        return lambda t: p*t

    @staticmethod
    def cosine(w0):
        return lambda t: np.cos(w0 * t)

    @staticmethod
    def chirp(w0):
        return lambda t: np.cos(w0 * (1-t) * (1-t))

    @staticmethod
    def noise():
        return lambda t: np.random.uniform(-1, +1, size=t.shape)

class Transformations(object):
    @staticmethod
    def nextpower(v):
        return lambda t: v(t)**2

    @staticmethod
    def scale(v, beta):
        return lambda t: v(t * beta)

    @staticmethod
    def timeshift(v, tau):
        return lambda t: v(t + tau)

    @staticmethod
    def ampshift(v, a):
        return lambda t: a + v(t)

class ExcitationGenerator(object):
    def __init__(self, t):

        self.t = t
        self.S = len(t)
        self.T = t[-1] - t[0]
        self.vf = []
        # self.transformations = []
        self.actuators = []

    def add_library_function(self, v):
        # make sure v is a function
        if not callable(v):
            raise ValueError("v must be a callable function of time")

        self.vf.append(v)

    def add_actuator(self, type='independent', dependent_on=None, lb=-1., ub=+1.):
        if type not in ['independent', 'dependent']:
            raise ValueError("Invalid actuator type. Must be 'independent' or 'dependent'.")
        if type == 'dependent' and dependent_on is None:
            raise ValueError("Dependent actuators must specify 'dependent_on' parameter.")

        self.actuators.append({
            'type': type,
            'dependent_on': dependent_on,
            'lb': lb,
            'ub': ub,
        })

    def generate(self):
        if len(self.vf) != len(self.actuators):
            raise ValueError("Number of monomials must match number of actuators.")

        # assemble dependent and indenpendent actuator sets and check
        self.I = [i for i, act in enumerate(self.actuators) if act['type'] == 'independent']
        self.D = [i for i, act in enumerate(self.actuators) if act['type'] == 'dependent']
        for i in self.D:
            act = self.actuators[i]
            if act['dependent_on'] >= i:
                raise ValueError("Dependent actuator must depend on a previous actuator.")
            depact = self.actuators[act['dependent_on']]
            if depact['type'] != 'independent':
                raise ValueError("Dependent actuator must depend on an independent actuator.")
            if depact['lb'] * depact['ub'] <= 0:
                raise ValueError("Independent actuator bounds must be strictly positive or negative, if another actuator depends on it.")

        # extract info of actuators
        self.n = len(self.actuators)
        self.ni = len(self.I)
        self.nd = len(self.D)
        u_lb = np.array([self.actuators[i]['lb'] for i in range(self.n)])
        u_ub = np.array([self.actuators[i]['ub'] for i in range(self.n)])

        # just use vf list
        V = np.zeros((self.n, self.S))
        V[0] = self.vf[0](self.t)
        for i, v in enumerate(self.vf):
            V[i] = v(self.t)

        self.V = V

        # orthogonalize basis functions (g is such that B = g @ V are orthonormal)
        self.B, self.g = solve_gram_schmidt(self.n, self.V, self.t)
        b_min = np.min(self.B, axis=1)
        b_max = np.max(self.B, axis=1)
        B_int = np.trapezoid(self.B, self.t, axis=1)

        # optimize excitation of independent actuators
        self.cstari, self.astari = maximize_excitation(b_min[self.I], b_max[self.I],
                                           u_lb[self.I], u_ub[self.I],
                                           B_int[self.I], self.T*np.ones(self.ni),
                                           integral_mode='equal')

        self.U = np.zeros_like(self.B)
        self.U[self.I] = self.cstari[:, np.newaxis] * self.B[self.I]  +  self.astari[:, np.newaxis]

        # optimize excitation of dependent actuators
        if not self.nd:
            self.cstard, self.astard = [], []
            return

        ptilde = np.zeros_like(self.B[self.D])
        for k, d in enumerate(self.D):
            i = self.actuators[d]['dependent_on']
            ptilde[k] = self.B[d] / self.U[i]
            # should never divide by zero because Ui is forced to have lower bound > 0

        ptilde_min = np.min(ptilde, axis=1)
        ptilde_max = np.max(ptilde, axis=1)

        G = np.trapezoid(self.U[self.I], self.t, axis=1)

        self.cstard, self.astard = maximize_excitation(ptilde_min, ptilde_max,
                                           u_lb[self.D], u_ub[self.D],
                                           B_int[self.D], G,
                                           integral_mode='zero')

        self.U[self.D] = self.cstard[:, np.newaxis] * ptilde  +  self.astard[:, np.newaxis]

    def plot(self):
        fig, axs = plt.subplots(4, 1, figsize=(7, 8.5), sharex=True)

        # ensure that the style is readible without colors
        LINETYPES = ['-', '--', '-.', ':']

        # labels follow the logic: "Motor i" for independent actuators, "Elevon j" for dependent actuators
        LABELS = [f'Motor {i+1}' if i in self.I else f'Elevon {i+1-self.ni}' for i in range(self.n)]

        for i in range(self.n):
            axs[0].plot(self.t, self.V[i].T, label=LABELS[i], linewidth=2, linestyle=LINETYPES[i % len(LINETYPES)])
            axs[1].plot(self.t, self.B[i].T, label=LABELS[i], linewidth=2, linestyle=LINETYPES[i % len(LINETYPES)])
            axs[2].plot(self.t, self.U[i].T, label=LABELS[i], linewidth=2, linestyle=LINETYPES[i % len(LINETYPES)])

        # plot products for dependent actuators in a 4th plot
        if self.nd:
            for d in self.D:
                axs[3].plot(self.t, self.U[d].T * self.U[self.actuators[d]['dependent_on']].T,
                            LINETYPES[d],
                            linewidth=2,
                            color=axs[2].lines[d].get_color(),
                            label=f'$u_{d} * u_{{{self.actuators[d]["dependent_on"]}}}$')

        axs[0].set_title("Basis Functions", fontsize=14)
        axs[1].set_title("Orthogonalized Functions", fontsize=14)
        axs[2].set_title("Scaled Excitation Signals", fontsize=14)
        axs[3].set_title("Products of Dependent Actuators", fontsize=14)
        axs[-1].set_xlabel("Time", fontsize=12)

        # make legend appear above plot, and horizontal
        axs[0].legend(loc='upper center',ncol=self.n)

        # make legend appear normally for 4th plot
        if self.nd:
            axs[3].legend(loc='upper right')

        # set plot/subplot spoacing
        plt.subplots_adjust(top=0.95, bottom=0.08, left=0.1, right=0.95, hspace=0.25, wspace=0.2)

        for ax in axs:
            ax.set_ylim(-1.1, 1.1)
            ax.grid()

        axs[0].set_ylim(-1.1, 1.8)

        egd.fig_signals = fig

    def plot_inner_products(self):

        self.Z = self.U.copy()
        for d in self.D:
            ci = self.actuators[d]['dependent_on']
            self.Z[d] = self.U[d]*self.U[ci]

        figV, axV = plt.subplots(self.n, self.n, figsize=(10, 8), sharex=True, sharey=True)
        figB, axB = plt.subplots(self.n, self.n, figsize=(10, 8), sharex=True, sharey=True)
        figU, axU = plt.subplots(self.n, self.n, figsize=(10, 8), sharex=True, sharey=True)
        for i in range(self.n):
            for j in range(self.n):
                #%% plotting basis functions
                cumsumV = cumulative_trapezoid(self.V[i] * self.V[j], self.t, initial=0)
                cumsumB = cumulative_trapezoid(self.B[i] * self.B[j], self.t, initial=0)
                axV[i, j].plot(self.t, cumsumV, label=f'v{i+1} * v{j+1}')
                axB[i, j].plot(self.t, cumsumB, label=f'v{i+1} * v{j+1}')
                axV[i, j].set_title(f'Inner Product v{i+1} * v{j+1}')
                axV[i, j].grid(True)
                axB[i, j].set_title(f'Inner Product b{i+1} * b{j+1}')
                axB[i, j].grid(True)
                if i == self.n - 1:
                    axV[i, j].set_xlabel('t')
                if j == 0:
                    axV[i, j].set_ylabel('Inner Product')

                #%% plotting actuators impulses, PLEASE make these ifs prettier...
                axU[i, j].grid(True)
                if i == j:
                    cumsumU = cumulative_trapezoid(self.Z[i] - self.Z[i].mean(), self.t, initial=0)
                    axU[i, j].plot(self.t, cumsumU)
                    if i in self.D:
                        ci = self.actuators[i]['dependent_on']
                        axU[i, j].set_title(f'integral of u{i}*u{ci} - mean(u{i}*u{ci})')
                    else:
                        axU[i, j].set_title(f'integral of u{i} - mean(u{i})')
                else:
                    cumsumU = cumulative_trapezoid(self.Z[i] - self.Z[j], self.t, initial=0)
                    axU[i, j].plot(self.t, cumsumU)
                    if i in self.D:
                        ci = self.actuators[i]['dependent_on']
                        if j in self.D:
                            cj = self.actuators[j]['dependent_on']
                            axU[i, j].set_title(f'integral of u{i}*u{ci} - u{j}*u{cj}')
                        else:
                            axU[i, j].clear()
                            axU[i, j].set_title(f'integral of u{i}*u{ci} - u{j}')
                    else:
                        if j in self.D:
                            axU[i, j].clear()
                            cj = self.actuators[j]['dependent_on']
                            axU[i, j].set_title(f'integral of u{i} - u{j}*u{cj}')
                        else:
                            axU[i, j].set_title(f'integral of u{i} - u{j}')
                if i == self.n - 1:
                    axV[i, j].set_xlabel('t')
                if j == 0:
                    axV[i, j].set_ylabel('Integral')

    def output_indiflight(self):
        """
        Output excitation signals in a format suitable for inclusion in the indiflight codebase.
        """

        gout = np.zeros(int( (self.n*(self.n+1))/2 ) )
        k = 0
        for i in range(self.n):
            for j in range(i+1):
                gout[k] = self.g[i, j]
                k += 1

        print(f"""
    .K = {self.n},
    .tf = {0.5:.6f}f,
    .base_type = ORTHO_BASE_CHIRP,
    .base_param = {4*np.pi:.6f}f,
    .transform_types = {{
        ORTHO_TRANS_NONE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
    }},
    .transform_params = {{
        0.f,
        {0.85:.6f}f,
        {0.85**2:.6f}f,
        {0.85**3:.6f}f,
    }},
    .alpha = {{{', '.join([f'{a:.6f}f' for a in np.concatenate((self.cstari, self.cstard))])}}},
    .beta = {{{', '.join([f'{a:.6f}f' for a in np.concatenate((self.astari, self.astard))])}}},
    .dependency = {{{', '.join([str(-1) if i in self.I else str(self.actuators[i]['dependent_on']) for i in range(self.n)])}}},  // -1 for independent, otherwise index of dependency
    .mixing_matrix = {{
        {', '.join([f'{v:.6f}f' for v in gout])}
    }}
        """)

if __name__ == "__main__":
    t = np.linspace(0, 1, 1001)

    #%% test with independent actuators
    ## vi = lambda t: np.cos(4*np.pi*t)
    ## egi = ExcitationGenerator(t)
    ## egi.add_library_function(vi)
    ## egi.add_library_function(Transformations.scale(vi, 0.85**1))
    ## egi.add_library_function(Transformations.scale(vi, 0.85**2))
    ## egi.add_library_function(Transformations.scale(vi, 0.85**3))

    ## egi.add_actuator(type='independent', lb=+0.2, ub=+0.8)
    ## egi.add_actuator(type='independent', lb=+0.2, ub=+0.8)
    ## egi.add_actuator(type='independent', lb=+0.2, ub=+0.8)
    ## egi.add_actuator(type='independent', lb=+0.2, ub=+0.8)

    ## egi.generate()

    ## # print results
    ## print("Quadcopter")
    ## print("Coefficients for Independent Actuators:")
    ## print("Ui  = ", egi.cstari, "* Bi +", egi.astari)

    #%% test with dependent actuators
    vd = lambda t: np.cos(4*np.pi*(1-t)*(1-t))
    egd = ExcitationGenerator(t)
    egd.add_library_function(vd)
    egd.add_library_function(Transformations.scale(vd, 0.85))
    egd.add_library_function(Transformations.scale(vd, 0.85**2))
    egd.add_library_function(Transformations.scale(vd, 0.85**3))

    egd.add_actuator(type='independent', lb=+0.25, ub=+0.7)
    egd.add_actuator(type='independent', lb=+0.25, ub=+0.7)
    egd.add_actuator(type='dependent', lb=-0.6, ub=+0.6, dependent_on=0)
    egd.add_actuator(type='dependent', lb=-0.6, ub=+0.6, dependent_on=1)

    start = time()
    egd.generate()
    end = time()

    # print results
    print("Tailsitter")
    print()
    print(f"Excitation generation took {end - start:.4f} seconds.")
    print()
    print("Coefficients for Independent Actuators:")
    print("Ui  = ", egd.cstari, "* Bi +", egd.astari)

    print()
    print("Coefficients for Dependent Actuators:")
    print("Ud  = ", egd.cstard, "* Bd/Ui +", egd.astard)

    # plot results
    egd.plot()
    egd.fig_signals.savefig("signals.eps", format='eps')
    # egd.plot_inner_products()
