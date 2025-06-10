import numpy as np
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec

from plotting import local_rc, BlittedCursor
plt.rcParams.update(local_rc)

class Estimator(object):
    def __init__(self, n, d=1):
        self.n = n
        self.d = d
        self.N = 1

        self.theta = np.zeros((n, 1))
        self.theta[:] = np.nan
        self.P = np.zeros((n, n))
        self.P[:] = np.nan
        self.A = np.zeros((1, n))
        self.y = np.zeros((d, 1))
        self.e = np.zeros((d, 1))

        self.theta_h = []
        self.P_h = []
        self.A_h = []
        self.y_h = []
        self.e_h = []

        self.name = "Estimator"
        self.parNames = ["$\\theta$"] if self.n==1 else [f"$\\theta_{{ {i} }}$" for i in range(self.n)]
        self.regNames = [[f"$A_{{{i},{j}}}$" for j in range(self.n)] for i in range(self.d)]
        self.outNames = ["$y$"] if self.d==1 else [f"$y{{ {i} }}$" for i in range(self.d)]

    def setTitle(self, title):
        self.name = title

    def setNames(self, parameters, regressors, outputs):
        if len(parameters) != self.n:
            raise ValueError(f"There must be n={self.n} entries in parameter names, got {len(parameters)}")

        if len(regressors) != self.d:
            raise ValueError(f"There must be d={self.d} rows in regressor names, got {len(regressors)}")
        else:
            for j, line in enumerate(regressors):
                if len(line) != self.n:
                    raise ValueError(f"Each row in the regressor names have n={self.n} elements, got {len(line)} in row {j}")

        if len(outputs) != self.d:
            raise ValueError(f"There must be d={self.d} entries in output names, got {len(outputs)}")

        self.parNames = parameters
        self.regNames = regressors
        self.outNames = outputs

    def setParameters(self, theta):
        theta = np.asarray(theta)
        if theta.ndim == 0:
            theta = np.array([theta])
        elif theta.ndim > 1:
            theta = theta.squeeze()

        l = len(theta)
        if l == self.n:
            self.theta[:, 0] = theta
        else:
            raise ValueError(f"Parameter must be length n={self.n}, got {l}")

    def setCovariance(self, P):
        P = np.asarray(P)
        if P.shape != self.P.shape:
            raise ValueError(f"Covariance must be a symmetric numpy array of shape ({self.n}, {self.n}), got {P.shape}")
        if not np.allclose(P, P.T, atol=1e-8):
            raise ValueError(f"Covariance must be a symmetric")

        self.P[:] = P

    def newSample(self, A, y):
        # accept one-dimensional only if either n or d are 1
        A = np.asarray(A)
        if (A.ndim == 1) and (self.d == 1):
            if len(A) != self.n:
                raise ValueError(f"Regressors A have be length-n ({self.n}), got {len(A)}")
            A = A[np.newaxis]
        elif (A.ndim == 1) and (self.n == 1):
            if len(A) != self.d:
                raise ValueError(f"Regressors A have be length-d ({self.d}), got {len(A)}")
            A = A[:, np.newaxis]
        elif (A.ndim == 1) or (A.shape[0] != self.d) or (A.shape[1] != self.n):
            raise ValueError(f"Regressors A must have shape (d, n), ie ({self.d}, {self.n}), got {A.shape}")

        y = np.asarray(y)
        if y.ndim == 0:
            if self.d == 1:
                y = y[np.newaxis, np.newaxis]
            else:
                raise ValueError(f"Output y is singleton, but has to be length {self.d}")
        elif (y.ndim == 1):
            if len(y) == self.d:
                y = y[:, np.newaxis]
            else:
                raise ValueError(f"If output y is ndim=1, it has to be length {self.d}, got {len(y)}")
        else:
            if (y.shape != (self.d, 1)):
                raise ValueError(f"If output y is ndim=2, it has to be shape {(self.d, 1)}, got {y.shape}")

        self.A[:] = A
        self.y[:] = y

    def log(self):
        self.theta_h.append(self.theta.copy())
        self.P_h.append(self.P.copy())
        self.A_h.append(self.A.copy())
        self.y_h.append(self.y.copy())
        self.e_h.append(self.e.copy())

    def update(self):
        raise NotImplementedError("update must be implemented in a child class")

    def predictNew(self, A):
        return A @ self.theta

    def predictOnline(self):
        return [self.A_h[i] @ self.theta_h[i] for i in range(self.N)]

    def plotParameters(self, parGroups=None, outGroups=None, timeMs=None, sharey=True, zoomy=False, cursor=True, extra_rows=0):
        # parameters and variances
        if parGroups is None:
            parGroups = [[i] for i in range(self.n)]

        if outGroups is None:
            outGroups = [[i] for i in range(self.d)]

        if timeMs is None:
            self.timeMs = list(range(self.N))
            timeLabel = "iterations"
        else:
            self.timeMs = np.asarray(timeMs)
            timeLabel = "Time [ms]"

        with plt.rc_context(rc=local_rc):
            self.f = plt.figure()

            left=0.04
            bottom=0.06
            right=0.975
            top=0.925
            hspace=0.15
            wspace=0.25
            rWidth = (right - left + 0*0.3*wspace) * 1 / (1 + len(parGroups)) + left
            rHeight = (top - bottom) * 2 / (2 + len(outGroups)) + bottom

            faceGs = GridSpec(2, 2,
                               width_ratios=(rWidth, 1 - rWidth),
                               height_ratios=(rHeight, 1 - rHeight),
                               )
            faceGs.update(left=0., bottom=0., right=1.0, top=0.95, hspace=0, wspace=0)

            outerGs = GridSpec(2, 2,
                               width_ratios=(1, len(parGroups)),
                               height_ratios=(2, len(outGroups)),
                               )
            outerGs.update(left=left, bottom=bottom, right=right, top=top, hspace=hspace, wspace=wspace)

            colors = ['white', 'gray', 'blue', 'green']
            for i, col in enumerate(colors):
                grayAx = self.f.add_subplot(faceGs[i])
                grayAx.grid(False)
                grayAx.set_facecolor(col)
                grayAx.patch.set_alpha(0.3)
                grayAx.tick_params(axis='both',which='both',bottom=0,left=0,
                                  labelbottom=0, labelleft=0)

            parGs = outerGs[0, 1].subgridspec(2+extra_rows, len(parGroups))
            regGs = outerGs[1, 1].subgridspec(len(outGroups), len(parGroups))
            yGs   = outerGs[1, 0].subgridspec(len(outGroups), 1)

            parAxs = []
            varAxs = []
            yAxs = []
            regAxs = []
            self.all_axes = []
            self.extraAxes = []

            for i in range(len(parGroups)):
                parAx = self.f.add_subplot(parGs[0, i]); parAxs.append(parAx)

                varAx = self.f.add_subplot(parGs[1, i]); varAxs.append(varAx)
                varAx.set_yscale('log')

                self.all_axes.append(parAx)
                self.all_axes.append(varAx)

                if i > 0:
                    if sharey:
                        varAx.sharey(varAxs[0])
                        parAx.sharey(parAxs[0])
                else:
                    parAx.set_ylabel("Parameter(s)")
                    varAx.set_ylabel("Variance(s)")

            for i in range(len(outGroups)):
                yAx = self.f.add_subplot(yGs[i, 0]); yAxs.append(yAx)

                self.all_axes.append(yAx)

                regAxsRow = []
                for j in range(len(parGroups)):
                    regAx = self.f.add_subplot(regGs[i, j]); regAxsRow.append(regAx)
                    if j == 0:
                        regAx.set_ylabel("Regressor(s)")
                    if (j > 0) and sharey:
                        regAx.sharey(regAxsRow[0])
                    # if (i > 0) and sharey:
                    #     regAx.sharey(regAxs[0][j])
                regAxs.append(regAxsRow)

            for i in range(extra_rows):
                extraAxisRow = []
                for j in range(len(parGroups)):
                    extraAxisRow.append(self.f.add_subplot(parGs[2+i, j]))
                    self.all_axes.append(extraAxisRow[-1])
                self.extraAxes.append(extraAxisRow)

            x = np.array(self.theta_h)
            P = np.array(self.P_h)
            A = np.array(self.A_h)
            y = np.array(self.y_h)

            for parIdxs, parAx, varAx in zip(parGroups, parAxs, varAxs):
                maxy = 0.
                miny = 0.
                for i in parIdxs:
                    maxy = max(maxy, x[-1, i])
                    miny = min(miny, x[-1, i])
                    parAx.plot(self.timeMs, x[:, i], label=self.parNames[i])
                    varAx.plot(self.timeMs, P[:, i, i], label=f"var({self.parNames[i]})")
                if zoomy:
                    diffy = maxy - miny
                    maxy += diffy * 1.
                    miny -= diffy * 1.
                    parAx.set_ylim(bottom=miny, top=maxy)
                parAx.plot(self.timeMs, self.timeMs*0, "g--")
                parAx.legend()
                varAx.legend()

            yLastTheta = self.predictNew(A)
            yRealTime = np.array(self.predictOnline())
            printLegend = True
            for yIdxs, yAx in zip(outGroups, yAxs):
                for i in yIdxs:
                    yAx.plot(self.timeMs, y[:, i], label="Target")
                    # yAx.plot(timeMs, yLastTheta[:, i], label="A posteriori")
                    yAx.plot(self.timeMs, yRealTime[:, i], label="Real Time")
                yAx.set_ylabel("Output "+self.outNames[i])
                if printLegend:
                    legend_ypos = 0.38 / yAx.get_position().height #FIXME: this doesnt work
                    yAx.legend(loc='upper center', bbox_to_anchor=(0.5, legend_ypos))
                    printLegend = False

            for yIdxs, regAxRow in zip(outGroups, regAxs):
                for parIdxs, regAx in zip(parGroups, regAxRow):
                    for i in yIdxs:
                        for j in parIdxs:
                            regAx.plot(self.timeMs, A[:, i, j], label=self.regNames[i][j])
                    self.all_axes.append(regAx)
                    regAx.legend()

            self.f.suptitle(f"{self.name} -- Regressors, Parameters and Variance", fontsize=18)

            yAxs[-1].set_xlabel(timeLabel)
            for regAx in regAxs[-1]:
                regAx.set_xlabel(timeLabel)

            if cursor:
                self.curser = BlittedCursor(self.all_axes, self.f.canvas)

            return self.f

    def plotGains(self):
        # k and e
        raise NotImplementedError("todo")

class RLS(Estimator):
    def __init__(self, n, d=1, gamma=1e8, forgetting=0.995):
        super().__init__(n, d)

        self.n = n
        self.d = d

        self.K = np.empty((self.n, self.d))
        self.K[:] = np.nan
        self.lam = forgetting
        self.e = np.empty((self.d, 1))
        self.e[:] = np.nan

        self.setParameters(np.zeros((self.n, 1)))
        self.setCovariance(gamma * np.eye(n))

        self.K_h = []
        self.e_h = []
        self.lam_h = []

        self.setTitle("Recursive Least Squares")

    def log(self):
        super().log()
        self.K_h.append(self.K)
        self.e_h.append(self.e)
        self.lam_h.append(self.lam)

    def update(self):
        if self.N == 1:
            self.log()  # log initial conditions

        # shorthands
        theta = self.theta
        P = self.P
        A = self.A
        y = self.y
        lam = self.lam
        K = self.K
        e = self.e
        n = self.n
        d = self.d

        # vanilla RLS equations
        e[:] = y - A @ theta

        M = lam * np.eye(d) + A @ P @ A.T
        K[:] = ( P @ A.T ) @ np.linalg.inv(M)

        theta[:] += K @ e
        P[:] = ( P - K @ A @ P ) / lam

        # log result
        self.N += 1
        self.log()

class LMS(Estimator):
    def __init__(self, n, d=1, mu=1e-8):
        super().__init__(n, d)

        self.n = n
        self.d = d

        self.mu = mu
        self.e = np.empty((self.d, 1))
        self.e[:] = np.nan

        self.setParameters(np.zeros((self.n, 1)))
        self.setCovariance(np.eye(n))

        self.e_h = []
        self.mu_h = []

        self.setTitle("Least Mean Squares")

    def log(self):
        super().log()
        self.e_h.append(self.e)
        self.mu_h.append(self.mu)

    def update(self):
        if self.N == 1:
            self.log()  # log initial conditions

        # shorthands
        theta = self.theta
        e = self.e
        mu = self.mu
        y = self.y
        A = self.A

        # vanilla LMS equations
        e[:] = y - A @ theta

        theta[:] += mu * A.T * e

        # log result
        self.N += 1
        self.log()

class EMWV(Estimator):
    def __init__(self, forgetting=0.995):
        super().__init__(2, 1)
        self.lam = forgetting

        self.setTitle("EMWV")
        self.setNames(["mean", "variance"], [["dummy 1", "dummy 2"]], ["sample"])

    def log(self):
        super().log()

    def update(self):
        if self.N == 1:
            self.log()  # log initial conditions

        mean = self.theta[0, 0]
        var = self.theta[1, 0]
        sample = self.y[0, 0]

        diff = sample - mean
        self.theta[0, 0] += (1. - self.lam) * diff
        self.theta[1, 0] = self.lam * ( var + (1. - self.lam) * diff**2 )

        self.N += 1
        self.log()

class RLS_fortescue(Estimator):
    def __init__(self, n, d=1, gamma=1e8, forgetting_base=0.995, N0=1):
        super().__init__(n, d)

        self.n = n
        self.d = d

        self.K = np.empty((self.n, self.d))
        self.K[:] = 0.
        self.e = np.empty((self.d, 1))
        self.e[:] = np.nan
        self.lam = forgetting_base
        self.lam_base = forgetting_base
        self.N0 = N0

        self.setParameters(np.zeros((self.n, 1)))
        self.setCovariance(gamma * np.eye(n))

        self.K_h = []
        self.e_h = []
        self.lam_h = []

        self.setTitle("Recursive Least Squares")

    def log(self):
        super().log()
        self.K_h.append(self.K)
        self.e_h.append(self.e)
        self.lam_h.append(self.lam)

    def update(self):
        if self.N == 1:
            self.log()  # log initial conditions

        # shorthands
        theta = self.theta
        P = self.P
        A = self.A
        y = self.y
        K = self.K
        e = self.e
        n = self.n
        d = self.d

        # RLS with fortescue tuning
        e[:] = y - A @ theta
        e2 = e.T @ e
        self.lam = 1. - (1. - A @ K) * e2 / self.N0
        self.lam = np.clip(self.lam[0, 0], self.lam_base, 1.0)

        M = self.lam * np.eye(d) + A @ P @ A.T
        K[:] = ( P @ A.T ) @ np.linalg.inv(M)

        theta[:] += K @ e
        P[:] = ( P - K @ A @ P ) / self.lam

        # log result
        self.N += 1
        self.log()

    def plotParameters(self, **kwargs):
        # Call the parent method to initialize the plot
        super().plotParameters(extra_rows=1, **kwargs)
        self.extraAxes[0][0].plot(self.timeMs, self.lam_h, label="Forgetting factor")
