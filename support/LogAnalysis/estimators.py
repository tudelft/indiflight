import numpy as np
import numbers
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec

from pyFlightPlotter import local_rc
plt.rcParams.update(local_rc)

class Estimator(object):
    def __init__(self, n, d=1):
        self.n = n
        self.d = d
        self.N = 0

        self.theta = np.zeros((n, 1))
        self.theta[:] = np.nan
        self.P = np.zeros((n, n))
        self.P[:] = np.nan
        self.A = np.zeros((d, n))
        self.y = np.zeros((d, 1))
        self.e = np.zeros((d, 1))

        self.t = 0.

        self.theta_h = []
        self.P_h = []
        self.A_h = []
        self.y_h = []
        self.e_h = []
        self.t_h = []

        self.A_acc = []
        self.y_acc = []
        self.t_acc = []

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

    def newSample(self, A, y, t):
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

        if not isinstance(t, numbers.Number):
            raise ValueError(f"Time t must be a number, got {type(t)}")

        self.A_acc.append(A.copy())
        self.y_acc.append(y.copy())
        self.t_acc.append(t)

    def log(self):
        self.theta_h.append(self.theta.copy())
        self.P_h.append(self.P.copy())
        self.e_h.append(self.e.copy())

    def update(self):
        self.N_batch = len(self.A_acc)

        self.theta_h.extend([self.theta.copy()]*(self.N_batch-1))
        self.e_h.extend([self.e.copy()]*(self.N_batch-1))
        self.P_h.extend([self.P.copy()]*(self.N_batch-1))
        self.A_h.extend(self.A_acc)
        self.y_h.extend(self.y_acc)
        self.t_h.extend(self.t_acc)

        self.A[:] = self.A_acc[-1]
        self.y[:] = self.y_acc[-1]

        # reset accumulators
        self.A_acc = []
        self.y_acc = []
        self.t_acc = []

    def predictNew(self, A):
        return A @ self.theta

    def predictOnline(self):
        return [self.A_h[i] @ self.theta_h[i] for i in range(self.N)]

    def plotParameters(self, parGroups=None, truePars=None, outGroups=None, parGroupNames=None, outGroupsNames=None, sharey=True, zoomy=False, extra_rows=0, figsize=None, uncertainty=False):
        # parameters and variances
        if parGroups is None:
            parGroups = [[i] for i in range(self.n)]

        if truePars is None:
            truePars = [[None]*len(g) for g in parGroups]

        if parGroupNames is None:
            parGroupNames = [f"Group {i}" for i in parGroups]

        if outGroups is None:
            outGroups = [[i] for i in range(self.d)]

        if outGroupsNames is None:
            outGroupsNames = [f"Group {i}" for i in outGroups]

        if figsize is None:
            figsize = (16, 9)

        timeLabel = "Time [s]"
        self.t_h = np.asarray(self.t_h)

        with plt.rc_context(rc=local_rc):
            self.f = plt.figure(figsize=figsize)

            left=0.04
            bottom=0.06
            right=0.975
            top=0.91
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
                parAx.set_title(parGroupNames[i])

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
                    self.all_axes.append(regAxsRow[-1])
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

            if x.shape[0] == 0:
                raise RuntimeError("No data to plot, run update() first")

            has_bounds = hasattr(self, 'theta_bounds_h') and (len(self.theta_bounds_h) == x.shape[0])
            if has_bounds:
                bounds = np.array(self.theta_bounds_h)

            if hasattr(self, 'NIS_h') and (len(self.NIS_h) == x.shape[0]):
                NIS = np.array(self.NIS_h)
                axNIS = self.f.add_subplot(outerGs[0, 0])
                axNIS.plot(self.t_h, NIS, label="NIS")
                # axNIS.set_yscale('log')
                axNIS.set_title("Normalized Innovation Squared")
                axNIS.set_ylabel("NIS")
                axNIS.set_xlabel(timeLabel)
                axNIS.legend()
                self.all_axes.append(axNIS)


            for parIdxs, parAx, varAx, truePar in zip(parGroups, parAxs, varAxs, truePars):
                maxy = 0.
                miny = 0.
                for i, trueParVal in zip(parIdxs, truePar):
                    maxy = max(maxy, x[-1, i])
                    miny = min(miny, x[-1, i])
                    parAx.plot(self.t_h, x[:, i], label=self.parNames[i])
                    if has_bounds:
                        # area plot of lower and upper bounds using the same color as the line
                        parAx.fill_between(self.t_h, bounds[:, i, 0], bounds[:, i, 1],
                                           color=parAx.lines[-1].get_color(),
                                           alpha=0.3, label=None)
                    if trueParVal is not None:
                        parAx.plot(self.t_h, np.ones_like(self.t_h)*trueParVal, "--", color=parAx.lines[-1].get_color(), label=None)
                    varAx.plot(self.t_h, P[:, i, i], label=f"var({self.parNames[i]})")
                if zoomy:
                    diffy = maxy - miny
                    maxy += diffy * 1.
                    miny -= diffy * 1.
                    parAx.set_ylim(bottom=miny, top=maxy)
                # parAx.plot(self.t_h, self.t_h*0, "g--")
                parAx.legend(fontsize=7)
                # varAx.legend()

            yLastTheta = self.predictNew(A)
            yRealTime = np.array(self.predictOnline())
            printLegend = True
            for yIdxs, yAx in zip(outGroups, yAxs):
                for i in yIdxs:
                    yAx.plot(self.t_h, y[:, i], label="Target")
                    yAx.plot(self.t_h, yRealTime[:, i], label="Real Time")
                    yAx.plot(self.t_h, yLastTheta[:, i], label="A posteriori")
                yAx.set_ylabel("Output "+self.outNames[i])
                if printLegend:
                    # legend_ypos = 0.38 / yAx.get_position().height #FIXME: this doesnt work
                    # yAx.legend(loc='upper center', bbox_to_anchor=(0.5, legend_ypos))
                    # printLegend = False
                    yAx.legend(loc='upper center')
                    printLegend = False

            for yIdxs, regAxRow in zip(outGroups, regAxs):
                for parIdxs, regAx in zip(parGroups, regAxRow):
                    for i in yIdxs:
                        for j in parIdxs:
                            regAx.plot(self.t_h, A[:, i, j], label=self.regNames[i][j])
                    self.all_axes.append(regAx)
                    # regAx.legend()

            self.f.suptitle(f"{self.name} -- Regressors, Parameters and Variance", fontsize=18)

            yAxs[-1].set_xlabel(timeLabel)
            for regAx in regAxs[-1]:
                regAx.set_xlabel(timeLabel)

            return self.f

    def plotGains(self):
        # k and e
        raise NotImplementedError("todo")

    def diagnose(self, i, output_name=None):
        if output_name is None:
            output_name = f"Output {i}"

        X = np.array(self.A_h)[:, i, :]
        idx_nonzero = np.linalg.norm(X, axis=0) > 1
        X = X[:, idx_nonzero]  # remove zero columns
        Y = np.array(self.y_h)[:, i]
        M = X.shape[1]

        U,s,Vt = np.linalg.svd(X, full_matrices=False)
        cond = s.max()/s.min()
        if cond > 1e12:
            print(f"Warning: regressor matrix ill-conditioned (cond={cond:.2e})")

        eps = np.finfo(float).eps
        tol = max(X.shape)*eps*s.max()
        rank = np.sum(s > tol)

        if rank < M:
            print(f"Warning: regressor matrix rank deficient (rank={rank} < {M})")

        from sklearn.linear_model import LinearRegression

        VIF = np.zeros(M)
        for j in range(M):
            Xj = X[:, j]
            Xothers = np.delete(X, j, axis=1)
            lr = LinearRegression().fit(Xothers, Xj)
            R2 = lr.score(Xothers, Xj)
            VIF[j] = 1.0/(1-R2)

        # pairwise correlations
        corr_matrix = np.corrcoef(X, rowvar=False)

        # use matplotlib to plot a heatmap of correlation matrix
        f, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(corr_matrix, cmap='coolwarm', vmin=-1, vmax=1)
        f.colorbar(im, label='Correlation Coefficient')
        # add values (rounded to 2 decimals) on the heatmap
        for m in range(M):
            for n in range(M):
                ax.text(n, m, f"{corr_matrix[m, n]:.2f}", ha='center', va='center', color='black', fontsize=8)

        ax.set_title(f'Reg Corr Mtx for {output_name} -- {self.name}')
        ax.set_xticks(ticks=np.arange(M))
        ax.set_xticklabels(labels=[f"X{i}" for i in range(M)], rotation=45)
        ax.set_yticks(ticks=np.arange(M))
        ax.set_yticklabels(labels=[f"X{i}" for i in range(M)])
        theta_hat = np.linalg.lstsq(X, Y, rcond=None)[0]
        res = Y - X.dot(theta_hat)
        err_corrs = np.array([np.corrcoef(res.squeeze(), X[:,j])[0,1] for j in range(M)])

        return f, VIF, corr_matrix, err_corrs, X, Y, theta_hat

class LS(Estimator):
    def __init__(self, n, d=1, gamma=1e8):
        super().__init__(n, d)

        self.setParameters(np.zeros((self.n, 1)))
        self.setCovariance(gamma * np.eye(n))

        self.setTitle("Least Squares")

    def update(self):
        super().update()

        # vanilla LS equations
        y = np.vstack(self.y_h)
        A = np.vstack(self.A_h)

        self.theta[:], _, _, _ = np.linalg.lstsq(A, y, rcond=None)

        # log result
        self.N += self.N_batch
        self.log()

class RLS(Estimator):
    def __init__(self, n, d=1, gamma=1e8, forgetting=0.995):
        super().__init__(n, d)

        self.setParameters(np.zeros((self.n, 1)))
        self.setCovariance(gamma * np.eye(n))

        # additinoal parameters for RLS
        self.K = np.empty((self.n, self.d))
        self.K[:] = np.nan
        self.lam = forgetting
        self.NIS = 1.
        self.theta_bounds = np.empty((self.n, 2))
        self.theta_bounds[:] = np.nan

        self.K_h = []
        self.lam_h = []
        self.NIS_h = []
        self.theta_bounds_h = []

        self.setTitle("Recursive Least Squares")

    def log(self):
        super().log()
        self.K_h.append(self.K.copy())
        self.lam_h.append(self.lam)
        self.NIS_h.append(self.NIS)
        self.theta_bounds_h.append(self.theta_bounds.copy())

    def update(self):
        super().update()

        # shorthands
        theta = self.theta
        theta_bounds = self.theta_bounds
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
        Minv = np.linalg.inv(M)

        NISk = (e.T @ Minv @ e)[0, 0]
        self.NIS = 0.995 * self.NIS + (1. - 0.995) * NISk

        K[:] = ( P @ A.T ) @ Minv

        theta[:] += K @ e
        P[:] = ( P - K @ A @ P ) / lam

        # 99% confidence bounds using normal test statistic (assuming N is large)
        theta_var = self.NIS*np.diag(P)
        from scipy.stats import norm
        norm_val = norm.ppf(1 - (1 - 0.997)/2) # icdf just a shittier name
        theta_bounds[:, 0] = theta[:, 0] - norm_val * np.sqrt(theta_var)
        theta_bounds[:, 1] = theta[:, 0] + norm_val * np.sqrt(theta_var)

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

class EWMV(Estimator):
    def __init__(self, forgetting=0.995):
        super().__init__(2, 1)
        self.lam = forgetting

        self.setTitle("EMWV")
        self.setNames(["mean", "variance"], [["dummy 1", "dummy 2"]], ["sample"])

    def log(self):
        super().log()

    def update(self):
        super().update()
        # if self.N == 1:
        #     self.log()  # log initial conditions

        mean = self.theta[0, 0]
        var = self.theta[1, 0]
        sample = self.y[0, 0]

        diff = sample - mean
        self.theta[0, 0] += (1. - self.lam) * diff
        self.theta[1, 0] = self.lam * ( var + (1. - self.lam) * diff**2 )

        self.N += 1
        self.log()

class Welford(Estimator):
    def __init__(self):
        super().__init__(2, 1)

        self.setTitle("Welford")
        self.setNames(["mean", "variance"], [["dummy 1", "dummy 2"]], ["sample"])

    def log(self):
        super().log()

    def update(self):
        super().update()
        # if self.N == 1:
        #     self.log()  # log initial conditions

        mean = self.theta[0, 0]
        var = self.theta[1, 0]
        sample = self.y[0, 0]

        self.N += 1
        diff = sample - mean
        mean += diff / self.N
        var += diff * (sample - mean)

        self.theta[0, 0] = mean
        if self.N > 1:
            self.theta[1, 0] = var / (self.N - 1)
        else:
            self.theta[1, 0] = 0.

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
        super().update()

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
        self.extraAxes[0][0].plot(self.t_h, self.lam_h, label="Forgetting factor")

class RLS_linear(Estimator):
    def __init__(self, n=4, gamma=1e8, forgetting_base=0.995, N0=1):
        self.n = 3 + n
        self.d = 3
        super().__init__(self.n, self.d)

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

        self.setTitle("Recursive Least Squares -- with IMU")

    def log(self):
        super().log()
        self.K_h.append(self.K.copy())
        self.e_h.append(self.e)
        self.lam_h.append(self.lam)

    def update(self):
        super().update()

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
