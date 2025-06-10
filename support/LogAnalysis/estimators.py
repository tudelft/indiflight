import numpy as np
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import time

local_rc = plt.rcParams.copy()
local_rc.update({
#    "text.usetex": True,
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 10,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 10,
    "axes.titlesize": 14,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'best',
    'figure.subplot.bottom': 0.025,
    'figure.subplot.left': 0.025,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.925,
    'figure.subplot.hspace': 0.2,
    'figure.subplot.wspace': 0.25,
})
plt.rcParams.update(local_rc)

COLORS = plt.rcParams['axes.prop_cycle'].by_key()['color']


class BlittedCursor(object):
    def __init__(self, axes, canvas, sharex=True):
        self.axes = axes
        self.canvas = canvas
        self.backgrounds = []
        self.cursors = []

        ax0 = self.axes[0]

        for i, ax in enumerate(self.axes):
            if i > 0 and sharex:
                ax.sharex(ax0)

            lb, ub = ax.get_xlim()
            self.cursors.append(ax.axvline(x=lb,
                                           color='black',
                                           linestyle='--',
                                           lw=0.8,
                                           visible=False))

        self.canvas.mpl_connect('draw_event', self._on_draw)
        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)

    def _on_draw(self, event):
        self.backgrounds = [self.canvas.copy_from_bbox(ax.bbox) for ax in self.axes]

    def _on_mouse_move(self, event):
        if event.xdata is None or not self.backgrounds:
            return

        for ax, line, bg in zip(self.axes, self.cursors, self.backgrounds):
            self.canvas.restore_region(bg)
            line.set_xdata([event.xdata])
            line.set_visible(True)
            ax.draw_artist(line)
            self.canvas.blit(ax.bbox)


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


class AttitudePlotter(object):
    def __init__(self, data, name="AttitudePlotter", follow=False):
        self.data = data
        self.name = name
        self.follow = follow
        self.fig = plt.figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Quadrotor geometry (in local frame)
        l = 0.2
        self.arms = np.array([
            [ -l, +l, 0.],
            [ +l, +l, 0.],
            [ -l, -l, 0.],
            [ +l, -l, 0.],
        ])
        self.front = np.array([
            [    l,     0., -0.3*l],
            [    l, -0.4*l,     0.],
            [1.7*l,     0.,     0.],
            [    l, +0.4*l,     0.],
            [    l,     0., -0.3*l],
            [1.7*l,     0.,     0.],
        ])

        # preprocess data
        self.t = self.data['timeS'].to_numpy()

        if "pos[0]" not in self.data.columns and not self.follow:
            raise ValueError("Data must contain 'pos' column for non-follow mode.")

        # interpolators
        self.series = {
            "quat": {"raw": self.data[[f"quat[{i}]" for i in [1,2,3,0]]].to_numpy()    , "style": "solid",  "color": COLORS[0], "marker": None, "width": 1.5, "label": "Estimate"},
            "quatSp": {"raw": self.data[[f"quatSp[{i}]" for i in [1,2,3,0]]].to_numpy(), "style": "dashed", "color": COLORS[0], "marker": None, "width": 1.0,  "label": "Setpoint"},
        }

        if not self.follow:
            self.series.update({
                "pos": {"raw": self.data[[f"pos[{i}]" for i in range(3)]].to_numpy()       , "style": "solid",  "color": COLORS[1], "marker": "o", "width": 1.5, "label": "Estimate"},
                "posSp": {"raw": self.data[[f"posSp[{i}]" for i in range(3)]].to_numpy()   , "style": "solid", "color": COLORS[1], "marker": ".", "width": 2.0,  "label": "Setpoint"},
                "vel": {"raw": self.data[[f"vel[{i}]" for i in range(3)]].to_numpy()       , "style": "solid",  "color": COLORS[2], "marker": None, "width": 1.5, "label": "Estimate"},
                "velSp": {"raw": self.data[[f"velSp[{i}]" for i in range(3)]].to_numpy()   , "style": "dashed", "color": COLORS[2], "marker": None, "width": 1.0,  "label": "Setpoint"},
                "accSp": {"raw": self.data[[f"accSp[{i}]" for i in range(3)]].to_numpy()   , "style": "dashed", "color": COLORS[3], "marker": None, "width": 1.0,  "label": "Setpoint"},
            })

        for key, value in self.series.items():
            self.series[key]['interpolator'] = interp1d(
                self.t,
                value["raw"].T,
                kind="nearest",
                bounds_error=False,
                fill_value=(value["raw"][0], value["raw"][-1]))

            self.series[key]['line'] = self.ax.plot(
                [np.nan], [np.nan], [np.nan],
                linestyle=value['style'],
                color=value['color'],
                lw=value['width'],
                label=value["label"])[0]

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # set view angle
        self.ax.view_init(elev=-25, azim=150, roll=180)

        # set limits
        if self.follow:
            minx, miny, minz = 0, 0, 0
            maxx, maxy, maxz = 0, 0, 0
        else:
            minx, miny, minz = np.min(self.series["pos"]["raw"], axis=0)
            maxx, maxy, maxz = np.max(self.series["pos"]["raw"], axis=0)

        self.ax.set_xlim(minx-0.5, maxx+0.5)
        self.ax.set_ylim(miny-0.5, maxy+0.5)
        self.ax.set_zlim(minz-0.5, maxz+0.5)

        self.ax.legend(loc='best')

        self.ax.set_title(self.name)

        self.fig.show()

    def update(self, event):
        if event.xdata is None:
            return

        for ser in self.series.keys():
            try:
                self.series[ser]['line'].remove()
            except ValueError:
                pass

        # get interpolates
        interpolates = {}
        for ser in self.series.keys():
            interpolates[ser] = self.series[ser]['interpolator'](event.xdata)

        for ser in ["quat", "quatSp"]:
            rotation = R.from_quat(interpolates[ser])
            rotated_front = rotation.apply(self.front)

            # Plot circles (representing rotors)
            xs = np.array([])
            ys = np.array([])
            zs = np.array([])
            for arm in self.arms:
                u = np.linspace(0, 2*np.pi, 20)
                x = arm[0] + 0.1 * np.cos(u)
                y = arm[1] + 0.1 * np.sin(u)
                z = np.ones_like(x) * arm[2]
                x, y, z = rotation.apply(np.array([x,y,z]).T).T
                xs = np.concatenate((xs, x, np.array([np.nan])))
                ys = np.concatenate((ys, y, np.array([np.nan])))
                zs = np.concatenate((zs, z, np.array([np.nan])))

            # plot front triangle
            xs = np.concatenate((xs, rotated_front[:, 0]))
            ys = np.concatenate((ys, rotated_front[:, 1]))
            zs = np.concatenate((zs, rotated_front[:, 2]))

            if not self.follow:
                xs += interpolates["pos"][0]
                ys += interpolates["pos"][1]
                zs += interpolates["pos"][2]

            self.series[ser]['line'] = self.ax.plot(xs, ys, zs,
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'])[0]

        if not self.follow:
            for ser in ["pos", "posSp"]:
                self.series[ser]['line'] = self.ax.scatter(
                    interpolates[ser][0],
                    interpolates[ser][1],
                    interpolates[ser][2],
                    linestyle=self.series[ser]['style'],
                    color=self.series[ser]['color'],
                    lw=self.series[ser]['width'],
                    marker=self.series[ser]['marker'],
                    facecolor='none',
                    s=50)

            for ser in ["vel", "velSp"]:
                self.series[ser]['line'] = self.ax.plot(
                    [interpolates['pos'][0], 0.2*interpolates[ser][0] + interpolates['pos'][0]],
                    [interpolates['pos'][1], 0.2*interpolates[ser][1] + interpolates['pos'][1]],
                    [interpolates['pos'][2], 0.2*interpolates[ser][2] + interpolates['pos'][2]],
                    linestyle=self.series[ser]['style'],
                    color=self.series[ser]['color'],
                    lw=self.series[ser]['width'])[0]

            self.series["accSp"]['line'] = self.ax.plot(
                [interpolates['pos'][0], 0.1*interpolates['accSp'][0] + interpolates['pos'][0]],
                [interpolates['pos'][1], 0.1*interpolates['accSp'][1] + interpolates['pos'][1]],
                [interpolates['pos'][2], 0.1*interpolates['accSp'][2] + interpolates['pos'][2]],
                linestyle=self.series["accSp"]['style'],
                color=self.series["accSp"]['color'],
                lw=self.series["accSp"]['width'])[0]

        # make sure all the axes are equal
        self.ax.set_aspect('equal', adjustable='box')

        self.fig.canvas.draw()


class FlightPlotter(object):
    def __init__(self, data, name="FlightPlotter"):
        self.data = data
        self.name = name
        self.all_axes = []

        self.fig = plt.figure(figsize=(12, 8))
        self.gs = GridSpec(nrows=3, ncols=3,
                      width_ratios=[1, 2, 2],
                      height_ratios=[1, 1, 1])

        # preprocess data
        self.t = self.data['timeS'].to_numpy()

        # go
        self._populate()
        self._dress()

        self.curser = BlittedCursor(self.all_axes, self.fig.canvas, sharex=True)

        self.fig.show()

    def add_callback(self, event_type, callback):
        # e,g, motion_notify_event
        self.fig.canvas.mpl_connect(event_type, callback)

    def _populate(self):
        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 1]),
                         solid=[self.data[f'gyroADCafterRpm[{i}]'].to_numpy() for i in range(3)],
                         light=[self.data[f'gyroSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["roll", "pitch", "yaw"],
                         style_labels=["Raw", "Setpoint", None],
                         title="Body Angular Rates",
                         ylabel="Angular Rate [rad/s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 2]),
                         solid=[self.data[f'accADCafterRpm[{i}]'].to_numpy() for i in range(3)],
                         light=[self.data[f'spfSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["X", "Y", "Z"],
                         style_labels=["Raw", "Setpoint", None],
                         title="Body Accelerations",
                         ylabel="Acceleration [m/s²]")

    def _plot_timeseries(self, ax, solid=[], light=[], dashed=[], series_labels=[], style_labels=[None, None, None], title="", ylabel=""):
        if len(solid) == 0:
            raise ValueError("At least one solid series must be provided.")
        if len(series_labels) != len(solid):
            raise ValueError("series_labels must have the same length as solid series.")
        if len(style_labels) != 3:
            raise ValueError("style_labels must have exactly 3 entries. Set to None if not needed.")
        if len(light) == 0:
            light = [None] * len(solid)
        if len(dashed) == 0:
            dashed = [None] * len(solid)
        lengths = np.array([len(solid), len(light), len(dashed), len(series_labels)])
        if not (lengths == lengths[0]).all():
            raise ValueError("solid, light, dashed and labels must all have the same length if given.")

        for i, series in enumerate(solid):
            ax.plot(self.t, series, label=series_labels[i], color=COLORS[i], lw=1.5, linestyle='-')

        for i, series in enumerate(light):
            if series is not None:
                ax.plot(self.t, series, color=COLORS[i], alpha=0.5, lw=1.0, linestyle='-')

        for i, series in enumerate(dashed):
            if series is not None:
                ax.plot(self.t, series, color=COLORS[i], lw=1.0, linestyle='--')

        self.all_axes.append(ax)
        ax.set_title(title)
        ax.set_ylabel(ylabel)

        ax.add_artist(ax.legend(loc='upper left'))
        ax.add_artist(self._generate_style_legend(ax, style_labels))

    def _generate_style_legend(self, ax, labels):
        linestyles = []
        if labels[0] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=1.0, lw=1.5, linestyle='-', label=labels[0]))
        if labels[1] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=0.5, lw=1.0, linestyle='-', label=labels[1]))
        if labels[2] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=1.0, lw=1.0, linestyle='--', label=labels[2]))

        leg_styles = ax.legend(handles=linestyles, title='Line Styles', loc='lower left')

        return leg_styles

    def _dress(self):
        for ax in self.all_axes:
            ax.grid(True)
            ax.set_xlabel("Time [s]")
            # if ax.get_subplotspec().is_last_row():
            #     ax.set_xlabel("Time [s]")
            # else:
            #     ax.xaxis.set_ticklabels([])

        # self.fig.tight_layout()
        # self.fig.subplots_adjust(hspace=0.3, wspace=0.3)

        self.fig.suptitle(self.name)
