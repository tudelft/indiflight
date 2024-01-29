import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, lfilter, lfilter_zi
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec
import logging
from copy import deepcopy

plt.rcParams.update({
    "text.usetex": True,
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 12,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 12,
    "axes.titlesize": 16,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.loc": 'best',
    'figure.subplot.bottom': 0.025,
    'figure.subplot.left': 0.025,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.925,
    'figure.subplot.hspace': 0.2,
    'figure.subplot.wspace': 0.25,
})

class Signal(object):
    def __init__(self, time, signal):
        # multivariate time series with
        # 1. signal
        # 2. signal diff order n
        # 3. signal derivative order n
        # 4. Hi-Lo filtered versions of the above
        # bonus points: cache implementation

        # IMPORTANT: all filtering/diff/derivative are done causally!
        # this implies an n sample delay for n-order diff/derivative

        # get inputs and validate
        self.t = np.array(time, dtype=float).squeeze()
        if self.t.ndim > 1:
            raise ValueError("Time input must be 1 dimensional")
        self.dt = np.zeros_like(self.t, dtype=float)
        self.dt[:-1] = np.diff(self.t, n=1)
        self.dt[-1] = self.dt[-2]
        self.fs = 1. / np.mean(self.dt)

        self.y = np.array(signal, dtype=float).squeeze()
        if (self.y.ndim == 1) and (self.y.size != self.t.size):
            raise ValueError("Signal and time must be same length")

        if (self.y.ndim == 2) and (self.y.shape[0] != self.t.size):
            raise ValueError("2 dimensional signal must have as many columns as time has entries")

        if self.y.ndim > 2:
            raise NotImplementedError("only 1 or 2 dimensional signal array supported")

        # make row vector from one dimensional signal
        if self.y.ndim == 1:
            self.y = self.y[:, np.newaxis]

        self.sig = {}

    def setSignal(self, new_signal):
        newy_np = np.array(new_signal, dtype=float)
        if self.y.shape == newy_np.shape:
            self.y = newy_np
        else:
            raise ValueError("New signal has to have the same shape as existing signal")

    def diff(self, order=1):
        key = f'diff-{order}'
        if key not in self.sig.keys():
            logging.debug(f'Cache miss for key {key}')
            yDiff = np.zeros_like(self.y, dtype=float)
            yDiff[order:] = np.diff(self.y, n=order, axis=0)

            self.sig[key] = Signal(self.t, yDiff)
        else:
            logging.debug(f'Cache hit for key {key}')

        return self.sig[key]

    def dot(self, order=1):
        key = f'dot-{order}'
        if key not in self.sig.keys():
            logging.debug(f'Cache miss for key {key}')
            yDiff = np.zeros_like(self.y, dtype=float)
            yDiff[order:] = np.diff(self.y, n=order, axis=0)
            yDot = yDiff / (self.dt[:, np.newaxis]**order)

            self.sig[key] = Signal(self.t, yDot)
        else:
            logging.debug(f'Cache hit for key {key}')

        return self.sig[key]

    def filter(self, type, order, cutoff_hz):
        if type not in ['lowpass', 'highpass', 'bandpass']:
            raise ValueError("If filter_type is given, it must be either 'lowpass', 'highpass', 'bandpass'")

        # check cutoff argument for bandpass
        if type == 'bandpass':
            try:
                if len(cutoff_hz) != 2:
                    raise ValueError("cutoff_Hz must be length-2 list for type='bandpass'")
                cutoff_hz = list(cutoff_hz)
            except TypeError:
                raise TypeError("cutoff_Hz must be length-2 list for type='bandpass'")

        # check if not yet cached and compute values
        key = f'{type}-{order}-{cutoff_hz}'
        if key not in self.sig.keys():
            logging.debug(f'Cache miss for key {key}')
            # design filter and initial condition
            if order == 1:
                # pure lowpass with single zero
                #b, a = butter(order, cutoff_hz, btype=type, fs=self.fs, output='ba')
                #b[0] += b[1]
                #b[1] = 0
                alpha = 2*np.pi / (self.fs/cutoff_hz + 2*np.pi)
                b = [alpha, 0]
                a = [1., -(1. - alpha)]
                zi = (self.y[0]*lfilter_zi(b, a))[np.newaxis]

                yFilt, _ = lfilter(b, a, self.y, axis=0, zi=zi)
            else:
                # second order and up are implemented as maximally flat butterworth
                sos = butter(order, cutoff_hz, type, fs=self.fs, output='sos')
                zi = self.y[0] * sosfilt_zi(sos)[:, :, np.newaxis]

                yFilt, _ = sosfilt(sos, self.y, axis=0, zi=zi)

            # perform filtering and add to hashmap
            self.sig[key] = Signal(self.t, yFilt)
        else:
            logging.debug(f'Cache hit for key {key}')

        return self.sig[key]

class RLS(object):
    def __init__(self, n, d=1, gamma=1e8, forgetting=0.995):
        self.n = n
        self.d = d
        self.x = np.zeros((n, 1))
        self.P = gamma * np.eye(n)
        self.forgetting = forgetting

        self.name = "Recursive Least Squares"
        self.parNames = ["$\\theta$"] if n == 1 else [f"$\\theta_{{{i}}}$" for i in range(n)]
        self.yNames = ["$y$"] if d == 1 else [f"$y_{{{i}}}$" for i in range(n)]
        self.regNames = [[f"$A_{{{i},{j}}}$" for j in range(n)] for i in range(d)]

        self.N = 0
        self.A_hist = []
        self.y_hist = []
        self.K_hist = []
        self.e_hist = []
        self.x_hist = []
        self.P_hist = []

    def setName(self, name):
        self.name = name

    def setRegressorNames(self, regNames):
        if len(regNames) != self.d:
            raise ValueError(f"regNames has to be length {self.d}, got {len(regNames)}")
        for i, row in enumerate(regNames):
            if len(row) != self.n:
                raise ValueError(f"All rows in regNames have to be length {self.n}, got {len(row)} in row {i}")
        self.regNames = regNames

    def setParameterNames(self, parNames):
        if len(parNames) != self.n:
            raise ValueError(f"parNames has to be length {self.n}, got {len(parNames)}")
        self.parNames = parNames

    def setOutputNames(self, yNames):
        if len(yNames) != self.d:
            raise ValueError(f"yNames has to be length {self.d}, got {len(yNames)}")
        self.yNames = yNames

    def newSample(self, A, y, disable=None):
        # accept one-dimensional only if either n or d are 1
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

        # accept output as singleton, one-dim array or shape(d,1)
        y = np.array(y)
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

        # regressors a and observation y
        lam = self.forgetting
        K = ( self.P @ A.T ) @ np.linalg.inv( lam * np.eye(self.d) + A @ self.P @ A.T )
        self.P = (1. / lam) * (self.P - K @ A @ self.P)

        e = y - A @ self.x
        if (disable==None):
            self.x += K @ e
        else:
            self.x[~disable] += K[~disable, ~disable] @ e[~disable]

        self.N += 1
        self.A_hist.append(A.copy())
        self.y_hist.append(y.copy())
        self.K_hist.append(K.copy())
        self.e_hist.append(e.copy())
        self.x_hist.append(self.x.copy())
        self.P_hist.append(self.P.copy())

    def predictNew(self, A):
        return A @ self.x

    def predictRealTime(self):
        A = np.array(self.A_hist)
        x = np.array(self.x_hist)
        return np.array([A[i] @ x[i] for i in range(self.N)])

    def plotParameters(self, parGroups=None, yGroups=None, timeMs=None, sharey=True, zoomy=False):
        # parameters and variances
        if parGroups is None:
            parGroups = [[i] for i in range(self.n)]

        if yGroups is None:
            yGroups = [[i] for i in range(self.d)]

        if timeMs is None:
            timeMs = list(range(self.N))
            timeLabel = "iterations"
        else:
            timeLabel = "Time [ms]"

        f = plt.figure()

        left=0.04
        bottom=0.06
        right=0.975
        top=0.925
        hspace=0.15
        wspace=0.25
        rWidth = (right - left + 0*0.3*wspace) * 1 / (1 + len(parGroups)) + left
        rHeight = (top - bottom) * 2 / (2 + len(yGroups)) + bottom

        faceGs = GridSpec(2, 2,
                           width_ratios=(rWidth, 1 - rWidth),
                           height_ratios=(rHeight, 1 - rHeight),
                           )
        faceGs.update(left=0., bottom=0., right=1.0, top=0.95, hspace=0, wspace=0)

        outerGs = GridSpec(2, 2,
                           width_ratios=(1, len(parGroups)),
                           height_ratios=(2, len(yGroups)),
                           )
        outerGs.update(left=left, bottom=bottom, right=right, top=top, hspace=hspace, wspace=wspace)

        colors = ['white', 'gray', 'blue', 'green']
        for i, col in enumerate(colors):
            grayAx = f.add_subplot(faceGs[i])
            grayAx.grid(False)
            grayAx.set_facecolor(col)
            grayAx.patch.set_alpha(0.3)
            grayAx.tick_params(axis='both',which='both',bottom=0,left=0,
                              labelbottom=0, labelleft=0)

        parGs = outerGs[0, 1].subgridspec(2, len(parGroups))
        regGs = outerGs[1, 1].subgridspec(len(yGroups), len(parGroups))
        yGs   = outerGs[1, 0].subgridspec(len(yGroups), 1)

        parAxs = []
        varAxs = []
        yAxs = []
        regAxs = []
        for i in range(len(parGroups)):
            parAx = f.add_subplot(parGs[0, i]); parAxs.append(parAx)
            parAx.set_ylabel("Parameter(s)")

            varAx = f.add_subplot(parGs[1, i]); varAxs.append(varAx)
            varAx.set_ylabel("Variance(s)")
            varAx.sharex(parAxs[0])
            varAx.set_yscale('log')

            if i > 0:
                varAx.sharey(varAxs[0])
                parAx.sharex(parAxs[0])

        for i in range(len(yGroups)):
            yAx = f.add_subplot(yGs[i, 0]); yAxs.append(yAx)
            yAx.sharex(parAxs[0])

            regAxsRow = []
            for j in range(len(parGroups)):
                regAx = f.add_subplot(regGs[i, j]); regAxsRow.append(regAx)
                regAx.set_ylabel("Regressor(s)")
                regAx.sharex(parAxs[0])
                if (i > 0) and sharey:
                    regAx.sharey(regAxs[0][j])
            regAxs.append(regAxsRow)

        x = np.array(self.x_hist)
        P = np.array(self.P_hist)
        A = np.array(self.A_hist)
        y = np.array(self.y_hist)

        for parIdxs, parAx, varAx in zip(parGroups, parAxs, varAxs):
            maxy = 0.
            miny = 0.
            for i in parIdxs:
                maxy = max(maxy, x[-1, i])
                miny = min(miny, x[-1, i])
                parAx.plot(timeMs, x[:, i], label=self.parNames[i])
                varAx.plot(timeMs, P[:, i, i], label=f"var({self.parNames[i]})")
            if zoomy:
                diffy = maxy - miny
                maxy += diffy * 1.
                miny -= diffy * 1.
                parAx.set_ylim(bottom=miny, top=maxy)
            parAx.legend()
            varAx.legend()

        yLastTheta = self.predictNew(A)
        yRealTime = self.predictRealTime()
        printLegend = True
        for yIdxs, yAx in zip(yGroups, yAxs):
            for i in yIdxs:
                yAx.plot(timeMs, y[:, i], label="Target")
                yAx.plot(timeMs, yLastTheta[:, i], label="A posteriori")
                yAx.plot(timeMs, yRealTime[:, i], label="Real Time")
            yAx.set_ylabel("Output "+self.yNames[i])
            if printLegend:
                legend_ypos = 0.38 / yAx.get_position().height #FIXME: this doesnt work
                yAx.legend(loc='upper center', bbox_to_anchor=(0.5, legend_ypos))
                printLegend = False

        for yIdxs, regAxRow in zip(yGroups, regAxs):
            for parIdxs, regAx in zip(parGroups, regAxRow):
                for i in yIdxs:
                    for j in parIdxs:
                        regAx.plot(timeMs, A[:, i, j], label=self.regNames[i][j])
                regAx.legend()

        f.suptitle(f"{self.name} -- Regressors, Parameters and Variance", fontsize=18)

        yAxs[-1].set_xlabel(timeLabel)
        for regAx in regAxs[-1]:
            regAx.set_xlabel(timeLabel)

        return f

    def plotGains(self):
        # k and e
        raise NotImplementedError()


def imuOffsetCorrection(a, w, dw, r):
    # r is vector from CoG to IMU location, in FRD body coordintaes
    # f = a - ( dw x r ) - ( w x (w x r) ) 
    N = len(a)
    f = a
    f -= np.array([np.cross(dw[i], r) for i in range(N)])
    wxr = np.array([np.cross(w[i], r) for i in range(N)])
    f -= np.array([np.cross(w[i], wxr[i]) for i in range(N)])
    return f



if __name__=="__main__":
    logging.basicConfig(level=logging.DEBUG)

    a = Signal([0,0.01,0.02,0.03], [4,4,10,4])
    a.filter('lowpass', 1, 10).y
    a.filter('lowpass', 2, 10).y
    a.filter('lowpass', 3, 10).y
    a.filter('lowpass', 4, 10).y
    a.dot().y

    b = Signal([0,0.01,0.02,0.03], np.array([[4,4,10,4], [1, 2, 3, 4]]).T)
    b.filter('lowpass', 1, 10).y
    b.filter('lowpass', 2, 10).y
    b.filter('lowpass', 3, 10).y
    b.filter('lowpass', 4, 10).y
