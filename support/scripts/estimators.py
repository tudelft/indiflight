import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, lfilter, lfilter_zi
import logging

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
    def __init__(self, n, gamma=1e8, forgetting=0.995):
        self.x = np.zeros((n,1))
        self.P = gamma * np.eye(len(self.x))
        self.forgetting = forgetting
        self.a_hist = []
        self.y_hist = []
        self.k_hist = []
        self.e_hist = []
        self.x_hist = []
        self.P_hist = []

    def newSample(self, a, y, disable=None):
        # regressors a and observation y
        k = ( self.P @ a ) / ( self.forgetting + a.T @ self.P @ a )
        self.P = (1/self.forgetting) * (self.P - k @ a.T @ self.P)

        e = y - self.x.T @ a
        if (disable==None):
            self.x += k * e
        else:
            self.x[~disable] += k[~disable] * e

        self.a_hist.append(a.squeeze().copy())
        self.y_hist.append(y.squeeze().copy())
        self.k_hist.append(k.squeeze().copy())
        self.e_hist.append(e.squeeze().copy())
        self.x_hist.append(self.x.squeeze().copy())
        self.P_hist.append(self.P.copy())

    def plotParameters(self):
        # parameters and variances
        raise NotImplementedError()

    def plotRegressors(self):
        # regressors and target
        raise NotImplementedError()

    def plotGains(self):
        # k and e
        raise NotImplementedError()


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
