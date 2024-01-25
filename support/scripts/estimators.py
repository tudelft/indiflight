import numpy as np

class Signal(object):
    def __init__(self):
        # multivariate time series with
        # 1. signal
        # 2. signal diff order n
        # 3. signal derivative order n
        # 4. Hi-Lo filtered versions of the above
        # bonus points: cache implementation
        raise NotImplementedError()

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

