import numpy as np

class RLS(object):
    def __init__(self, n, gamma=1e8, forgetting=0.995):
        self.x = np.zeros((n,1))
        self.P = gamma * np.eye(len(self.x))
        self.forgetting = forgetting
        self.k_hist = []
        self.e_hist = []
        self.x_hist = []
        self.P_hist = []

    def newData(self, a, y, disable=None):
        # regressors a and observation y
        k = ( self.P @ a ) / ( self.forgetting + a.T @ self.P @ a )
        self.P = (1/self.forgetting) * (self.P - k @ a.T @ self.P)

        e = y - self.x.T @ a
        if (disable==None):
            self.x += k * e
        else:
            self.x[~disable] += k[~disable] * e

        self.k_hist.append(k)
        self.e_hist.append(e)
        self.x_hist.append(self.x)
        self.P_hist.append(self.P)
