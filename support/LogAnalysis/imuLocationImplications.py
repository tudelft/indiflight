import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "simulation"))

from PyNDIflight.crafts import MultiRotor, IMU

import numpy as np

#%% Generate craft
# Hexarotor matching configs/profiles/HexDemo_post_fix.txt.
#
# The firmware only knows the *specific* effectiveness G1 (per unit motor
# command, i.e. per Tmax), so the absolute mass/size below is a plausible
# choice and everything else is derived such that the sim reproduces the
# profile's G1/G2 (within the ~1.5% noise of the learned profile values):
#
#   indi_act_g1_fz    / 100  =  -Tmax / m                     [N/kg]
#   indi_act_g1_roll  / 10   =  -y_i * Tmax / Ixx             [rad/s/s]
#   indi_act_g1_pitch / 10   =  +x_i * Tmax / Iyy             [rad/s/s]
#   indi_act_g1_yaw   / 10   =  +cm_i * Tmax / Izz            [rad/s/s]
#   indi_act_g2_yaw   * 1e-5 =  Irotor / Izz                  [-]
#
# Geometry from the signs of g1_roll/g1_pitch (FRD body frame):
# motor 0 points forward and the indices go counter-clockwise seen from
# above. Spin directions alternate (sign of g1_yaw, cm<0 is CW from above):
#
#             x (nose)
#             ^
#             0 (CW)
#    1 (CCW)  |  5 (CCW)
#             +-----> y (right)
#    2 (CW)      4 (CW)
#             3 (CCW)
#
# Note that the MOCKUP target does not apply motor_output_reordering, so
# rotor i here is INDI actuator i of the profile.
G1_FZ    = 7.00                      # N/kg,   -indi_act_g1_fz / 100
G1_ROLL  = 500.   # rad/s/s, roll effectiveness of a motor at lever arm R; |g1_roll| / 10 of the 60deg arms is R*sin(60deg)
G1_PITCH = 500.                     # rad/s/s, average of |g1_pitch| / 10 (half-arms scaled by 2)
G1_YAW   = 20.                     # rad/s/s, |g1_yaw| / 10
G2_YAW   = 1e-4                      # -,       |g2_yaw| * 1e-5
OMEGA_MAX = 40000. / 60. * 2.*np.pi  # rad/s,   indi_act_max_rpm (also hardcoded in the learner via LEARNER_HARDCODE_MAXRPM)
TAU = 0.025                          # s,       indi_act_time_constant_ms
K_ESC = 0.5                          # -,       indi_act_nonlinearity / 100

m = 0.6                              # kg, assumed
R = 0.1                              # m,  assumed arm length (center to motor)
Tmax = G1_FZ * m                     # N per motor
Ixx = Tmax * R / G1_ROLL
Iyy = Tmax * R / G1_PITCH
Izz = Ixx + Iyy                      # thin-plate assumption
cm = G1_YAW * Izz / Tmax             # m, drag-torque-to-thrust ratio
Irotor = G2_YAW * Izz
k = Tmax / OMEGA_MAX**2

# NOTE: the profile's indi_act_hover_rpm (21600) is not consistent with
# g1_fz = -700 and max_rpm = 37000: with these numbers the sim hovers at
# ~17900 rpm. Hovering at 21600 rpm would require G1_FZ ~= 4.8 N/kg. The
# hover rpm only seeds the INDI linearization (rpm feedback is enabled),
# so the sim follows G1 instead.

def make_hex(tilt_deg=0.):
    """Hexarotor above. tilt_deg tilts all rotor axes forward (about body y),
    so that the most efficient hover direction is no longer body -z."""
    mc = MultiRotor(Nr=6)
    mc.setInertia(m=m, I=np.diag([Ixx, Iyy, Izz]))
    spin = [-1., +1., -1., +1., -1., +1.]  # sign of cm, from sign of indi_act_g1_yaw
    t = np.radians(tilt_deg)
    for i in range(6):
        psi = -i * np.pi / 3.  # 0 front, then counter-clockwise seen from above
        mc.setRotor(i, X=[R*np.cos(psi), R*np.sin(psi), 0.], ax=[-np.sin(t), 0., -np.cos(t)],
                    k=k, cm=spin[i]*cm, wmax=OMEGA_MAX, tau=TAU, kESC=K_ESC, I=Irotor)
    return mc

mc = make_hex()
G1, G2, G2_scaler = mc.calculateG1G2()
E1 = G1 / OMEGA_MAX / OMEGA_MAX


#%% ===================== IMU lever arm investigation =====================
#
# Notation (G1 units, regressor x = omega^2 / omegaMax^2, so E1 = G1/omegaMax^2):
#   Bf  = G1[:3]  specific force at the CoG per x           (3 x N)
#   Br  = G1[3:]  angular acceleration per x                (3 x N)
#   accelerometer at r:   a_imu = f_cg + dOmega x r + Omega x (Omega x r)
#                               = f_cg - [r]x dOmega + Omega x (Omega x r)
#   so with Omega = 0 the identified force rows are
#       Bf_imu = Bf - [r]x Br                                          (*)
#
# Split actuator space R^N = range(Br^T) (+) null(Br):  Pr (N x 3), Nr (N x N-3).
# (*) gives  Bf_imu Nr = Bf Nr  EXACTLY, the corruption only lives in Bf Pr.

import matplotlib
import matplotlib.pyplot as plt
from scipy.constants import g as GRAVITY

np.set_printoptions(precision=3, suppress=True, linewidth=160)
FIGDIR = os.environ.get("IMULOC_FIGDIR", None)
DT = 1. / 8000.

def skew(v):
    return np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])

def rotSplit(Br):
    """orthonormal bases of range(Br^T) (Pr) and null(Br) (Nr), like learner.c"""
    Q, _ = np.linalg.qr(Br.T, 'complete')
    return Q[:, :3], Q[:, 3:]

def hoverFromNullspaceMap(M, Nr):
    """learner.c:calculateHoverAttitude steps 3-7, given M = Bf @ Nr"""
    w, V = np.linalg.eigh(M.T @ M)
    v = V[:, -1] * GRAVITY / np.sqrt(w[-1])
    if (Nr @ v).sum() < 0.:
        v = -v
    t = M @ v
    return t / np.linalg.norm(t), Nr @ v

def hoverDir(Bf, Br):
    _, Nr = rotSplit(Br)
    return hoverFromNullspaceMap(Bf @ Nr, Nr)

def angDeg(a, b):
    return np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1., 1.)))

def uFromX(x, kESC=K_ESC):
    """invert the steady-state ESC curve x = kESC u^2 + (1-kESC) u"""
    x = np.clip(x, 0., 1.)
    return (-(1. - kESC) + np.sqrt((1. - kESC)**2 + 4.*kESC*x)) / (2.*kESC)

crafts = {"hex": make_hex(0.), "hex tilted 15deg": make_hex(15.)}


#%% A) analytical: sensitivity of Bf_imu to the lever arm, and the hover direction
print("\n===== A) analytical, Omega = 0 =====")
for name, c in crafts.items():
    G1c, _, _ = c.calculateG1G2()
    Bf, Br = G1c[:3], G1c[3:]
    tTrue, _ = hoverDir(Bf, Br)
    print(f"\n--- {name}: true hover thrust direction {tTrue}")
    for r in ([0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]):
        r = np.array(r)
        BfImu = Bf - skew(r) @ Br
        tImu, _ = hoverDir(BfImu, Br)
        _, Nr = rotSplit(Br)
        # print(f"r = {r*100} cm: Bf_imu - Bf =\n{BfImu - Bf}")
        print(f"r = {r*100} cm: Bf_imu =\n{BfImu}")
        print(f"   |Bf_imu Nr - Bf Nr| = {np.abs((BfImu - Bf) @ Nr).max():.1e},"
              f"  hover dir error {angDeg(tImu, tTrue):.2e} deg")

# Result: 1 cm moves entries of Bf by ~5 N/kg per unit x, compared to fz = 7 N/kg.
# Yet the nullspace map Bf Nr, and with that the hover direction, is untouched.
# So *with Omega = 0 and a linear map*, calculateHoverAttitude is immune to r.


#%% simulation helpers
def simulate(c, imuR, T, uFun, Omega0=(0., 0., 0.), accStd=0., accBias=(0., 0., 0.), gyroStd=0., seed=0):
    """open loop or closed loop sim of the bare craft in free fall (no ground)."""
    np.random.seed(seed)
    c.setPose(x=[0., 0., -1e4], q=[1., 0., 0., 0.])
    c.setTwist(v=[0., 0., 0.], w=list(Omega0))
    c.r_w[:] = 0.
    c.throw_time = np.inf
    imu = IMU(c, r=list(imuR), accBias=list(accBias), accStd=accStd, gyroStd=gyroStd)
    n = int(round(T / DT))
    log = {k: np.zeros((n, d)) for k, d in
           (("x", c.Nr), ("Omega", 3), ("dOmega", 3), ("acc", 3), ("gyro", 3), ("u", c.Nr))}
    for i in range(n):
        u = np.asarray(uFun(i*DT, c), dtype=np.float32)
        c.r_u[:] = np.clip(u, 0., 1.)
        c.tick(DT)
        imu.update()
        log["x"][i] = c.r_w**2 / c.r_wmax**2
        log["Omega"][i] = c.OB
        log["dOmega"][i] = c.ODotB
        log["acc"][i] = imu.acc
        log["gyro"][i] = imu.gyro
        log["u"][i] = c.r_u
    log["t"] = np.arange(n) * DT
    return log

def leverArmCorrect(log, rDot, rCent):
    """learner.c fxSpfCorrected: remove dOmega x r and Omega x (Omega x r).
    rDot / rCent: lever arm assumed for the tangential / centripetal term"""
    W, dW = log["Omega"], log["dOmega"]
    return log["acc"] - np.cross(dW, rDot) - np.cross(W, np.cross(W, rCent))

def fwQuery(nAct, stepMs=50, rampMs=100, overlapMs=50, stepAmp=0.35, rampAmp=0.7, delayMs=0.):
    """learner.c LEARNING_QUERY_ACTIVE motor sequence (without the gyro cutoff)"""
    period = (stepMs + rampMs - overlapMs) * 1e-3
    def u(t, c):
        out = np.zeros(nAct)
        for i in range(nAct):
            ti = t - delayMs*1e-3 - i*period
            if 0. <= ti < stepMs*1e-3:
                out[i] = stepAmp
            elif stepMs*1e-3 <= ti < (stepMs + rampMs)*1e-3:
                out[i] = rampAmp * (1. - (ti - stepMs*1e-3) / (rampMs*1e-3))
        return out
    T = delayMs*1e-3 + (nAct - 1)*period + (stepMs + rampMs)*1e-3 + 0.02
    return u, T

def quadRates(W):
    """the 6 monomials of Omega x (I Omega) for a general inertia"""
    return np.column_stack((W[:, 0]**2, W[:, 1]**2, W[:, 2]**2,
                            W[:, 0]*W[:, 1], W[:, 0]*W[:, 2], W[:, 1]*W[:, 2]))

def lowpass(Y, fc, order=2):
    """causal butterworth, applied to all signals alike (like the learner's sync filters)"""
    from scipy.signal import butter, sosfilt
    return sosfilt(butter(order, fc, fs=1./DT, output="sos"), Y, axis=0)

def regressDiff(X, Y):
    """learner.c style: regress sample-to-sample differences, dY = B dX"""
    dX, dY = np.diff(X, axis=0), np.diff(Y, axis=0)
    B, *_ = np.linalg.lstsq(dX, dY, rcond=None)
    return B.T


#%% B) firmware query in simulation, lever arm correction with an estimate rEst = rTrue + dr
print("\n===== B) firmware motor-step query (free fall, open loop, initial tumble) =====")
rTrue = np.array([0.01, -0.005, 0.015])
Omega0 = (1., -2., 1.5)
drs = {"dr = 0": [0, 0, 0], "dr = 5mm x": [5e-3, 0, 0], "dr = 5mm y": [0, 5e-3, 0],
       "dr = 5mm z": [0, 0, 5e-3], "no correction": list(-rTrue)}
resB = {}
for name, c in crafts.items():
    G1c, _, _ = c.calculateG1G2()
    tTrue, _ = hoverDir(G1c[:3], G1c[3:])
    uQ, TQ = fwQuery(c.Nr)
    log = simulate(c, rTrue, TQ, uQ, Omega0=Omega0)
    print(f"\n--- {name}: max |Omega| during query {np.abs(log['Omega']).max():.1f} rad/s,"
          f" true hover dir {tTrue}")
    BrHat = regressDiff(log["x"], log["dOmega"])
    BrHatEuler = regressDiff(np.hstack((log["x"], quadRates(log["Omega"]))), log["dOmega"])[:, :c.Nr]
    relErr = lambda B: np.abs(B - G1c[3:]).max() / np.abs(G1c[3:]).max()
    print(f"Br relative error: as in learner.c {relErr(BrHat):.2%}, "
          f"with Omega_i*Omega_j regressors (Euler terms) {relErr(BrHatEuler):.2%}")
    resB[name] = {"BrHat": BrHat, "BrHatEuler": BrHatEuler}
    for drName, dr in drs.items():
        rEst = rTrue + np.array(dr)
        out = {}
        for corr, (rDot, rCent) in {"full": (rEst, rEst), "centripetal exact": (rEst, rTrue)}.items():
            BfHat = regressDiff(log["x"], leverArmCorrect(log, rDot, rCent))
            tHat, _ = hoverDir(BfHat, BrHat)
            out[corr] = (angDeg(tHat, tTrue), np.abs(BfHat - G1c[:3]).max())
        print(f"{drName:>14}: hover err {out['full'][0]:6.2f} deg (max|dBf| {out['full'][1]:5.2f}) | "
              f"centripetal term exact: hover err {out['centripetal exact'][0]:6.2f} deg "
              f"(max|dBf| {out['centripetal exact'][1]:5.2f})")
    resB[name]["log"] = log

# Result: the tangential dOmega x dr error is large in Bf but harmless for the
# hover direction (exactly the (*) argument). What hurts is the centripetal
# Omega x (Omega x dr) term: Omega builds up to many rad/s during the open-loop
# steps, the term is quadratic in Omega and correlates with the steps.


#%% C) nullspace excitation: excite only along null(BrHat) while a rate damper
#      acts in range(BrHat^T). Omega and dOmega stay ~0, so r drops out entirely
print("\n===== C) nullspace excitation with rate damping =====")
def nullspaceQuery(BrHat, xBias=0.3, amp=0.15, Kp=15., settle=0.3, stepT=0.06):
    Pr, Nr = rotSplit(BrHat)
    BrPrInv = np.linalg.inv(BrHat @ Pr)
    xB = Nr @ (Nr.T @ (xBias * np.ones(BrHat.shape[1])))   # bias inside the nullspace
    nN = Nr.shape[1]
    seq = []
    for j in range(nN):
        seq += [(j, +amp), (j, -amp), (j, 0.)]
    def u(t, c):
        x = xB - Pr @ (BrPrInv @ (Kp * c.OB))                  # rate damping only in range(Br^T)
        if t > settle:
            idx = int((t - settle) // stepT)
            if idx < len(seq):
                j, a = seq[idx]
                x = x + a * Nr[:, j]
        return uFromX(x)
    return u, settle + len(seq)*stepT + 0.02, Pr, Nr

resC = {}
for name, c in crafts.items():
    G1c, _, _ = c.calculateG1G2()
    tTrue, _ = hoverDir(G1c[:3], G1c[3:])
    for brName in ("BrHat", "BrHatEuler"):
        BrHat = resB[name][brName]
        uN, TN, Pr, Nr = nullspaceQuery(BrHat)
        log = simulate(c, rTrue, TN, uN, Omega0=Omega0)
        sel = log["t"] > 0.25
        print(f"\n--- {name}, nullspace from {brName}: max |Omega| during excitation "
              f"{np.abs(log['Omega'][sel]).max():.3f} rad/s")
        resC[(name, brName)] = {"log": log}
        for drName, dr in drs.items():
            rEst = rTrue + np.array(dr)
            spf = leverArmCorrect(log, rEst, rEst)[sel]
            ab = log["x"][sel] @ np.hstack((Nr, Pr))             # nullspace (alpha) and range (beta) coords
            Mfull = regressDiff(ab, spf)
            M = Mfull[:, :Nr.shape[1]]                             # = Bf Nr estimate
            tHat, _ = hoverFromNullspaceMap(M, Nr)
            print(f"{drName:>14}: hover err {angDeg(tHat, tTrue):6.3f} deg, "
                  f"max|M - Bf Nr| {np.abs(M - G1c[:3] @ Nr).max():.3f}")

# Result: the remaining error is  -[dr]x Br Nr_hat: it only exists because the
# estimated nullspace is not exactly the true one (Br error from Euler terms
# during the fast open-loop tumble). With Euler regressors it essentially vanishes.


#%% D) how well can r be found from the free-fall tumble (learner.c imuRls)?
#      a_imu = dOmega x r + Omega x (Omega x r) + bias + noise, motors off
print("\n===== D) IMU location from free-fall tumble (motors off, 250 ms) =====")
def estimateR(log, withBias, fc=10.):
    W = lowpass(log["gyro"], fc)
    dW = np.gradient(W, DT, axis=0)
    acc = lowpass(log["acc"], fc)
    sel = log["t"] > 0.05                                   # filter transient
    A = np.array([skew(dw) + skew(w) @ skew(w) for w, dw in zip(W[sel], dW[sel])])  # dW x r + W x (W x r)
    if withBias:
        A = np.concatenate((A, np.broadcast_to(np.eye(3), A.shape)), axis=2)
    th, *_ = np.linalg.lstsq(A.reshape(-1, A.shape[2]), acc[sel].reshape(-1), rcond=None)
    return th[:3]

c = crafts["hex"]
for Om0 in [(1., -2., 1.5), (3., -6., 4.5), (0., 0., 8.)]:
    for bias in [(0., 0., 0.), (0.1, -0.15, 0.2)]:
        log = simulate(c, rTrue, 0.25, lambda t, c: np.zeros(c.Nr), Omega0=Om0,
                       accStd=0.3, accBias=bias, gyroStd=0.01)
        e0 = (estimateR(log, False) - rTrue) * 1e3
        e1 = (estimateR(log, True) - rTrue) * 1e3
        print(f"Omega0 {Om0}, acc bias {bias}: r error [mm] no bias est {e0}, with bias est {e1}")

# Result: see printout. Without a bias state, an accelerometer bias b maps to
# dr ~ b / |Omega|^2: 0.2 m/s^2 at 2.5 rad/s is ~3 cm. With a bias state the
# tumble must change direction (Euler terms), otherwise r and b are
# indistinguishable; a pure spin about a principal axis never identifies r_z.


#%% E) what does the controller do with a lever arm? Point p = r on the body,
#      planar pitch: a_x(p) = -g theta + r_z theta_ddot  ->  zeros of a_x/theta_ddot
import sympy as sp
s, rz, gg = sp.symbols("s r_z g", real=True)
G = sp.simplify((-gg + rz * s**2) / s**2)
print("\n===== E) a_x at the IMU / theta_ddot:", G, " zeros:", sp.solve(sp.numer(G), s))
for rzv in (0.005, 0.01, 0.03):
    print(f"   r_z = {rzv*100:.1f} cm (IMU below CoG, FRD):  zeros at +-{np.sqrt(GRAVITY/rzv):.0f} rad/s"
          f" = {np.sqrt(GRAVITY/rzv)/2/np.pi:.1f} Hz")


#%% F) low-rate per-motor excitation instead of nullspace excitation:
#      small orthogonal sines on every motor (distinct frequencies), around a
#      collective bias, with / without a rate damper acting in range(Br^T).
#      Regress the FULL Bf_imu and Br per motor, without Euler regressors.
print("\n===== F) per-motor sines, low body rates =====")
def sineQuery(nAct, BrPrior=None, xBias=0.3, amp=0.08, freqs=None, Kp=15., T=0.8):
    freqs = np.linspace(7., 17., nAct) if freqs is None else freqs
    if BrPrior is not None:
        Pr, _ = rotSplit(BrPrior)
        BrPrInv = np.linalg.inv(BrPrior @ Pr)
    def u(t, c):
        x = xBias + amp * np.sin(2.*np.pi*freqs*t)
        if BrPrior is not None:
            x = x - Pr @ (BrPrInv @ (Kp * c.OB))
        return uFromX(x)
    return u, T

for name, c in crafts.items():
    G1c, _, _ = c.calculateG1G2()
    tTrue, _ = hoverDir(G1c[:3], G1c[3:])
    for dampName, prior in (("no damper", None), ("rate damper (prior Br 5% off)", resB[name]["BrHat"])):
        uS, TS = sineQuery(c.Nr, prior)
        log = simulate(c, rTrue, TS, uS, Omega0=Omega0)
        sel = log["t"] > 0.2
        BrHat = regressDiff(log["x"][sel], log["dOmega"][sel])
        errs = []
        for drName, dr in drs.items():
            rEst = rTrue + np.array(dr)
            BfHat = regressDiff(log["x"][sel], leverArmCorrect(log, rEst, rEst)[sel])
            tHat, _ = hoverDir(BfHat, BrHat)
            errs.append(f"{drName}: {angDeg(tHat, tTrue):.3f}")
        print(f"{name:>17}, {dampName:>30}: max|Omega| {np.abs(log['Omega'][sel]).max():4.1f} rad/s, "
              f"Br err {np.abs(BrHat - G1c[3:]).max() / np.abs(G1c[3:]).max():.2%}\n      hover err [deg] "
              + ", ".join(errs))

# Result: keeping |Omega| << sqrt(|dOmega|) is all it takes. Then the full
# per-motor Bf_imu is available (incl. the Bf Pr block), and Bf_imu Nr = Bf Nr
# gives the hover direction without knowing r and without Euler regressors.


#%% G) allocation: WLS on E1_imu vs. sequential (rotation first, then force in null(Br))
#
#   Q = [Pr Nr] orthonormal,  u = Pr beta + Nr alpha
#   E Q = [ Bf Pr   Bf Nr ]      block triangular, because Br Nr = 0
#         [ Br Pr     0   ]
#   1) beta  = (Br Pr)^-1 dOmega_des            (Br Pr = R^T of the QR, 3x3)
#   2) alpha = argmin |Wf (Bf Nr alpha - (f_des - Bf Pr beta))|^2 + lam |alpha - alpha_pref|^2
#   The force sub-problem only ever sees Bf Nr (exact, r-independent): it
#   cannot buy force with rotation. Bf Pr beta is only a feed-forward of the
#   force that the rotation command makes anyway (at the IMU point if Bf_imu is used).
print("\n===== G) allocation with IMU at r = (0, 0, 1cm), motor 0 10% weak, flat hex =====")
cW = make_hex(0.)
cW.r_k[0] *= 0.9
G1w, _, _ = cW.calculateG1G2()
rG = np.array([0., 0., 0.01])
EimuG = np.vstack((G1w[:3] - skew(rG) @ G1w[3:], G1w[3:]))
vDes = np.array([0., 0., -1., 100., 0., 0.])           # 1 N/kg more thrust, 100 rad/s^2 roll

def wls(E, v, Wv, lam=1e-6):
    A = np.vstack((np.diag(Wv) @ E, np.sqrt(lam) * np.eye(E.shape[1])))
    b = np.hstack((np.diag(Wv) @ v, np.zeros(E.shape[1])))
    return np.linalg.lstsq(A, b, rcond=None)[0]

def sequential(E, v, Wf=(1., 1., 1.), lam=1e-6):
    Pr, Nr = rotSplit(E[3:])
    beta = np.linalg.solve(E[3:] @ Pr, v[3:])
    fRes = v[:3] - E[:3] @ Pr @ beta
    alpha = wls(E[:3] @ Nr, fRes, np.asarray(Wf), lam)
    return Pr @ beta + Nr @ alpha

cases = {"WLS Wv = 1,1,1,1,1,1": wls(EimuG, vDes, np.ones(6)),
         "WLS Wv = 1,1,50,50,50,5": wls(EimuG, vDes, np.array([1, 1, 50, 50, 50, 5.])),
         "sequential, Wf = 1,1,1": sequential(EimuG, vDes)}
for nm, du in cases.items():
    print(f"{nm:>26}: CoG [fx fy fz | p q r]dot = {G1w @ du},  at IMU fx,fy = {(EimuG @ du)[:2]}")


#%% H) box constraints. u = Pr beta + Nr alpha with 0 <= u <= 1 is a general
#      polytope in (alpha, beta), NOT a box (Q is a rotation, not a permutation).
#      Stay in u-space instead and get the same decoupling with a row operation:
#
#        Pr (Br Pr)^-1 Br = Pr Pr^T    (because Br = Br Pr Pr^T)
#        K  = Bf Pr (Br Pr)^-1          3x3, "force per angular accel of a min-norm rotation command"
#        E' = [Bf - K Br; Br] = T' E,   v' = T' v,   T' = [[I, -K], [0, I]]
#
#      E' has Bf' Pr = 0: its force rows only see Nr Nr^T u, so rotation can't be
#      traded for force. Same box-constrained WLS solver, only E and v change.
#      (Lever arm: K_imu = K_cg - [r]x, i.e. T(r) is the same kind of row operation.)
from scipy.optimize import lsq_linear
print("\n===== H) box-constrained WLS on E_imu vs E' = T'E_imu =====")

def decouple(E):
    Pr, _ = rotSplit(E[3:])
    K = E[:3] @ Pr @ np.linalg.inv(E[3:] @ Pr)
    T = np.eye(6); T[:3, 3:] = -K
    return T @ E, T

def boxWls(E, v, Wv, u0, Wu=1e-3):
    """min |Wv (E u - v)|^2 + Wu |u - u0|^2,  0 <= u <= 1  (absolute, not incremental, for clarity)"""
    A = np.vstack((np.diag(Wv) @ E, np.sqrt(Wu) * np.eye(E.shape[1])))
    b = np.hstack((np.diag(Wv) @ v, np.sqrt(Wu) * u0))
    return lsq_linear(A, b, bounds=(0., 1.)).x

EdG, TdG = decouple(EimuG)
Pr, Nr = rotSplit(EimuG[3:])
print(f"check Pr (BrPr)^-1 Br == Pr Pr^T: {np.allclose(Pr @ np.linalg.solve(EimuG[3:] @ Pr, EimuG[3:]), Pr @ Pr.T)}, "
      f"max|Bf' Pr| = {np.abs(EdG[:3] @ Pr).max():.1e}")
uHov = np.full(6, 0.4)
for vName, v in (("feasible: fz -4, q_dot 100", np.array([0, 0, -4., 0, 100, 0])),
                 ("rotation saturates: fz -4, q_dot 1500", np.array([0, 0, -4., 0, 1500, 0])),
                 ("lateral request at IMU: fy 2", np.array([0, 2., -4., 0, 0, 0]))):
    print(f"--- {vName}")
    for eName, E, vv in (("E_imu", EimuG, v), ("E'   ", EdG, TdG @ v)):
        for wName, Wv in (("Wv 1,1,50,50,50,5   ", np.array([1, 1, 50, 50, 50, 5.])),
                          ("Wv 100,100,1,1,1,1", np.array([100, 100, 1, 1, 1, 1.]))):
            u = boxWls(E, vv, Wv, uHov)
            print(f"   {eName} {wName}: u = {u},  CoG = {G1w @ u}")


#%% I) geometry-agnostic excitation for an unknown, asymmetric craft
#      phase 1: short, small probe pulses per motor in a bad order (left side first)
#               -> rough Br only (gyro, r-independent), accumulated rates stay low
#      phase 2: bias in null(Br_rough) + per-motor sines + rate damper with Br_rough
#               -> accurate Br and Bf_imu, hover direction from Bf_imu Nr
print("\n===== I) probe pulses -> damped sines, asymmetric craft, bad motor order =====")
def make_asym():
    c = make_hex(0.)
    c.r_X[:] -= np.array([[0.02], [0.015], [0.]], dtype=np.float32)   # CoG 2.5 cm off centre
    c.r_k[2] *= 0.85                                                    # one weak motor
    return c

def probeQuery(order, xp=0.15, tp=0.025, gap=0.015, T0=0.05):
    def u(t, c):
        x = np.zeros(c.Nr)
        k = int((t - T0) // (tp + gap)) if t > T0 else -1
        if 0 <= k < len(order) and (t - T0) - k*(tp + gap) < tp:
            x[order[k]] = xp
        return uFromX(x)
    return u, T0 + len(order)*(tp + gap) + 0.04

order = [1, 2, 3, 0, 5, 4]                 # left side (y < 0) first
for name, c in (("flat hex", make_hex(0.)), ("asymmetric hex", make_asym())):
    G1c, _, _ = c.calculateG1G2()
    tTrue, _ = hoverDir(G1c[:3], G1c[3:])
    uP, TP = probeQuery(order)
    logP = simulate(c, rTrue, TP, uP, Omega0=Omega0)
    BrRough = regressDiff(logP["x"], logP["dOmega"])
    relErr = lambda B: np.abs(B - G1c[3:]).max() / np.abs(G1c[3:]).max()
    # bias: hover allocation from the rough probe (Bf_imu Nr is r-independent), scaled to mean x = 0.3
    BfRough = regressDiff(logP["x"], logP["acc"])
    _, uH = hoverDir(BfRough, BrRough)
    xBias = 0.3 * uH / uH.mean()
    uS, TS = sineQuery(c.Nr, BrRough, xBias=xBias)
    # phase 2 continues from the end state of phase 1
    Wend = tuple(logP["Omega"][-1])
    logS = simulate(c, rTrue, TS, uS, Omega0=Wend)
    sel = logS["t"] > 0.2
    BrHat = regressDiff(logS["x"][sel], logS["dOmega"][sel])
    errs = []
    for drName, dr in drs.items():
        rEst = rTrue + np.array(dr)
        BfHat = regressDiff(logS["x"][sel], leverArmCorrect(logS, rEst, rEst)[sel])
        tHat, _ = hoverDir(BfHat, BrHat)
        errs.append(f"{drName}: {angDeg(tHat, tTrue):.3f}")
    print(f"{name}: probe max|Omega| {np.abs(logP['Omega']).max():.1f} rad/s, Br_rough err {relErr(BrRough):.1%}, "
          f"xBias {xBias}\n   phase 2 max|Omega| {np.abs(logS['Omega'][sel]).max():.2f} rad/s, Br err {relErr(BrHat):.2%}, "
          f"hover err [deg] " + ", ".join(errs))


#%% plots
fig, axs = plt.subplots(3, 2, figsize=(12, 8), sharex="col")
for col, (title, log) in enumerate([("B) firmware query", resB["hex"]["log"]),
                                    ("C) nullspace excitation", resC[("hex", "BrHatEuler")]["log"])]):
    axs[0, col].plot(log["t"], log["x"]); axs[0, col].set_ylabel("x = w^2/wmax^2 [-]")
    axs[0, col].set_title(title + " (hex)")
    axs[1, col].plot(log["t"], log["Omega"]); axs[1, col].set_ylabel("Omega [rad/s]")
    cent = -np.cross(log["Omega"], np.cross(log["Omega"], [5e-3, 0, 0]))
    tang = -np.cross(log["dOmega"], [5e-3, 0, 0])
    axs[2, col].plot(log["t"], tang, "--"); axs[2, col].plot(log["t"], cent)
    axs[2, col].set_ylabel("spf error for dr=5mm x [N/kg]\n-- tangential, - centripetal")
    axs[2, col].set_xlabel("t [s]")
for a in axs.flat:
    a.grid(True)
fig.tight_layout()
if FIGDIR:
    fig.savefig(os.path.join(FIGDIR, "imuLocationImplications.png"), dpi=110)
elif matplotlib.get_backend().lower() != "agg":
    plt.show()
