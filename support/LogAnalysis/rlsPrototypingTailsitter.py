from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import RLS, LS
from tailsitter_id import offline_rls
from pyFlightPlotter import BlittedCursor
from pyFlightPlotter.crafts import Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport

import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
plt.close('all')

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Perform offline estimator prototyping on a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

fplt = IndiflightPlotter(log.data, Ns=2, Nr=2, name=f"{args.name} -- Flight Data")

craft = Tailsitter()
pplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Flight Data")
fplt.connect_viewport(pplt)

# splt = SysIdPlotter(log.data, name=f"{args.name} -- Onboard Sys ID Analysis")
# splt.connect_viewport(pplt)

# cursor = BlittedCursor(fplt.all_axes, sharex=True)
# cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)


#%% load data for offline estimator

N = log.data.shape[0]
iend = log.data.index[-1]
n = 2

# gyro, rotor speeds, servos, attitude and velocity: tailsitter_id.offline_rls (15 Hz, localVel)
prep = offline_rls.prepare_data(log.data, freq_hz=15, order=2)
t = prep['t']
Of, wf, df = prep['signals']['Of'], prep['signals']['wf'], prep['signals']['df']

t_raw = log.data["timeS"]                                    .to_numpy()
dm_raw = log.data[[f"motor[{i}]" for i in range(n)]]         .to_numpy() + 0.084
u_raw = log.data[[f"u[{i}]" for i in range(4)]]              .to_numpy()

order = 2
freq_hz = 15
dmf = Signal(t_raw, dm_raw, rebase=t).filter("lowpass", order, freq_hz)
uf = Signal(t_raw, u_raw, rebase=t).filter("lowpass", order, freq_hz)

#%% do moments first

rls_motors = RLS(8, 2, gamma=1e-11, forgetting=0.9999)
rls_motors.setTitle("RLS for Motors")
rls_motors.setParameters(              [0, 0, 0, 0,  0, 0, 0, 0])
rls_motors.setCovariance(1e1 * np.diag([1, 1, 1, 1,  1, 1, 1, 1]))

rls_servos = RLS(8, 2, gamma=1e-11, forgetting=0.9999)
rls_servos.setTitle("RLS for Servos")
rls_servos.setParameters(              [0, 0, 0, 0,  0, 0, 0, 0])
rls_servos.setCovariance(1e1 * np.diag([1, 1, 1, 1,  1, 1, 1, 1]))

ls_servos = LS(8, 2)
ls_servos.setTitle("LS for Servos")
ls_servos.setParameters(              [0, 0, 0, 0,  0, 0, 0, 0])

count = 0
for ti, dmi, wi, wdoti, ui, di, ddoti, dddoti in tqdm(zip(t, dmf.y, wf.y, wf.dot().y, uf.y, df.y, df.dot().y, df.dot().dot().y), total=len(dmf.y)):
    y = wi
    A = np.zeros((2, 8))
    for i in range(2):
        A[i, i*4]   = dmi[i]
        A[i, i*4+1] = np.sqrt( dmi[i] ) if dmi[i] >= 0 else 0.0
        A[i, i*4+2] = 1.0
        A[i, i*4+3] = -1e-3*wdoti[i]

    rls_motors.newSample(A, y, ti); rls_motors.update()

    y = di
    for i in range(2):
        A[i, i*4]   = ui[i+2]
        A[i, i*4+1] = 1.0
        A[i, i*4+2] = -ddoti[i]
        A[i, i*4+3] = -dddoti[i]

    rls_servos.newSample(A, y, ti); rls_servos.update()
    ls_servos.newSample(A, y, ti)

    count += 1
    if count % 20 == 0:
        ls_servos.update()


# rls_motors.plotParameters(parGroups=[[0,4], [1,5], [2,6], [3,7]],
#                         parGroupNames=["a", "b", "idle", "tau"],
#                         sharey=False, zoomy=False)
# 
# rls_servos.plotParameters(parGroups=[[0,4], [1,5], [2,6], [3,7]],
#                         parGroupNames=["a", "b", "c", "d"],
#                         sharey=False, zoomy=False)
# 
# ls_servos.theta[4] = 1.75
# ls_servos.theta[5] = 0.0
# ls_servos.theta[6] = 0.03333
# ls_servos.theta[7] = 4.2e-4
# 
# ls_servos.plotParameters(parGroups=[[0,4], [1,5], [2,6], [3,7]],
#                         parGroupNames=["a", "b", "c", "d"],
#                         sharey=False, zoomy=False)

#%% moment RLS models (tailsitter_id.offline_rls: regressors, initial covariances, forgetting)

rls = offline_rls.run(prep, full=True)
rls_phi9, rls_phi9_noI, rls_phi7_noI = rls['phi9'], rls['phi9_noI'], rls['phi7_noI']
rls_phi3_noI, rls_noPhi_noI = rls['phi3_noI'], rls['noPhi_noI']
rls_phi7, rls_phi3, rls_noPhi = rls['phi7'], rls['phi3'], rls['noPhi']
rls_act_phi3_I, rls_act_phi3_noI, rls_act_noPhi_noI = rls['act_phi3_I'], rls['act_phi3_noI'], rls['act_noPhi_noI']

# regressors of rls_act_phi3_I (incl. inertia and aero columns), for the error metrics
A_act_hist = offline_rls.regressors('act', prep['O'], prep['w'], prep['wdot'], prep['d'], vB=prep['vB'])

# remove all [9, 10] and [11] entries from all these parGroups, and also parGroupNames, go!

# rls_phi9.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [12,13,14,15], [16,17,18], [19,20]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$", "$C_{m\\omega_{cross}}$"],
#                         sharey=False, zoomy=False)
# 
# rls_phi9_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [12,13,14,15], [16,17,18], [19,20]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$", "$C_{m\\omega_{cross}}$"],
#                             sharey=False, zoomy=False)
# 
# rls_phi7_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [12,13,14,15], [16,17,18]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$"],
#                             sharey=False, zoomy=False)
# 
# rls_phi3_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8], [16,17,18]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{m\\omega diag}$"],
#                             sharey=False, zoomy=False)
# 
# rls_noPhi_noI.plotParameters(parGroups=[[3,4,5], [6,7], [8]],
#                             parGroupNames=["$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$"],
#                             sharey=False, zoomy=False)
# 
# rls_phi7.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [12,13,14,15], [16,17,18]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{mv}$", "$C_{m\\omega diag}$"],
#                         sharey=False, zoomy=False)
#  
rls_phi3.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8], [16,17,18]],
                        parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$", "$C_{m\\omega diag}$"],
                        sharey=False, zoomy=False)
# rls_noPhi.plotParameters(parGroups=[[0,1,2], [3,4,5], [6,7], [8]],
#                         parGroupNames=["$\\sigma$", "$C_{\\omega^2}$", "$C_{{\\omega^2} \\delta}$", "$C_\\dot{\\omega}$"],
#                         sharey=False, zoomy=False)

Ixx_true, Iyy_true, Izz_true = 6e-3, 2e-3, 6.5e-3
k = 1.254e-06
dy = 1.26e-01
cd = np.array([-6.241e-07, 0, 0, 0, -2.601e-08, -7.293e-08])


d0_true = 0*np.array([-3.665e-1, -1.602e-1])

Cld_true = float(0.0)
Cmd_true = float(cd[4])
Cnd_true = float(cd[5])

Clww_true = 1.556e-7 / Ixx_true
Cmww_true = ( 0.0 - Cmd_true * d0_true ) / Iyy_true
Cnww_true = ( 2.734e-08 - Cnd_true * d0_true ) / Izz_true


rls_act_phi3_I.plotParameters(parGroups=[[0,1,2], [3,6], [4,7], [5,8],   [9,12], [10,13], [11,14],   [15,18], [16,19], [17,20], [21,22,23]],
                           truePars=[[0,0,0], 
                                     [Clww_true, -Clww_true], [0,0], [0,0],
                                     [Cmww_true[0], Cmww_true[1]], [Cmd_true/Iyy_true, Cmd_true/Iyy_true], [0,0],
                                     [Cnww_true[0], -Cnww_true[1]], [Cnd_true / Izz_true, -Cnd_true / Izz_true], [-3.34e-6/6.5e-3, 3.34e-6/6.5e-3],
                                     [0, 0, 0]],
                           parGroupNames=["$\\sigma$",
                                          "$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                                          "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\omega}, q}$",
                                          "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                                          "$C_{m\\omega diag}$"],
                           sharey=False, zoomy=False)


rls_act_phi3_noI.plotParameters(parGroups=[[0,1,2], [3,6], [4,7], [5,8],   [9,12], [10,13], [11,14],   [15,18], [16,19], [17,20], [21,22,23]],
                           truePars=[[0,0,0], 
                                     [Clww_true, -Clww_true], [0,0], [0,0],
                                     [Cmww_true[0], Cmww_true[1]], [Cmd_true/Iyy_true, Cmd_true/Iyy_true], [0,0],
                                     [Cnww_true[0], -Cnww_true[1]], [Cnd_true/Izz_true, -Cnd_true/Izz_true], [-3.34e-6/6.5e-3, 3.34e-6/6.5e-3],
                                     [0, 0, 0]],
                           parGroupNames=["$\\sigma$",
                                          "$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                                          "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\omega}, q}$",
                                          "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                                          "$C_{m\\omega diag}$"],
                           sharey=False, zoomy=False)

rls_act_noPhi_noI.plotParameters(parGroups=[[0,1,2], [3,6], [4,7], [5,8],   [9,12], [10,13], [11,14],   [15,18], [16,19], [17,20], [21,22,23]],
                           truePars=[[0,0,0], 
                                     [Clww_true, -Clww_true], [0,0], [0,0],
                                     [Cmww_true[0], Cmww_true[1]], [Cmd_true/Iyy_true, Cmd_true/Iyy_true], [0,0],
                                     [Cnww_true[0], -Cnww_true[1]], [Cnd_true / Izz_true, -Cnd_true / Izz_true], [-3.34e-6/6.5e-3, 3.34e-6/6.5e-3],
                                     [0, 0, 0]],
                           parGroupNames=["$\\sigma$",
                                          "$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                                          "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\omega}, q}$",
                                          "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                                          "$C_{m\\omega diag}$"],
                           sharey=False, zoomy=False)


def get_d0(rls):
    minCmd1_d0 = rls.theta[9,0]
    Cmd1 = rls.theta[10,0]
    d0_1 = - minCmd1_d0 / Cmd1 if np.abs(Cmd1) > 1e-6 else 0.0

    minCmd2_d0 = rls.theta[12,0]
    Cmd2 = rls.theta[13,0]
    d0_2 = - minCmd2_d0 / Cmd2 if np.abs(Cmd2) > 1e-6 else 0.0

    return float(d0_1), float(d0_2)

print("rls_act_noPhi_noI d0 estimates:", get_d0(rls_act_noPhi_noI))
print("rls_act_phi3_I d0 estimates:", get_d0(rls_act_phi3_I))
print("rls_act_phi3_noI d0 estimates:", get_d0(rls_act_phi3_noI))

#%% error metrics

rls_acts = [rls_act_noPhi_noI, rls_act_phi3_noI, rls_act_phi3_I]

# 1. prediction error (RMSE) over dataset
def compute_RMSE(rls, A_data, y_data):
    y_preds = rls.predictNew(A_data).squeeze()
    e = y_preds - y_data
    rmse = np.sqrt(np.mean(np.linalg.norm(e, axis=1)**2))
    return float(rmse)

# parameters = [Clww, Cmww, Cnww, Cmd, Cnd]  keep d0 separate
eval_pars = {'Clww1': [3, Clww_true],          'Clww2': [6, -Clww_true],
             'Cmww1': [9, Cmww_true[0]],       'Cmww2': [12, Cmww_true[1]],
             'Cnww1': [15, Cnww_true[0]],      'Cnww2': [18, -Cnww_true[1]],
             'Cmd1':  [10, Cmd_true/Iyy_true], 'Cmd2':  [13, Cmd_true/Iyy_true],
             'Cnd1':  [16, Cnd_true/Izz_true], 'Cnd2':  [19, -Cnd_true/Izz_true],
            }

A_data = A_act_hist
y_data = Of.dot().y
for rls in rls_acts:
    #print(f"Parameter errors for {rls.name}:")
    par_err_rel_squared_sum = 0.0
    n_correct_signs = 0
    n_correct_bounds = 0
    bound_size_rel_sum = 0.0
    for par_name, (par_idx, par_true) in eval_pars.items():

        # 2. parameter error relative to true values (do only for definitely non-zero values)
        par_est = rls.theta[par_idx, 0]
        par_err = par_est - par_true
        par_err_rel = par_err / par_true if np.abs(par_true) > 1e-6 else 0.0
        #print(f"  {par_name}: est={par_est:.3e}, true={par_true:.3e}, err={par_err:.3e}, rel err={par_err_rel:.2%}")
        par_err_rel_squared_sum += par_err_rel**2

        # 3. number of correct signs of relevant parameters
        if np.sign(par_est) == np.sign(par_true):
            n_correct_signs += 1

        # 4. number of bounds including the correct sign
        n_correct_bounds += (rls.theta_bounds[par_idx, 0] < par_true < rls.theta_bounds[par_idx, 1])

        # 5. size of the bounds relative to the parameter value
        bound_size_rel_sum += (rls.theta_bounds[par_idx, 1] - rls.theta_bounds[par_idx, 0]) / np.abs(par_true) if np.abs(par_true) > 1e-6 else 0.0

        # print(f"    relative bound size: {bound_size_rel:.2%}")

    par_err_rel_rms = np.sqrt(par_err_rel_squared_sum / len(eval_pars))
    rls.par_err_rel_rms = par_err_rel_rms
    print(f"{rls.name} - RMS relative parameter error: {par_err_rel_rms:.2%}")

    rls.n_correct_signs = n_correct_signs
    print(f"{rls.name} - Number of correct parameter signs: {n_correct_signs} out of {len(eval_pars)}")

    rls.n_correct_bounds = n_correct_bounds
    print(f"{rls.name} - Number of parameter bounds including true value: {n_correct_bounds} out of {len(eval_pars)}")

    rls.bound_size_rel_avg = bound_size_rel_sum / len(eval_pars)
    print(f"{rls.name} - Average relative parameter bound size: {rls.bound_size_rel_avg:.2%}")

    rmse = compute_RMSE(rls, A_data, y_data)
    print(f"RMSE for {rls.name}: {rmse:.2f} rad/s^2, {rmse/np.std(Of.dot().y):.2%} of data stddev")

print()



#%% dress up plots

# rls_var.plotParameters()

all_rls = [rls_phi3, rls_act_phi3_I, rls_act_phi3_noI, rls_act_noPhi_noI]

# all_rls = [rls_phi3, rls_var]
# all_rls = [rls_noPhi, rls_var]

# display figures
all_axes = []
for rls in all_rls:
    all_axes.extend(rls.all_axes)

# cursor = BlittedCursor(all_axes + fplt.all_axes + rls_motors.all_axes + rls_servos.all_axes + ls_servos.all_axes, sharex=True)
cursor = BlittedCursor(all_axes + fplt.all_axes, sharex=True)
plt.show()


# # save all figures as eps
# for rls in all_rls:
#     rls.f.savefig("output/" + rls.name.replace(" ", "_") + ".eps", format='eps', dpi=300)
# cursor = BlittedCursor(fplt.all_axes, sharex=True)

# %%
