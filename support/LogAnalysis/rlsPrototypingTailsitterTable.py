from indiflight_log_tools import IndiflightLog
from indiflight_log_tools.signal_tools import Signal
from estimators import LMS, RLS, RLS_fortescue, EWMV, Welford, LS
from pyFlightPlotter import BlittedCursor
from pyFlightPlotter.crafts import Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport

import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
plt.close('all')

from copy import deepcopy

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(description="Perform offline estimator prototyping on a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logpath", type=str, help="Path to the log file.")
# parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
# parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                    help="Crop the log to the given time range (in seconds).")
# parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")

args = parser.parse_args()
# 
# if args.name is None:
#     args.name = args.logfile.split("/")[-1].split(".")[0]

# log_path  = "/mnt/data/WorkData/BlackboxLogs/tailsitterSimStudy"

import glob
log_glob = "LOG*.BFL"
log_list = sorted( glob.glob(f"{args.logpath}/{log_glob}") )
logs = []
for log_file in log_list:
    print(f"Processing log file: {log_file}")
    log = IndiflightLog(log_file, logId=1, resetTime=False)
    log.name = log_file.split("/")[-1].split(".")[0]
    if args.crop:
        log.data, _ = log.crop(args.crop[0], args.crop[1])

    logs.append(log)

    # fplt = IndiflightPlotter(log.data, Ns=2, Nr=2, name=f"{args.name} -- Flight Data")

    # craft = Tailsitter()
    # pplt = IndiflightViewport(craft, log.data, Nr=2, Ns=2, follow=False, title=f"{args.name} -- Flight Data")
    # fplt.connect_viewport(pplt)

# add output folder to logfolder path
import os
output_path = f"{args.logpath}/output"
os.makedirs(output_path, exist_ok=True)


#%% load data for offline estimator

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

import pandas as pd
table = pd.DataFrame()

Ixx_true, Iyy_true, Izz_true = 6e-3, 2e-3, 6.5e-3
k = 1.254e-06
dy = 1.26e-01
cd = np.array([-6.241e-07, 0, 0, 0, -2.601e-08, -7.293e-08])
cdd  = np.array([         0,          0,          0,          0, -1.956e-03,          0], dtype=np.float32)
cddd = np.array([         0,          0,          0,          0, -2.961e-05,          0], dtype=np.float32)

Phi = np.array([
   [+2.748e-01,          0, +3.666e-02,          0, -1.479e-04,          0],
   [         0, +3.128e-02,          0, +1.425e-03,          0, -4.223e-03],
   [+3.666e-02,          0, +3.950e-02,          0, +1.490e-04,          0],
   [         0, +1.425e-03,          0, +8.233e-04,          0, +7.577e-04],
   [-1.479e-04,          0, +1.490e-04,          0, +8.688e-04,          0],
   [         0, -4.223e-03,          0, +7.577e-04,          0, +3.282e-03],
], dtype=np.float32)

d0_true = np.array([-3.665e-1, -1.602e-1])

Cld_true = float(0.0)   / Ixx_true
Cmd_true = float(cd[4]) / Iyy_true
Cnd_true = float(cd[5]) / Izz_true

Cmdd_true = float(cdd[4]) / Iyy_true
Cmddd_true = float(cddd[4]) / Iyy_true
Cnwd_true = -3.34e-6 / Izz_true

Clww_true = 1.556e-7 / Ixx_true
Cmww_true = ( 0.0 - Cmd_true * d0_true )
Cnww_true = ( 2.734e-08 - Cnd_true * d0_true ) / Izz_true

Clp_true = Phi[3,3] / Ixx_true
Cmq_true = Phi[4,4] / Iyy_true
Cnr_true = Phi[5,5] / Izz_true

inertia_ratios = np.array([(Izz_true - Iyy_true) / Ixx_true,
                           (Ixx_true - Izz_true) / Iyy_true,
                           (Iyy_true - Ixx_true) / Izz_true])

# parameters = [Clww, Cmww, Cnww, Cmd, Cnd]  keep d0 separate
eval_pars = {'Clww1': [0, Clww_true],      'Clww2': [3, -Clww_true],
             'Cmww1': [6, Cmww_true[0]],   'Cmww2': [10, Cmww_true[1]],
             'Cmd1':  [7, Cmd_true],       'Cmd2':  [11, Cmd_true],
             'Cmdd1':  [8, Cmdd_true],      'Cmdd2':  [12, Cmdd_true],
             'Cmddd1':  [9, Cmddd_true],     'Cmddd2':  [13, Cmddd_true],
             'Cnww1': [14, Cnww_true[0]],  'Cnww2': [17, -Cnww_true[1]],
             'Cnd1':  [15, Cnd_true],      'Cnd2':  [18, -Cnd_true],
             'Cnwd1':  [16, Cnwd_true],     'Cnwd2':  [19, -Cnwd_true],
             'sigmap': [20, inertia_ratios[0]], 'sigmaq': [21, inertia_ratios[1]], 'sigmar': [22, inertia_ratios[2]],
             'Clp': [23, Clp_true], 'Cmq': [24, Cmq_true], 'Cnr': [25, Cnr_true]
            }

row = {'logfile': 'groundtruth', "model": "groundtruth"}
row['d0_1'] = d0_true[0]
row['d0_2'] = d0_true[1]
for par_name, (par_idx, par_true) in eval_pars.items():
    row[par_name] = par_true

table = pd.concat([table, pd.DataFrame([row])], ignore_index=True)



def runRls(log: IndiflightLog):
    N = log.data.shape[0]
    iend = log.data.index[-1]
    n = 2

    t_raw = log.data["timeS"]                                    .to_numpy()
    O_raw = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
    a_raw = log.data[[f"accADCafterRpm[{i}]"  for i in range(3)]].to_numpy()
    w_raw = log.data[[f"omegaUnfiltered[{i}]" for i in range(n)]].to_numpy()
    dm_raw = log.data[[f"motor[{i}]" for i in range(n)]]         .to_numpy() + 0.084
    u_raw = log.data[[f"u[{i}]" for i in range(4)]]              .to_numpy()
    d_raw = log.data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy()
    # d_raw = np.roll(d_raw, -15, axis=0)
    q_raw = log.data[[f"quat[{i}]" for i in range(4)]]           .to_numpy()
    v_raw = log.data[[f"localVel[{i}]" for i in range(3)]]       .to_numpy()

    t = np.linspace(t_raw[0], t_raw[-1], N)
    dt = np.mean(np.diff(t))

    O = Signal(t_raw, O_raw, rebase=t)
    a = Signal(t_raw, a_raw, rebase=t)
    w = Signal(t_raw, w_raw, rebase=t)
    dm = Signal(t_raw, dm_raw, rebase=t)
    u = Signal(t_raw, u_raw, rebase=t)
    d = Signal(t_raw, d_raw, rebase=t)
    q = Signal(t_raw, q_raw, rebase=t)
    v = Signal(t_raw, v_raw, rebase=t)

    order = 2
    freq_hz = 15
    Of = O.filter("lowpass", order, freq_hz)
    af = a.filter("lowpass", order, freq_hz)
    wf = w.filter("lowpass", order, freq_hz)
    dmf = dm.filter("lowpass", order, freq_hz)
    uf = u.filter("lowpass", order, freq_hz)
    df = d.filter("lowpass", order, freq_hz)
    qf = q.filter("lowpass", order, freq_hz)
    vf = v.filter("lowpass", order, freq_hz)

    import quaternion
    qfy_norm = quaternion.as_quat_array( qf.y / np.linalg.norm(qf.y, axis=1, keepdims=True))
    qfy_norm_inv = quaternion.as_quat_array( (qf.y * np.array([-1, 1, 1, 1])) / np.linalg.norm(qf.y, axis=1, keepdims=True))
    R_BI = quaternion.as_rotation_matrix(qfy_norm)
    R_IB = quaternion.as_rotation_matrix(qfy_norm_inv)
    v_B = (R_IB @ vf.y[:, :, np.newaxis]).squeeze()
    v_norm = np.linalg.norm(v_B, axis=1)
    O_norm = np.linalg.norm(Of.y, axis=1)

    eta_B = np.hstack((v_B, Of.y))
    phi = 1.0
    eta = np.sqrt(0*v_norm**2 + phi * O_norm**2)

    #%% do moments first
    Oyz = Of.y[:, 1] * Of.y[:, 2]
    Ozx = Of.y[:, 2] * Of.y[:, 0]
    Oxy = Of.y[:, 0] * Of.y[:, 1]
    OI = np.array([np.diag([Oxy[i], Oyz[i], Ozx[i]]) for i in range(Oyz.shape[0])])

    w2 = wf.y*wf.y
    wdot = wf.dot().y
    w2d = w2 * df.y
    ddot = df.dot().y
    ddotdot = df.dot().dot().y

    w2a = np.repeat(w2[:,0] + w2[:,1], 3).reshape(-1, 3)
    w2t = np.array([w2[:,0] - w2[:,1], w2[:,0] + w2[:,1], w2[:,0] - w2[:,1]]).T
    wdott = np.array([wdot[:,0] - wdot[:,1], wdot[:,0] + wdot[:,1], wdot[:,0] - wdot[:,1]]).T
    w2dt = np.array([w2d[:,0] - w2d[:,1], w2d[:,0] + w2d[:,1], w2d[:,0] - w2d[:,1]]).T
    ddott = np.array([ddot[:,0] - ddot[:,1], ddot[:,0] + ddot[:,1], ddot[:,0] - ddot[:,1]]).T
    ddotdott = np.array([ddotdot[:,0] - ddotdot[:,1], ddotdot[:,0] + ddotdot[:,1], ddotdot[:,0] - ddotdot[:,1]]).T

    rls_acts = []

    rls_act_all = RLS(26, 3, gamma=1e-11, forgetting=0.9999)
    rls_act_all.setTitle("RLS Moments -- Inertias, Actuators and 3-param Phi")
    # rls_act_all.setParameters([0, 0, 0,   0, 0, 0,  0, 0, 0,    0, 0, 0,  0, 0, 0,    0, 0, 0,  0, 0, 0,   0, 0, 0])
    rls_act_all.setCovariance(1e-12*np.diag([1, 1, 1e8,  1, 1, 1e8,    1, 1, 1e8, 1e7,  1, 1, 1e8, 1e7,    1, 1, 1e8,  1, 1, 1e8,    1e9, 1e9, 1e9, 1e8, 1e8, 1e8]))
    rls_acts.append(rls_act_all)

    ls_act_all = LS(26, 3)
    ls_act_all.setTitle("LS Moments -- Inertias, Actuators and 3-param Phi")
    rls_acts.append(ls_act_all)

    rls_act_noI = deepcopy(rls_act_all)
    rls_act_noI.setTitle("RLS Moments -- No Inertias")
    rls_acts.append(rls_act_noI)

    rls_act_noI_noPhi = deepcopy(rls_act_all)
    rls_act_noI_noPhi.setTitle("RLS Moments -- Actuators only")
    rls_acts.append(rls_act_noI_noPhi)

    rls_act_noI_noPhi_noDdot = deepcopy(rls_act_all)
    rls_act_noI_noPhi_noDdot.setTitle("RLS Moments -- No Delta derivatives")
    rls_acts.append(rls_act_noI_noPhi_noDdot)

    rls_act_noI_noPhi_noDdot_noWdot = deepcopy(rls_act_all)
    rls_act_noI_noPhi_noDdot_noWdot.setTitle("RLS Moments -- No Delta or Omega derivatives")
    rls_acts.append(rls_act_noI_noPhi_noDdot_noWdot)

    A_act_hist = []
    count = 0
    for ti, di, w2i, wdoti, w2_ai, w2_ti, w2_d_ti, wdot_ti, ddot_ti, ddotdot_ti, Oi, Odoti, eta_Bi, eta_i, ddot_i, ddotdot_i in tqdm(zip(t, df.y, w2, wdot, w2a, w2t, w2dt, wdott, ddott, ddotdott, Of.y, Of.dot().y, eta_B, eta, df.dot().y, df.dot().dot().y), total=len(w2a)):
        y = Odoti
        # first try: actuayors only, negelct aerodynamics
        # T = Cw2 * w2ti  +  Cw2d * w2dti  +  Cwdot * wdotti  +  Cddot * ddti  +  Cddotdot * dddti
        inertia_regs = -np.diag([Oi[1]*Oi[2], Oi[2]*Oi[0], Oi[0]*Oi[1]])

        #  |  w1*w1  |  w1*w1*delta1  |  wdot1  |  w2*w2  |  w2*w2*delta2  |  wdot2  |
        A_act_all = np.zeros((3, 26))
        A_act_all[0, 0:3]   = [w2i[0], w2i[0] * di[0], wdoti[0]]
        A_act_all[0, 3:6]   = [w2i[1], w2i[1] * di[1], wdoti[1]]
        A_act_all[1, 6:10]  = [w2i[0], w2i[0] * di[0], ddot_i[0], ddotdot_i[0]]
        A_act_all[1, 10:14] = [w2i[1], w2i[1] * di[1], ddot_i[1], ddotdot_i[1]]
        A_act_all[2, 14:17] = [w2i[0], w2i[0] * di[0], wdoti[0]]
        A_act_all[2, 17:20] = [w2i[1], w2i[1] * di[1], wdoti[1]]
        A_act_all[:, 20:23] = inertia_regs
        A_act_all[:, 23:26] = - eta_i * np.diag(eta_Bi[3:]) # C_m_w
        A_act_hist.append(A_act_all)

        rls_act_all.newSample(A_act_all, y, ti); rls_act_all.update()
        ls_act_all.newSample(A_act_all, y, ti);

        count += 1 
        if count > 150 and count % 10 == 0:
            ls_act_all.update()

        A_act_noI = A_act_all.copy()
        A_act_noI[:, 20:23] = 0
        rls_act_noI.newSample(A_act_noI, y, ti); rls_act_noI.update()

        A_act_noI_noPhi = A_act_noI.copy()
        A_act_noI_noPhi[:, 23:26] = 0
        rls_act_noI_noPhi.newSample(A_act_noI_noPhi, y, ti);rls_act_noI_noPhi.update()

        A_act_noI_noPhi_noDdot = A_act_noI_noPhi.copy()
        A_act_noI_noPhi_noDdot[:, [8,9,12,13]] = 0
        rls_act_noI_noPhi_noDdot.newSample(A_act_noI_noPhi_noDdot, y, ti); rls_act_noI_noPhi_noDdot.update()

        A_act_noI_noPhi_noDdot_noWdot = A_act_noI_noPhi_noDdot.copy()
        A_act_noI_noPhi_noDdot_noWdot[:, [2,5,16,19]] = 0
        rls_act_noI_noPhi_noDdot_noWdot.newSample(A_act_noI_noPhi_noDdot_noWdot, y, ti); rls_act_noI_noPhi_noDdot_noWdot.update()

    parGroups = [[0,3], [1,4], [2,5],   [6,10], [7,11], [8,12], [9,13],   [14,17], [15,18], [16,19], [20,21,22], [23,24,25]]
    parGroupNames = ["$C_{\\omega^2, p}$", "$C_{{\\omega^2} \\delta, p}$", "$C_{\\dot{\\omega}, p}$",
                     "$C_{\\omega^2, q}$", "$C_{{\\omega^2} \\delta, q}$", "$C_{\\dot{\\delta}, q}$", "$C_{\\ddot{\\delta}, q}$",
                     "$C_{\\omega^2, r}$", "$C_{{\\omega^2} \\delta, r}$", "$C_{\\dot{\\omega}, r}$",
                     "$C_{m\\sigma}$",
                     "$C_{m\\omega diag}$"]
    truePars = [[Clww_true, -Clww_true], [0,0], [0,0],
                [Cmww_true[0], Cmww_true[1]], [Cmd_true, Cmd_true], [Cmdd_true,Cmdd_true], [Cmddd_true,Cmddd_true],
                [Cnww_true[0], -Cnww_true[1]], [Cnd_true, -Cnd_true], [Cnwd_true, -Cnwd_true],
                list(inertia_ratios),
                [Clp_true, Cmq_true, Cnr_true]]

    for rls in rls_acts:
        _ = rls.plotParameters(parGroups=parGroups, truePars=truePars, parGroupNames=parGroupNames, sharey=False, zoomy=False)
        rls.f.savefig(f"{output_path}/{rls.name}_{log.name}_parameters.png", dpi=300)
        plt.close(rls.f)

        for i, axis in enumerate(["Roll", "Pitch", "Yaw"]):
            f,V,c,e,X,Y,t = rls.diagnose(i, output_name=axis)
            f.savefig(f"{output_path}/{rls.name}_diagnose_{axis.lower()}_{log.name}.png", dpi=300)
            plt.close(f)

            # output V and error-regressor correlation (e) as one single txt
            np.savetxt(f"{output_path}/{rls.name}_diagnose_V_{axis.lower()}_{log.name}.txt", V)
            np.savetxt(f"{output_path}/{rls.name}_diagnose_e_{axis.lower()}_{log.name}.txt", e)



    def get_d0(rls):
        minCmd1_d0 = rls.theta[9,0]
        Cmd1 = rls.theta[10,0]
        d0_1 = - minCmd1_d0 / Cmd1 if np.abs(Cmd1) > 1e-6 else 0.0

        minCmd2_d0 = rls.theta[12,0]
        Cmd2 = rls.theta[13,0]
        d0_2 = - minCmd2_d0 / Cmd2 if np.abs(Cmd2) > 1e-6 else 0.0

        return float(d0_1), float(d0_2)

    #%% error metrics

    # 1. prediction error (RMSE) over dataset
    def compute_RMSE(rls, A_data, y_data):
        y_preds = rls.predictNew(A_data).squeeze()
        e = y_preds - y_data
        rmse = np.sqrt(np.mean(np.linalg.norm(e, axis=1)**2))
        return float(rmse)


    A_data = A_act_hist
    y_data = Of.dot().y
    for rls in rls_acts:
        row = {'logfile': log.name}
        #print(f"Parameter errors for {rls.name}:")
        par_err_rel_squared_sum = 0.0
        n_correct_signs = 0
        n_correct_bounds = 0
        bound_size_rel_sum = 0.0
        row["model"] = rls.name

        row["d0_1"], row["d0_2"] = get_d0(rls)

        row["RMSE"] = compute_RMSE(rls, A_data, y_data)
        row["RMSE_rel"] = row["RMSE"] / np.std(Of.dot().y)

        for par_name, (par_idx, par_true) in eval_pars.items():

            row[par_name] = rls.theta[par_idx, 0]
            if hasattr(rls, 'theta_bounds'):
                row[f"{par_name}_lb"] = rls.theta_bounds[par_idx, 0]
                row[f"{par_name}_ub"] = rls.theta_bounds[par_idx, 1]

            # # 2. parameter error relative to true values (do only for definitely non-zero values)
            # par_est = rls.theta[par_idx, 0]
            # par_err = par_est - par_true
            # par_err_rel = par_err / par_true if np.abs(par_true) > 1e-6 else 0.0
            # #print(f"  {par_name}: est={par_est:.3e}, true={par_true:.3e}, err={par_err:.3e}, rel err={par_err_rel:.2%}")
            # par_err_rel_squared_sum += par_err_rel**2

            # # 3. number of correct signs of relevant parameters
            # if np.sign(par_est) == np.sign(par_true):
            #     n_correct_signs += 1

            # # 4. number of bounds including the correct sign
            # n_correct_bounds += (rls.theta_bounds[par_idx, 0] < par_true < rls.theta_bounds[par_idx, 1])

            # # 5. size of the bounds relative to the parameter value
            # bound_size_rel_sum += (rls.theta_bounds[par_idx, 1] - rls.theta_bounds[par_idx, 0]) / np.abs(par_true) if np.abs(par_true) > 1e-6 else 0.0

            # # print(f"    relative bound size: {bound_size_rel:.2%}")

        # par_err_rel_rms = np.sqrt(par_err_rel_squared_sum / len(eval_pars))
        # rls.par_err_rel_rms = par_err_rel_rms
        # print(f"{rls.name} - RMS relative parameter error: {par_err_rel_rms:.2%}")

        # rls.n_correct_signs = n_correct_signs
        # print(f"{rls.name} - Number of correct parameter signs: {n_correct_signs} out of {len(eval_pars)}")

        # rls.n_correct_bounds = n_correct_bounds
        # print(f"{rls.name} - Number of parameter bounds including true value: {n_correct_bounds} out of {len(eval_pars)}")

        # rls.bound_size_rel_avg = bound_size_rel_sum / len(eval_pars)
        # print(f"{rls.name} - Average relative parameter bound size: {rls.bound_size_rel_avg:.2%}")

        # rmse = compute_RMSE(rls, A_data, y_data)
        # print(f"RMSE for {rls.name}: {rmse:.2f} rad/s^2, {rmse/np.std(Of.dot().y):.2%} of data stddev")

        global table
        table = pd.concat([table, pd.DataFrame([row])], ignore_index=True)

    return rls_acts

rls_list_list = []
for log in logs:
    rls_list = runRls(log)
    rls_list_list.append(rls_list)


# output table as csv
table.to_csv(f"{output_path}/estimator_comparison_table.csv", index=False)


