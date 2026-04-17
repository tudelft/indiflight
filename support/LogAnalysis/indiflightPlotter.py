import numpy as np

from pyFlightPlotter import FlightPlotterBase, Viewport, Craft3D
from scipy.spatial.transform import Rotation as R

from indiflight_log_tools.signal_tools import Signal

class IndiflightPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for general Indiflight analysis"""
    def __init__(self, data, Nr=0, Ns=0, name="Flight Plotter"):
        # extract time and intialize base class
        self.data = data
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        # check for fields
        self.has_pos = 'pos[0]' in self.data.columns
        self.has_servo_feedback = 'servo_feedback[0]' in self.data.columns

        self.define_layout(figsize=(12, 8), nrows=5, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        # | pos | velB  | rate
        # | vel | spf   | drate
        # | acc | actS  | actM
        # | rc  | servo | rpm
        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 2]),
                         light=[self.data[f'gyroADCafterRpm[{i}]'].to_numpy() for i in range(3)],
                         solid=[self.data[f'gyroADC[{i}]'].to_numpy() for i in range(3)],
                         dashed=[self.data[f'gyroSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["roll", "pitch", "yaw"],
                         style_labels=["Raw", "Filtered", "Setpoint"],
                         title="Body Angular Rates",
                         ylabel="Angular Rate [rad/s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 1]),
                         light=[self.data[f'accADCafterRpm[{i}]'].to_numpy() for i in range(3)],
                         solid=[self.data[f'accSmooth[{i}]'].to_numpy() for i in range(3)],
                         dashed=[self.data[f'spfSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["X", "Y", "Z"],
                         style_labels=["Raw", "Filtered", "Setpoint"],
                         title="Body Specific Forces",
                         ylabel="Specific Force [N/kg]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 2]),
                         light=None,
                         solid=[self.data[f'alpha[{i}]'].to_numpy() for i in range(3)],
                         dashed=[self.data[f'alphaSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["roll", "pitch", "yaw"],
                         style_labels=[None, "Filtered", "Setpoint"],
                         title="Body Angular Accel.",
                         ylabel="Angular Accel. [rad/s²]")

        if self.Nr > 0:
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 2]),
                             light=[self.data[f'omegaUnfiltered[{i}]'].to_numpy() for i in range(self.Nr)],
                             solid=[self.data[f'omega[{i}]'].to_numpy() for i in range(self.Nr)],
                             dashed=None,
                             series_labels=[f"Motor {i}" for i in range(1, self.Nr + 1)],
                             style_labels=["Raw", "Onboard filt.", None],
                             title="Motor Speeds",
                             ylabel="Motor Speed [rad/s]",
                             ylimits=(-100, None))

        self._plot_timeseries(self.fig.add_subplot(self.gs[3, 0]),
                         light=None,
                         solid=[self.data[f'rcCommand[{i}]'].to_numpy() for i in range(4)],
                         dashed=None,
                         series_labels=["RC Roll", "RC Pitch", "RC Yaw", "RC Throttle"],
                         style_labels=[None, "Raw", None],
                         title="RC Commands",
                         ylabel="RC Command",
                         ylimits=(-1.1, +1.1))

        N = self.Nr + self.Ns
        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 2]),
                         light=None, #[self.data[f'motor[{i}]'].to_numpy() for i in range(N)],
                         solid=[self.data[f'u_state[{i}]'].to_numpy() for i in range(self.Nr)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(self.Nr)],
                         series_labels=[f"Actuator {str(i)}" for i in range(1,self.Nr+1)],
                         style_labels=[None, "Est. state", "Command"],
                         title="Actuator Commands Motors",
                         ylabel="Actuator Commands [-]",
                         ylimits=(-0.05, 1.05))
        if self.Ns > 0:
            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 1]),
                                light=None, #[self.data[f'motor[{i}]'].to_numpy() for i in range(N)],
                                solid=[self.data[f'u_state[{i}]'].to_numpy() for i in range(self.Nr, N)],
                                dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(self.Nr, N)],
                                series_labels=[f"Actuator {str(i)}" for i in range(self.Nr+1,N+1)],
                                style_labels=[None, "Est. state", "Command"],
                                title="Actuator Commands Servos",
                                ylabel="Actuator Commands [-]",
                                ylimits=(-1.05, 1.05))

            if self.has_servo_feedback and self.Ns > 0:
                d = np.array([self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(self.Ns)])
                dSig = Signal(self.t, d.T).filtfilt(type='lowpass', order=2, cutoff_hz=20.0)
                dDot = dSig.dot(order=1).y.T
                dDotDot = dSig.dot(order=2).y.T

                self._plot_timeseries(self.fig.add_subplot(self.gs[3, 1]),
                             light=None,
                             solid=[d[i] for i in range(self.Ns)],
                             dashed=[self.data[f'u[{i}]'].to_numpy() * 100. * np.pi / 180. for i in range(self.Nr, N)],
                             series_labels=[f"Servo {i}" for i in range(1,self.Ns+1)],
                             style_labels=[None, "Unfiltered state", "Scaled Command"],
                             title="Servo State",
                             ylabel="Servo State [rad]",
                )

                self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
                             light=[dDotDot[i] for i in range(self.Ns)],
                             solid=[10*dDot[i] for i in range(self.Ns)],
                             dashed=None,
                             series_labels=[f"Servo {i}" for i in range(1,self.Ns+1)],
                             style_labels=["2nd Derivative", "1st Derivative", None],
                             title="Servo Derivatives",
                             ylabel="Servo Derivative",
                )

        quat = self.data[[f'quat[{i}]' for i in [1,2,3,0]]].to_numpy()
        norms = np.linalg.norm(quat, axis=1)
        if (np.any(norms < 1e-6)):
            print("Warning: found near-zero quaternion norm, replacing with identity to avoid NaNs in rotation")
            quat[norms < 1e-6] = np.array([0,0,0,1])  # avoid NaNs
        rot = R.from_quat(quat)
        irot = rot.inv()

        if self.has_pos:
            self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
                             light=[self.data[f'localPos[{i}]'].to_numpy() for i in range(3)] if 'localPos[0]' in self.data.columns else None,
                             solid=[self.data[f'pos[{i}]'].to_numpy() for i in range(3)],
                             dashed=[self.data[f'posSp[{i}]'].to_numpy() for i in range(3)],
                             series_labels=["X", "Y", "Z"],
                             style_labels=["Mocap", "Estimated", "Setpoint"],
                             title="Position",
                             ylabel="Position [m]")

            velMeas = self.data[[f'localVel[{i}]' for i in range(3)]].to_numpy() if 'localVel[0]' in self.data.columns else None
            vel = self.data[[f'vel[{i}]' for i in range(3)]].to_numpy()
            velSp = self.data[[f'velSp[{i}]' for i in range(3)]].to_numpy()

            velMeasB = irot.apply(velMeas.copy()) if velMeas is not None else None
            velB = irot.apply(vel.copy())
            velSpB = irot.apply(velSp.copy())


            self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                             light=[velMeas[:, i] for i in range(3)] if velMeas is not None else None,
                             solid=[vel[:, i] for i in range(3)] if vel is not None else None,
                             dashed=[velSp[:, i] for i in range(3)] if velSp is not None else None,
                             series_labels=["X", "Y", "Z"],
                             style_labels=["Mocap", "Estimated", "Setpoint"],
                             title="Velocity Global",
                             ylabel="Velocity [m/s]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[0, 1]),
                                light=[velMeasB[:, i] for i in range(3)] if velMeas is not None else None,
                                solid=[velB[:, i] for i in range(3)] if vel is not None else None,
                                dashed=[velSpB[:, i] for i in range(3)] if velSp is not None else None,
                                series_labels=["X", "Y", "Z"],
                                style_labels=["Mocap", "Estimated", "Setpoint"],
                                title="Velocity Body",
                                ylabel="Velocity [m/s]")


            # rotate IMU acceleration to global frame
            accB = self.data[[f'accSmooth[{i}]' for i in range(3)]].to_numpy()
            accI = rot.apply(accB.copy()) + np.array([0, 0, 9.81])

            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                             light=None,
                             solid=[accI[:, i] for i in range(3)],
                             dashed=[self.data[f'accSp[{i}]'].to_numpy() for i in range(3)],
                             series_labels=["X", "Y", "Z"],
                             style_labels=[None, "Estimated", "Setpoint"],
                             title="Acceleration Global",
                             ylabel="Acceleration [m/s²]")

class IndiflightMotorSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for Motor SysId analysis"""
    def __init__(self, data, Nr=0, true=None, name="System Identification Plotter -- Motor"):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        # do some investigation
        # self.has_motor_learning = 'motor_0_rls_x[0]' in self.data.columns
        # self.has_fx_learning = 'fx_x_rls_x[0]' in self.data.columns
        self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        # self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns
        # self.has_servo = 'servo_feedback[0]' in self.data.columns

        # check amount of actuators
        self.Nr = Nr

        self.define_layout(figsize=(12, 8), nrows=6, ncols=4,
                           width_ratios=[1, 1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        Nr = self.Nr

        # motor learning data
        a = np.array([self.data[f'motor_{i}_rls_x[0]'] for i in range(Nr)])
        b = np.array([self.data[f'motor_{i}_rls_x[1]'] for i in range(Nr)])
        widle = np.array([self.data[f'motor_{i}_rls_x[2]'] for i in range(Nr)])
        tau = np.array([self.data[f'motor_{i}_rls_x[3]'] for i in range(Nr)])
        wmax = a + b
        kappa = np.zeros_like(wmax)
        kappa[a+b > 0] = a[a+b > 0] / (a[a+b > 0] + b[a+b > 0])

        motor_e_var = np.array([self.data[f'motor_{i}_rls_e_var'] for i in range(Nr)])
        motor_lambda = np.array([self.data[f'motor_{i}_rls_lambda'] for i in range(Nr)])


        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
                            light=None,
                            solid=wmax,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Max Motor Speed",
                            ylabel="Angular Rate [rad/s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                            light=None,
                            solid=widle,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Idle Motor Speed",
                            ylabel="Angular Rate [rad/s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                            light=None,
                            solid=tau,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Motor Time Constant",
                            ylabel="Time Constant [s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[3, 0]),
                            light=None,
                            solid=kappa,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Motor Kappa",
                            ylabel="Kappa [rad/s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 0]),
                            light=None,
                            solid=motor_e_var,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Motor Error Variance",
                            ylabel="Variance")

        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 0]),
                            light=None,
                            solid=motor_lambda,
                            dashed=None,
                            series_labels=[f"Motor {i}" for i in range(Nr)],
                            style_labels=[None, "Onboard", None],
                            title="Motor Forgetting Factor",
                            ylabel="Forgetting Factor")

        # fx learning data
        if self.has_extended_fx_learning:
            # we have the extended logging (like in simulation)
            pqr_range = list(range(Nr)) + list(range(2*Nr, 3*Nr))
        else:
            pqr_range = list(range(2*Nr))

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)])
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)])
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)])
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range])
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range])
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range])

        AXES = ['x', 'y', 'z', 'p', 'q', 'r']
        fx_e_var  = np.array([self.data[f'fx_{ax}_rls_e_var'] for ax in AXES])
        fx_lambda = np.array([self.data[f'fx_{ax}_rls_lambda'] for ax in AXES])

        for i, axis in enumerate(['x', 'y', 'z']):
            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                light=None,
                                solid=x if axis == 'x' else y if axis == 'y' else z,
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(Nr)],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [N/kg/(rad/s)²]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
                                light=None,
                                solid=fx_e_var[:3],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")

        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
                                light=None,
                                solid=fx_lambda[:3],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

        for i, axis in enumerate(['p', 'q', 'r']):
            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 2]),
                                light=None,
                                solid=p[:Nr] if axis == 'p' else q[:Nr] if axis == 'q' else r[:Nr],
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(Nr)],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 3]),
                                light=None,
                                solid=p[Nr:] if axis == 'p' else q[Nr:] if axis == 'q' else r[Nr:],
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(Nr)],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [Nm/(kgm^2)/(rad/s²)]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 2]),
                                light=None,
                                solid=fx_e_var[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")

        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 2]),
                                light=None,
                                solid=fx_lambda[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

class IndiflightServoSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
    def __init__(self, data, Nr=0, Ns=0, true=None, name="System Identification Plotter -- Servo"):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns

        # check amount of actuators
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        self.define_layout(figsize=(12, 8), nrows=6, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        Nr = self.Nr
        Ns = self.Ns
        N = self.N

        dmax = 1e-3*np.array([self.data[f'motor_{i}_rls_x[0]'] for i in range(Nr,N)])
        d0 = 1e-3*np.array([self.data[f'motor_{i}_rls_x[1]'] for i in range(Nr,N)])
        delay = 1e-4*np.array([self.data[f'motor_{i}_rls_x[2]'] for i in range(Nr,N)])
        tau = np.array([self.data[f'motor_{i}_rls_x[3]'] for i in range(Nr,N)])
        # motor_e_var = np.array([self.data[f'motor_{i}_rls_e_var'] for i in range(Nr)])
        # motor_lambda = np.array([self.data[f'motor_{i}_rls_lambda'] for i in range(Nr)])

        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
                            light=None,
                            solid=dmax,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[0]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Scaler",
                            ylabel="Angle/cmd [rad/1]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                            light=None,
                            solid=d0,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[1]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Neutral Servo Angle",
                            ylabel="Angle [rad]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                            light=None,
                            solid=delay,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[2]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Delay",
                            ylabel="Delay [s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[3, 0]),
                            light=None,
                            solid=tau,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[3]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Time Constant",
                            ylabel="Time Constant [s]")

        if self.has_extended_fx_learning:
            # we have the extended logging (like in simulation)
            pqr_range = list(range(Nr)) + list(range(2*Nr, 3*Nr))
        else:
            pqr_range = list(range(2*Nr))

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)])
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)])
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)])
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range])
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range])
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range])

        AXES = ['x', 'y', 'z', 'p', 'q', 'r']
        fx_e_var  = np.array([self.data[f'fx_{ax}_rls_e_var'] for ax in AXES])
        fx_lambda = np.array([self.data[f'fx_{ax}_rls_lambda'] for ax in AXES])

        # for i, axis in enumerate(['x', 'y', 'z']):
        #     self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
        #                         light=None,
        #                         solid=x if axis == 'x' else y if axis == 'y' else z,
        #                         dashed=None,
        #                         series_labels=[
        #                             "Motors", "Elevons", "Motor Derivative", "Body Rates"
        #                         ],
        #                         style_labels=[None, "Onboard", None],
        #                         title=f"Fx {axis.upper()}",
        #                         ylabel="Fx [N/kg/(rad/s)²]")


        # self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
        #                         light=None,
        #                         solid=fx_e_var[:3],
        #                         dashed=None,
        #                         series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
        #                         style_labels=[None, "Onboard", None],
        #                         title="Fx Error Variance",
        #                         ylabel="Variance")

        # self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
        #                         light=None,
        #                         solid=fx_lambda[:3],
        #                         dashed=None,
        #                         series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
        #                         style_labels=[None, "Onboard", None],
        #                         title="Fx Forgetting Factor",
        #                         ylabel="Forgetting Factor")

        M = 4
        for i, axis in enumerate(['p', 'q', 'r']):
            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                light=None,
                                solid=p[:M] if axis == 'p' else q[:M] if axis == 'q' else r[:M],
                                dashed=None,
                                true_values=[self.true[f'fx_{axis}_rls_x[{i}]'] for i in range(M)] if self.true is not None else None,
                                series_labels=[
                                    "Motors", "Elevons", "Motor Derivative", "Body Rates"
                                ],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
                                light=None,
                                solid=fx_e_var[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")
        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
                                light=None,
                                solid=fx_lambda[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

        if self.has_inertia_learning:
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 1]),
                                  light=None,
                                  solid=np.array([self.data[f'sigma_rls[{i}]'] for i in range(3)]) / 1000,
                                  dashed=None,
                                  true_values=[self.true[f'sigma_rls[{i}]'] for i in range(3)] if self.true is not None else None,
                                  series_labels=["Sigma X", "Sigma Y", "Sigma Z"],
                                  style_labels=[None, "Onboard", None],
                                  title="Principal Inertia Ratios",
                                  ylabel="$\\sigma$ [-]")

class IndiflightIndividualSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
    def __init__(self, data, Nr=0, Ns=0, true=None, name="System Identification Plotter -- Individual"):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns

        # check amount of actuators
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        self.define_layout(figsize=(12, 8), nrows=6, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        Nr = self.Nr
        Ns = self.Ns
        N = self.N

        dmax = 1e-3*np.array([self.data[f'motor_{i}_rls_x[0]'] for i in range(Nr,N)])
        d0 = 1e-3*np.array([self.data[f'motor_{i}_rls_x[1]'] for i in range(Nr,N)])
        delay = 1e-4*np.array([self.data[f'motor_{i}_rls_x[2]'] for i in range(Nr,N)])
        tau = np.array([self.data[f'motor_{i}_rls_x[3]'] for i in range(Nr,N)])
        # motor_e_var = np.array([self.data[f'motor_{i}_rls_e_var'] for i in range(Nr)])
        # motor_lambda = np.array([self.data[f'motor_{i}_rls_lambda'] for i in range(Nr)])

        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
                            light=None,
                            solid=dmax,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[0]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Scaler",
                            ylabel="Angle/cmd [rad/1]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                            light=None,
                            solid=d0,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[1]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Neutral Servo Angle",
                            ylabel="Angle [rad]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                            light=None,
                            solid=delay,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[2]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Delay",
                            ylabel="Delay [s]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[3, 0]),
                            light=None,
                            solid=tau,
                            dashed=None,
                            true_values=[self.true[f'motor_{i}_rls_x[3]'] for i in range(Nr,N)] if self.true is not None else None,
                            series_labels=[f"Servo {i}" for i in range(Ns)],
                            style_labels=[None, "Onboard", None],
                            title="Servo Time Constant",
                            ylabel="Time Constant [s]")

        pqr_range = list(range(16))

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5

        qd = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in range(8,10)]) * 1e-3 * 1e-0 * 1e-0

        p[8:10] *= 1e5 * 1e-3
        q[8:10] *= 1e5 * 1e-3
        r[8:10] *= 1e5 * 1e-3

        p[10] *= 1e5 * 1e-1
        q[10] *= 1e5 * 1e-1
        r[10] *= 1e5 * 1e-1

        # compute effectiveness matrices (unscaled)
        vel = self.data[[f'vel[{i}]' for i in range(3)]].to_numpy()
        d = np.array([self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)])
        dSig = Signal(self.t, d.T).filtfilt(type='lowpass', order=2, cutoff_hz=20.0)
        dDot = dSig.dot(order=1).y.T * 1
        dDotDot = dSig.dot(order=2).y.T * 0

        sd = np.sin(d - 0.)
        cd = np.cos(d - 0.)

        clw = p[:Nr]

        cmw = q[:Nr]
        cmd = q[Nr:N]
        cmwd = q[8:10]

        cnw = r[:Nr]
        cnd = r[Nr:N]
        cnwd = r[8:10]

        L = len(self.t)
        eff = np.zeros((6, 4, L))
        eff[0, :Nr] = 0.
        eff[1, :Nr] = 0.
        eff[2, :Nr] = 0.
        eff[3, :Nr] = clw
        eff[4, :Nr] = cmw + cmd * sd
        eff[5, :Nr] = cnw + cnd * sd

        eff[0, Nr:N] = 0.
        eff[1, Nr:N] = 0.
        eff[2, Nr:N] = 0.
        eff[3, Nr:N] = 0.
        eff[4, Nr:N] = cmd * cd
        eff[5, Nr:N] = cnd * cd

        eff *= 1e6

        good_signs = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [-1,-1,0,0],
            [1,-1,0,0],
            [-1,-1,-1,-1],
            [1,-1,-1,+1]
        ])

        # infer time when learning starts
        for i in range(L):
            # first time when any eff is nonzero
            if (eff[:, :, i] != 0).any():
                istart = i
                tstart = self.t[i]
                print(f"Learning starts at t={self.t[i]:.2f}s with inertial velocity x={vel[i,0]:.2f} m/s, y={vel[i,1]:.2f} m/s, z={vel[i,2]:.2f} m/s")
                tend = tstart + 0.5
                # find closest index to tend
                iend = np.argmin(np.abs(self.t - tend))
                print(f"Learning ends at t={self.t[iend]:.2f}s with inertial velocity x={vel[iend,0]:.2f} m/s, y={vel[iend,1]:.2f} m/s, z={vel[iend,2]:.2f} m/s")

                self.idx_start_learning = istart
                self.idx_end_learning = iend
                break


        # compute first time that roll and pitch signs are correct
        for i in range(istart+100, L):
            if (eff[3, 0:2, i]*good_signs[3, 0:2] > 0).all():
                print(f"Axis roll correct sign from t={self.t[i]-tstart:.2f}s after learning starts")
                break

        # and also first time when the signs stay correct (excluding last 2? seconds of the log)
        for i in range(istart+100, L-500):
            if (eff[3, 0:2, i:L-500]*good_signs[3, 0:2, np.newaxis] > 0).all():
                print(f"Axis roll correct sign from t={self.t[i]-tstart:.2f}s until end of log")
                break

        for i in range(istart+100, L):
            if (eff[4, 2:4, i]*good_signs[4, 2:4] > 0).all():
                print(f"Axis pitch correct sign from t={self.t[i]-tstart:.2f}s after learning starts")
                break

        for i in range(istart+100, L-300):
            if (eff[4, 2:4, i:L-500]*good_signs[4, 2:4, np.newaxis] > 0).all():
                print(f"Axis pitch correct sign from t={self.t[i]-tstart:.2f}s until end of log")
                break

        AXES = ['x', 'y', 'z', 'p', 'q', 'r']
        fx_e_var  = np.array([self.data[f'fx_{ax}_rls_e_var'] for ax in AXES])
        fx_lambda = np.array([self.data[f'fx_{ax}_rls_lambda'] for ax in AXES])

        # for i, axis in enumerate(['x', 'y', 'z']):
        #     self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
        #                         light=None,
        #                         solid=x if axis == 'x' else y if axis == 'y' else z,
        #                         dashed=None,
        #                         series_labels=[
        #                             "Motors", "Elevons", "Motor Derivative", "Body Rates"
        #                         ],
        #                         style_labels=[None, "Onboard", None],
        #                         title=f"Fx {axis.upper()}",
        #                         ylabel="Fx [N/kg/(rad/s)²]")


        # self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
        #                         light=None,
        #                         solid=fx_e_var[:3],
        #                         dashed=None,
        #                         series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
        #                         style_labels=[None, "Onboard", None],
        #                         title="Fx Error Variance",
        #                         ylabel="Variance")

        # self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
        #                         light=None,
        #                         solid=fx_lambda[:3],
        #                         dashed=None,
        #                         series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
        #                         style_labels=[None, "Onboard", None],
        #                         title="Fx Forgetting Factor",
        #                         ylabel="Forgetting Factor")

        M = 4
        for i, axis in enumerate(['p', 'q', 'r']):
            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                light=None,
                                solid=p[:4] if axis == 'p' else q[:4] if axis == 'q' else r[:4],
                                dashed=None,
                                true_values=[self.true[f'fx_{axis}_rls_x[{i}]'] for i in range(4)] if self.true is not None else None,
                                series_labels=[
                                    "Motor 1", "Motor 2", "Elevon 1", "Elevon 2"
                                ],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 2]),
                                light=None,
                                solid=qd,
                                dashed=None,
                                true_values=[self.true[f'fx_q_rls_x[{i}]'] for i in range(8,10)] if self.true is not None else None,
                                series_labels=[
                                    "Elevon 1", "Elevon 2"
                                ],
                                style_labels=[None, "Onboard", None],
                                title=f"Elevon Velocity Q",
                                ylabel="Qd [(Nm/kgm^2) / (rad/s)]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 2]),
                            light=None,
                            solid=r[8:10],
                            dashed=None,
                            true_values=[self.true[f'fx_r_rls_x[{i}]'] for i in range(8,10)] if self.true is not None else None,
                            series_labels=[
                                "Motor 1", "Motor 2"
                            ],
                            style_labels=[None, "Onboard", None],
                            title=f"Fx R Omega_dot",
                            ylabel="Fx [Nm/(kgm^2)/(rad/s/s)]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
                                light=None,
                                solid=fx_e_var[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")
        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
                                light=None,
                                solid=fx_lambda[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

        axis = self._find_axis(self.gs[5,1])
        if axis is not None:
            axis.set_ylim(0.95, 1.01)

        if self.has_inertia_learning:
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 1]),
                                  light=None,
                                  solid=np.array([self.data[f'sigma_rls[{i}]'] for i in range(3)]) / 1000,
                                  dashed=None,
                                  true_values=[self.true[f'sigma_rls[{i}]'] for i in range(3)] if self.true is not None else None,
                                  series_labels=["Sigma X", "Sigma Y", "Sigma Z"],
                                  style_labels=[None, "Onboard", None],
                                  title="Principal Inertia Ratios",
                                  ylabel="$\\sigma$ [-]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[3, 2]),
                              light=None,
                              solid=np.array([self.data[f'fx_{axis}_rls_x[10]'] for axis in ['p', 'q', 'r']]) / 1000,
                              dashed=None,
                              true_values=None,
                              series_labels=["Aero P", "Aero Q", "Aero R"],
                              style_labels=[None, "Onboard", None],
                              title="Aero Derivative Coefficients",
                              ylabel="Coefficient [Nm/(kgm²)/(rad/s)^2]")

        gains = np.array([self.data[f'learner_gains[{i}]'] for i in range(6)]) * 1e-1

        self._plot_timeseries(self.fig.add_subplot(self.gs[4:6, 2]),
                                light=None,
                                solid=gains,
                                dashed=None,
                                true_values=None,
                                series_labels=["R", "A", "Vh", "Ph", "Vv", "Pv"],
                                style_labels=[None, "Onboard", None],
                                title="Learner Gains",
                                ylabel="Gain [-]")

class IndiflightMoments(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for Moment analysis"""
    def __init__(self, data, Nr=0, Ns=0, true=None, name="System Identification Plotter -- Individual"):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns

        # check amount of actuators
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        # self.I = np.diag([6.5e-3, 2e-3, 6e-3])
        self.I = np.diag([5.73e-3, 1.35e-3, 5.43e-3])

        self.define_layout(figsize=(12, 8), nrows=4, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1, 1])

        self.plot()

    def _populate(self):
        gyro = np.array([self.data[f'gyroADCafterRpm[{i}]'].to_numpy() for i in range(3)])
        gyro_S = Signal(self.t, gyro.T)
        alpha_raw = gyro_S.dot(order=1).y.T

        # M = I * alpha + omega x (I * omega)
        M_raw = self.I @ alpha_raw  +  np.cross(gyro, self.I @ gyro, axis=0)

        M_raw_S = Signal(self.t, M_raw.T)
        M_raw_ff = M_raw_S.filtfilt(type='lowpass', order=2, cutoff_hz=30.0).y.T

        #%% offboard model
        phi = 3.297e-1
        Phi = np.array([
            [+3.106e-01,          0, +4.148e-02,          0, -1.613e-04,          0],
            [         0, +3.603e-02,          0, +1.355e-03,          0, -3.538e-03],
            [+4.148e-02,          0, +4.465e-02,          0, +1.951e-04,          0],
            [         0, +1.355e-03,          0, +8.290e-04,          0, +7.494e-04],
            [-1.613e-04,          0, +1.951e-04,          0, +4.093e-04,          0],
            [         0, -3.538e-03,          0, +7.494e-04,          0, +2.790e-03],
        ], dtype=np.float32)

        cd = np.array([-7.032e-07,          0,          0,          0, -1.835e-08, -6.047e-08], dtype=np.float32)
        cdd = np.array([         0,          0,          0,          0, -1.412e-03,          0], dtype=np.float32)
        cddd = np.array([         0,          0,          0,          0, -2.201e-05,          0], dtype=np.float32)
        d0 = np.array([-3.618e-01, -1.540e-01], dtype=np.float32)

        cww = np.array([2.203e-07, 0, -1.396e-06, 1.487e-07, 0, +2.247e-08], dtype=np.float32)
        cwd = np.zeros_like(cww)

        # get body velocities
        quat = self.data[[f'quat[{i}]' for i in [1,2,3,0]]].to_numpy()
        norms = np.linalg.norm(quat, axis=1)
        if (np.any(norms < 1e-6)):
            print("Warning: found near-zero quaternion norm, replacing with identity to avoid NaNs in rotation")
            quat[norms < 1e-6] = np.array([0,0,0,1])  # avoid NaNs
        rot = R.from_quat(quat)
        irot = rot.inv()

        vel = self.data[[f'vel[{i}]' for i in range(3)]].to_numpy()
        velB = irot.apply(vel.copy())

        # eta as concat of velB and gyro
        eta_B = np.vstack((velB.T, gyro))

        self.eta = np.sqrt( np.sum(velB**2, axis=1) + phi * np.sum(gyro.T**2, axis=1) )
        self.FM_aero = - self.eta * (Phi @ eta_B)

        self.FM_act = np.zeros_like(self.FM_aero)
        omega = np.array([self.data[f'omegaUnfiltered[{i}]'].to_numpy() for i in range(2)])
        d = np.array([self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)])
        dSig = Signal(self.t, d.T).filtfilt(type='lowpass', order=2, cutoff_hz=20.0)
        dDot = dSig.dot(order=1).y.T * 1
        dDotDot = dSig.dot(order=2).y.T * 0
        ww = omega**2
        wwDd = ww * (d.T - d0).T
        wwsum = np.sum(ww, axis=0)
        wwdiff = ww[0] - ww[1]
        wwdsum = np.sum(wwDd, axis=0)
        wwddiff = wwDd[0] - wwDd[1]
        ddsum = np.sum(dDot, axis=0)
        dddsum = np.sum(dDotDot, axis=0)
        dddiff = np.zeros_like(wwsum)
        wddiff = np.zeros_like(wwsum)

        self.FM_act[0] = cww[0]*wwsum   +  0              +  cd[0]*wwdsum   +  cdd[0]*ddsum
        self.FM_act[1] = 0
        self.FM_act[2] = cww[2]*wwsum
        self.FM_act[3] = cww[3]*wwdiff
        self.FM_act[4] = cww[4]*wwsum   +  0              +  cd[4]*wwdsum   +  cdd[4]*ddsum   +  cddd[4]*dddsum
        self.FM_act[5] = cww[5]*wwdiff  +  cwd[5]*wddiff  +  cd[5]*wwddiff  +  cdd[5]*dddiff

        self.FM = self.FM_aero + self.FM_act

        #%% onboard model

        Nr = self.Nr
        Ns = self.Ns
        N = self.Nr + self.Ns
        pqr_range = list(range(4))

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5

        qd = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in range(4,6)]) * 1e-3 * 1e-0 * 1e-0

        AXES = ['x', 'y', 'z', 'p', 'q', 'r']

        final_idx = -500

        # motor and elevon moments
        Lact_online = np.sum(p[:2]*ww + p[2:4]*ww*d, axis=0) * self.I[0,0]
        Mact_online = np.sum(q[:2]*ww + q[2:4]*ww*d + qd*dDot, axis=0) * self.I[1,1]
        Nact_online = np.sum(r[:2]*ww + r[2:4]*ww*d, axis=0) * self.I[2,2]
        act_online = [Lact_online, Mact_online, Nact_online]

        # with final model
        Lact_final = (p[:2, final_idx]@ww + p[2:4, final_idx]@(ww*d)) * self.I[0,0]
        Mact_final = (q[:2, final_idx]@ww + q[2:4, final_idx]@(ww*d) + qd[:, final_idx] @ dDot) * self.I[1,1]
        Nact_final = (r[:2, final_idx]@ww + r[2:4, final_idx]@(ww*d)) * self.I[2,2]
        act_final = [Lact_final, Mact_final, Nact_final]


        #%% online and final aero moments
        paero = -np.array([self.data[f'fx_p_rls_x[6]']]).squeeze() / 1000 * 1e-1
        qaero = -np.array([self.data[f'fx_q_rls_x[6]']]).squeeze() / 1000 * 1e-1
        raero = -np.array([self.data[f'fx_r_rls_x[6]']]).squeeze() / 1000 * 1e-1
        Laero_online = -self.eta * gyro[0] * self.I[0,0] * paero
        Maero_online = -self.eta * gyro[1] * self.I[1,1] * qaero
        Naero_online = -self.eta * gyro[2] * self.I[2,2] * raero
        aero_online = [Laero_online, Maero_online, Naero_online]

        Laero_final = -self.eta * gyro[0] * self.I[0,0] * paero[final_idx]
        Maero_final = -self.eta * gyro[1] * self.I[1,1] * qaero[final_idx]
        Naero_final = -self.eta * gyro[2] * self.I[2,2] * raero[final_idx]
        aero_final = [Laero_final, Maero_final, Naero_final]

        act_aero_online = [act_online[i] + aero_online[i] for i in range(3)]
        act_aero_final = [act_final[i] + aero_final[i] for i in range(3)]

        #%% make plots

        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            self._plot_timeseries(self.fig.add_subplot(self.gs[0, i]),
                                light=None,
                                solid=[M_raw_ff[i], act_aero_final[i], self.FM[3+i]],
                                dashed=[None, act_aero_online[i], None],
                                series_labels=["Measured", "Onboard", "Phi"],
                                style_labels=[None, "Posteriori", "Online"],
                                title=f"Total Moment Estimation -- {axis}",
                                ylabel="Moment [Nm]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[1, i]),
                                light=[None, act_online[i], None],
                                solid=[None, act_final[i], self.FM_act[3+i]],
                                dashed=None,
                                series_labels=[None, "Onboard", "Phi"],
                                style_labels=["Online", "Posteriori", None],
                                title=f"Actuator Moment Estimation -- {axis}",
                                ylabel="Moment [Nm]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[2, i]),
                                light=[None, aero_online[i], None],
                                solid=[None, aero_final[i], self.FM_aero[3+i]],
                                dashed=None,
                                series_labels=[None, "Onboard", "Phi"],
                                style_labels=["Online", "Posteriori", None],
                                title=f"Aero Moment Estimation -- {axis}",
                                ylabel="Moment [Nm]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[3, i]),
                                light=[None, None, self.FM_aero[3+i]],
                                solid=[M_raw_ff[i], None, self.FM[3+i]],
                                dashed =[None, None, self.FM_act[3+i]],
                                series_labels=["Measured", None, "Phi"],
                                style_labels=["Aero", "Total", "Actuators"],
                                title=f"Moment Breakdown -- {axis}",
                                ylabel="Moment [Nm]")

class IndiflightFxSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
    def __init__(self, data, Nr=0, Ns=0, true=None, name="System Identification Plotter -- Servo"):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        # do some investigation
        # self.has_fx_learning = 'fx_x_rls_x[0]' in self.data.columns
        # self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        # self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns
        # self.has_servo = 'servo_feedback[0]' in self.data.columns

        # check amount of actuators
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        self.define_layout(figsize=(12, 8), nrows=6, ncols=4,
                           width_ratios=[1, 1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        Nr = self.Nr

        # fx learning data
        if self.has_extended_fx_learning:
            # we have the extended logging (like in simulation)
            pqr_range = list(range(Nr)) + list(range(2*Nr, 3*Nr))
        else:
            pqr_range = list(range(2*Nr))

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)])
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)])
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)])
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range])
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range])
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range])

        AXES = ['x', 'y', 'z', 'p', 'q', 'r']
        fx_e_var  = np.array([self.data[f'fx_{ax}_rls_e_var'] for ax in AXES])
        fx_lambda = np.array([self.data[f'fx_{ax}_rls_lambda'] for ax in AXES])

        for i, axis in enumerate(['x', 'y', 'z']):
            if self.craft == "quadrotor":
                self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                    light=None,
                                    solid=x if axis == 'x' else y if axis == 'y' else z,
                                    dashed=None,
                                    series_labels=[f"Motor {j}" for j in range(Nr)],
                                    style_labels=[None, "Onboard", None],
                                    title=f"Fx {axis.upper()}",
                                    ylabel="Fx [N/kg/(rad/s)²]")
            elif self.craft == "tailsitter":
                self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                    light=None,
                                    solid=x if axis == 'x' else y if axis == 'y' else z,
                                    dashed=None,
                                    series_labels=[
                                        "Motors", "Elevons", "Motor Derivative", "Body Rates"
                                    ],
                                    style_labels=[None, "Onboard", None],
                                    title=f"Fx {axis.upper()}",
                                    ylabel="Fx [N/kg/(rad/s)²]")


        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 1]),
                                light=None,
                                solid=fx_e_var[:3],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")

        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 1]),
                                light=None,
                                solid=fx_lambda[:3],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[:3]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

        for i, axis in enumerate(['p', 'q', 'r']):
            if self.craft == "quadrotor":
                self._plot_timeseries(self.fig.add_subplot(self.gs[i, 2]),
                                    light=None,
                                    solid=p[:Nr] if axis == 'p' else q[:Nr] if axis == 'q' else r[:Nr],
                                    dashed=None,
                                    series_labels=[f"Motor {j}" for j in range(Nr)],
                                    style_labels=[None, "Onboard", None],
                                    title=f"Fx {axis.upper()}",
                                    ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")
            elif self.craft == "tailsitter":
                self._plot_timeseries(self.fig.add_subplot(self.gs[i, 2]),
                                    light=None,
                                    solid=p[:Nr] if axis == 'p' else q[:Nr] if axis == 'q' else r[:Nr],
                                    dashed=None,
                                    series_labels=[
                                        "Motors", "Elevons", "Motor Derivative", "Body Rates"
                                    ],
                                    style_labels=[None, "Onboard", None],
                                    title=f"Fx {axis.upper()}",
                                    ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 3]),
                                light=None,
                                solid=p[Nr:] if axis == 'p' else q[Nr:] if axis == 'q' else r[Nr:],
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(Nr)],
                                style_labels=[None, "Onboard", None],
                                title=f"Fx {axis.upper()}",
                                ylabel="Fx [Nm/(kgm^2)/(rad/s²)]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[4, 2]),
                                light=None,
                                solid=fx_e_var[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Error Variance",
                                ylabel="Variance")
        self._plot_timeseries(self.fig.add_subplot(self.gs[5, 2]),
                                light=None,
                                solid=fx_lambda[3:],
                                dashed=None,
                                series_labels=[f"Fx {ax.upper()}" for ax in AXES[3:]],
                                style_labels=[None, "Onboard", None],
                                title="Fx Forgetting Factor",
                                ylabel="Forgetting Factor")

        if self.has_inertia_learning:
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 2]),
                                  light=None,
                                  solid=np.array([self.data[f'sigma_rls[{i}]'] for i in range(3)]) / 1000,
                                  dashed=None,
                                  series_labels=["Sigma X", "Sigma Y", "Sigma Z"],
                                  style_labels=[None, "Onboard", None],
                                  title="Principal Inertia Ratios",
                                  ylabel="$\\sigma$ [-]")

class IndiflightEffectiveness(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for Effectiveness analysis"""
    def __init__(self, data, Nr=0, Ns=0, true=None, name="System Identification Plotter -- Effectiveness", scheduled=True):
        # extract time and intialize base class
        self.data = data
        self.true = true
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.scheduled = scheduled

        # check amount of actuators
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        self.define_layout(figsize=(8, 8), nrows=1, ncols=1,
                           width_ratios=[1],
                           height_ratios=[1])

        self.plot()

    def _populate(self):
        ax = self.fig.add_subplot(self.gs[0, 0])
        testfx = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [-1, -1, -1, -1],
            [1,1,-1,-1],
            [1,-1,1,-1],
            [1,-1,-1,1],
        ])

        im = ax.imshow(testfx, cmap='viridis', aspect='auto')
        ax.set_yticks(np.arange(testfx.shape[0]))
        ax.set_yticklabels(['Fx', 'Fy', 'Fz', 'Roll', 'Pitch', 'Yaw'])
        ax.set_xticks(np.arange(testfx.shape[1]))
        ax.set_xticklabels([f'Actuator {i}' for i in range(testfx.shape[1])])
        # plt.colorbar(im, ax=ax)
        ax.set_title("Control Effectiveness Matrix Example")

    def showAtTime(self, time):
        """Show effectiveness values at given time"""
        idx = np.searchsorted(self.t, time)
        if idx >= len(self.t):
            idx = len(self.t) - 1

        ax = self.fig.axes[0]
        ax.cla()

        Nr = self.Nr
        Ns = self.Ns
        N = self.Nr + self.Ns


        a = np.array([self.data[f'motor_{i}_rls_x[0]'] for i in range(Nr)])
        b = np.array([self.data[f'motor_{i}_rls_x[1]'] for i in range(Nr)])
        widle = np.array([self.data[f'motor_{i}_rls_x[2]'] for i in range(Nr)])
        tau = np.array([self.data[f'motor_{i}_rls_x[3]'] for i in range(Nr)])
        wmax = a + b
        kappa = np.zeros_like(wmax)
        kappa[a+b > 0] = a[a+b > 0] / (a[a+b > 0] + b[a+b > 0])


        pqr_range = list(range(4))

        omega = np.array([self.data[f'omegaUnfiltered[{i}]'].to_numpy() for i in range(2)])
        d = np.array([self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)])
        dSig = Signal(self.t, d.T).filtfilt(type='lowpass', order=2, cutoff_hz=20.0)
        dDot = dSig.dot(order=1).y.T * 1
        dDotDot = dSig.dot(order=2).y.T * 0

        x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(Nr)]) * 1e-3 * 1e-1 * 1e-5
        p = np.array([self.data[f'fx_p_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        q = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5
        r = np.array([self.data[f'fx_r_rls_x[{i}]'] for i in pqr_range]) * 1e-3 * 1e-0 * 1e-5

        sd = np.sin(d[:, idx] - 0.)
        cd = np.cos(d[:, idx] - 0.)

        clw = p[:Nr, idx]

        cmw = q[:Nr, idx]
        cmd = q[Nr:, idx]
        cmwd = q[4:6, idx]

        cnw = r[:Nr, idx]
        cnd = r[Nr:, idx]
        cnwd = r[4:6, idx]

        qd = np.array([self.data[f'fx_q_rls_x[{i}]'] for i in range(4,6)]) * 1e-3 * 1e-0 * 1e-0

        eff = np.zeros((6, 4))
        eff[0, :Nr] = x[:Nr, idx]
        eff[1, :Nr] = y[:Nr, idx]
        eff[2, :Nr] = z[:Nr, idx]
        eff[3, :Nr] = clw
        eff[4, :Nr] = cmw + cmd * sd
        eff[5, :Nr] = cnw + cnd * sd

        if self.scheduled:
            eff[:, :Nr] *= wmax[:, idx]**2

        eff[0, Nr:] = 0.
        eff[1, Nr:] = 0.
        eff[2, Nr:] = 0.
        eff[3, Nr:] = 0.
        eff[4, Nr:] = cmd * cd
        eff[5, Nr:] = cnd * cd

        if self.scheduled:
            eff[:, Nr:] *= 100./180. * np.pi * omega[:, idx]**2

        if not self.scheduled:
            eff *= 1e6

        good_signs = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [-1,-1,0,0],
            [1,-1,0,0],
            [-1,-1,-1,-1],
            [1,-1,-1,+1]
        ])

        import matplotlib.colors as mcolors
        from matplotlib.patches import Ellipse

        lim = np.max(np.abs(eff))
        if lim > 0:
            norm = mcolors.TwoSlopeNorm(vmin=-lim, vcenter=0, vmax=lim)
        else:
            norm = mcolors.TwoSlopeNorm(vmin=-1, vcenter=0, vmax=1)

        im = ax.imshow(eff, cmap='viridis', norm=norm, aspect='auto')
        ax.set_yticks(np.arange(eff.shape[0]))
        ax.set_yticklabels(['Fx', 'Fy', 'Fz', 'Roll', 'Pitch', 'Yaw'])
        ax.set_xticks(np.arange(eff.shape[1]))
        ax.set_xticklabels(["Motor 1", "Motor 2", "Elevon 1", "Elevon 2"])
        # ax.set_xticklabels([f'Actuator {i}' for i in range(eff.shape[1])])
        ax.set_title(f"Control Effectiveness Matrix at t={time:.2f}s -- {'Scheduled' if self.scheduled else 'Unscheduled'}")

        # Loop over data dimensions and create text annotations.
        for i in range(len(eff)):
            for j in range(len(eff[0])):
                text = ax.text(j, i, f"{eff[i, j]:.2f}",
                               ha="center", va="center", color="w")
                if (good_signs*eff)[i, j] < 0:
                    # show ellipse if sign is wrong
                    ellipse = Ellipse(
                        (j, i),        # center (same as text)
                        width=0.9,     # adjust to taste
                        height=0.5,
                        fill=False,
                        edgecolor='red',
                        linewidth=2,
                        zorder=2
                    )

                    ax.add_patch(ellipse)

        # self.fig.canvas.draw_idle()
        self.fig.canvas.draw()

    def mouseHoverCallback(self, event):
        """Override to show effectiveness values on hover"""
        if event.xdata is not None:
            self.showAtTime(event.xdata)

class IndiflightViewport(Viewport):
    """Thin wrapper: extract series from log and intialize base class"""

    def __init__(self, craft: Craft3D, data, Nr=0, Ns=0, follow=False, interpolation="previous", title="Viewport"):
        self.data = data
        self.Nr = Nr
        self.Ns = Ns
        self.N = self.Nr + self.Ns

        # check if these numbers match the craft definition
        if self.Nr != len(craft.rotors) or self.Ns != len(craft.surfaces):
            raise ValueError(f"Craft definition has {len(craft.rotors)} rotors and {len(craft.surfaces)} servos, but log data has {self.Nr} rotors and {self.Ns} servos.")

        # get into format for base class
        rotor = np.zeros((len(self.data), self.Nr, 3))
        rotorSet = np.zeros((len(self.data), self.Nr, 3))
        rotor[:, :, 0] = self.data[[f'omega[{i}]' for i in range(self.Nr)]].to_numpy()
        rotor[:, :, 0] /= np.max(rotor[:, :, 0]) + 1e-6  # normalize for visualization
        rotorSet[:, :, 0] = self.data[[f'u[{i}]' for i in range(self.Nr) ]].to_numpy()

        if 'servo_feedback[0]' in self.data.columns:
            surface = self.data[[f'servo_feedback[{i}]' for i in range(self.Ns)]].to_numpy()
        else:
            surface = None
        surfaceSet = self.data[[f'u[{i}]' for i in range(self.Nr,self.Ns+self.Nr)]].to_numpy() * 100 * np.pi / 180.0  # scale to radians for visualization

        # rotate IMU acceleration to global frame
        if 'accSmooth[0]' in self.data.columns:
            from scipy.spatial.transform import Rotation as R
            accB = self.data[[f'accSmooth[{i}]' for i in range(3)]].to_numpy()
            quat = self.data[[f'quat[{i}]' for i in [1,2,3,0]]].to_numpy()
            norms = np.linalg.norm(quat, axis=1)
            if (np.any(norms < 1e-6)):
                print("Warning: found near-zero quaternion norm, replacing with identity to avoid NaNs in rotation")
                quat[norms < 1e-6] = np.array([0,0,0,1])  # avoid NaNs
            rot = R.from_quat(quat)
            accI = rot.apply(accB.copy()) + np.array([0, 0, 9.81])

        super().__init__(
            craft,
            time=self.data['timeS'].to_numpy(),
            att=self.data[[f'quat[{i}]' for i in range(4)]].to_numpy(),
            attSet=self.data[[f'quatSp[{i}]' for i in range(4)]].to_numpy(),
            attMeas=self.data[[f'localQuat[{i}]' for i in range(4)]].to_numpy() if 'localQuat[0]' in self.data.columns else None,
            pos=self.data[[f'pos[{i}]' for i in range(3)]].to_numpy() if 'pos[0]' in self.data.columns else None,
            posSet=self.data[[f'posSp[{i}]' for i in range(3)]].to_numpy() if 'posSp[0]' in self.data.columns else None,
            posMeas=self.data[[f'localPos[{i}]' for i in range(3)]].to_numpy() if 'localPos[0]' in self.data.columns else None,
            vel=self.data[[f'vel[{i}]' for i in range(3)]].to_numpy() if 'vel[0]' in self.data.columns else None,
            velSet=self.data[[f'velSp[{i}]' for i in range(3)]].to_numpy() if 'velSp[0]' in self.data.columns else None,
            velMeas=self.data[[f'localVel[{i}]' for i in range(3)]].to_numpy() if 'localVel[0]' in self.data.columns else None,
            acc=accI if 'accSmooth[0]' in self.data.columns else None,
            accSet=self.data[[f'accSp[{i}]' for i in range(3)]].to_numpy() if 'accSp[0]' in self.data.columns else None,
            accMeas=None, # not measured
            rotor=rotor,
            rotorSet=rotorSet,
            surface=surface,
            surfaceSet=surfaceSet,
            follow=follow,
            interpolation=interpolation,
            title=title
        )

if __name__ == "__main__":
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    from indiflight_log_tools import IndiflightLog
    import matplotlib.pyplot as plt

    from pyFlightPlotter import BlittedCursor, Quadrotor, Tailsitter

    parser = ArgumentParser(description="Analyse onboard ID data from a log file.",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
    parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
    parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                        help="Crop the log to the given time range (in seconds).")
    parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")
    parser.add_argument("--type", type=str, default="multirotor", choices=["tailsitter", "multirotor"],
                        help="Type of craft for visualization.")
    parser.add_argument("--follow", action="store_true", help="Follow the craft in the viewport.")

    args = parser.parse_args()

    if args.name is None:
        args.name = args.logfile.split("/")[-1].split(".")[0]

    log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
    if args.crop:
        log.data, _ = log.crop(args.crop[0], args.crop[1])

    if args.type == "tailsitter":
        fplt = IndiflightPlotter(log.data, Nr=2, Ns=2, name=f"{args.name} -- Flight Data -- {log.parameters['Firmware revision']}")
        craft = Tailsitter()
    elif args.type == "multirotor":
        fplt = IndiflightPlotter(log.data, Nr=4, Ns=0, name=f"{args.name} -- Flight Data -- {log.parameters['Firmware revision']}")
        craft = Quadrotor()
    else:
        raise ValueError(f"Unknown craft type: {args.type}")

    cursor = BlittedCursor(fplt.all_axes, sharex=True)

    pplt = IndiflightViewport(craft,
                              log.data,
                              Nr=fplt.Nr,
                              Ns=fplt.Ns,
                              follow=args.follow,
                              interpolation="previous",
                              title=f"{args.name} -- Onboard ID Analysis -- {log.parameters['Firmware revision']}")

    fplt.connect_viewport(pplt)

    plt.show()
