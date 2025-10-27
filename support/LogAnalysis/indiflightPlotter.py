import numpy as np

from pyFlightPlotter import FlightPlotterBase, Viewport, Craft3D

class IndiflightPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for general Indiflight analysis"""
    def __init__(self, data, name="Flight Plotter"):
        # extract time and intialize base class
        self.data = data
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        # check amount of rotors
        self.Nr = 0
        for i in range(8):
            if f'motor[{i}]' not in self.data.columns or (self.data[f'motor[{i}]'] == 0.).all():
                break
            self.Nr += 1

        # check amount of servos
        self.Ns = -self.Nr
        for i in range(16):
            if f'u[{i}]' not in self.data.columns or (self.data[f'u[{i}]'] == 0.).all():
                break
            self.Ns += 1

        # check for fields
        self.has_pos = 'pos[0]' in self.data.columns
        self.has_servo_feedback = 'servo_feedback[0]' in self.data.columns

        self.define_layout(figsize=(12, 8), nrows=4, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1, 1])

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
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 1]),
                         light=None,
                         solid=[self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(self.Ns)],
                         dashed=None,
                         series_labels=[f"Servo {i}" for i in range(1,self.Ns+1)],
                         style_labels=[None, "Unfiltered state", None],
                         title="Servo State",
                         ylabel="Servo State [rad]",
            )

        from scipy.spatial.transform import Rotation as R
        quat = self.data[[f'quat[{i}]' for i in [1,2,3,0]]].to_numpy()
        quat[np.linalg.norm(quat, axis=1) < 1e-6] = np.array([0,0,0,1])  # avoid NaNs
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

            velMeasB = irot.apply(velMeas) if velMeas is not None else None
            velB = irot.apply(vel)
            velSpB = irot.apply(velSp)


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
            accI = rot.apply(accB) + np.array([0, 0, 9.81])

            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                             light=None,
                             solid=[accI[:, i] for i in range(3)],
                             dashed=[self.data[f'accSp[{i}]'].to_numpy() for i in range(3)],
                             series_labels=["X", "Y", "Z"],
                             style_labels=[None, "Estimated", "Setpoint"],
                             title="Acceleration Global",
                             ylabel="Acceleration [m/s²]")

class IndiflightSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
    def __init__(self, data, name="System Identification Plotter", craft="quadrotor"):
        # extract time and intialize base class
        self.data = data
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.craft = craft

        # do some investigation
        self.has_motor_learning = 'motor_0_rls_x[0]' in self.data.columns
        self.has_fx_learning = 'fx_x_rls_x[0]' in self.data.columns
        self.has_extended_fx_learning = 'fx_r_rls_x[15]' in self.data.columns
        self.has_inertia_learning = 'sigma_rls[0]' in self.data.columns
        # self.has_servo = 'servo_feedback[0]' in self.data.columns

        # check amount of actuators
        self.N = 0
        for i in range(8):
            if f'u[{i}]' not in self.data.columns or (self.data[f'u[{i}]'] == 0.).all():
                break
            self.N += 1

        if self.N == 0:
            raise ValueError("No spinning actuators found in the log data!")

        self.define_layout(figsize=(12, 8), nrows=6, ncols=4,
                           width_ratios=[1, 1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        N = self.N

        # motor learning data
        if self.has_motor_learning:
            a = np.array([self.data[f'motor_{i}_rls_x[0]'] for i in range(N)])
            b = np.array([self.data[f'motor_{i}_rls_x[1]'] for i in range(N)])
            widle = np.array([self.data[f'motor_{i}_rls_x[2]'] for i in range(N)])
            tau = np.array([self.data[f'motor_{i}_rls_x[3]'] for i in range(N)])
            wmax = a + b
            kappa = np.zeros_like(wmax)
            kappa[a+b > 0] = a[a+b > 0] / (a[a+b > 0] + b[a+b > 0])

            motor_e_var = np.array([self.data[f'motor_{i}_rls_e_var'] for i in range(N)])
            motor_lambda = np.array([self.data[f'motor_{i}_rls_lambda'] for i in range(N)])


            self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
                                light=None,
                                solid=wmax,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Max Motor Speed",
                                ylabel="Angular Rate [rad/s]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                                light=None,
                                solid=widle,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Idle Motor Speed",
                                ylabel="Angular Rate [rad/s]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                                light=None,
                                solid=tau,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Motor Time Constant",
                                ylabel="Time Constant [s]")

            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 0]),
                                light=None,
                                solid=kappa,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Motor Kappa",
                                ylabel="Kappa [rad/s]")
        
            self._plot_timeseries(self.fig.add_subplot(self.gs[4, 0]),
                                light=None,
                                solid=motor_e_var,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Motor Error Variance",
                                ylabel="Variance")
        
            self._plot_timeseries(self.fig.add_subplot(self.gs[5, 0]),
                                light=None,
                                solid=motor_lambda,
                                dashed=None,
                                series_labels=[f"Motor {i}" for i in range(N)],
                                style_labels=[None, "Onboard", None],
                                title="Motor Forgetting Factor",
                                ylabel="Forgetting Factor")

        if self.has_fx_learning:
            # fx learning data
            if self.has_extended_fx_learning:
                # we have the extended logging (like in simulation)
                pqr_range = list(range(N)) + list(range(2*N, 3*N))
            else:
                pqr_range = list(range(2*N))

            x = np.array([self.data[f'fx_x_rls_x[{i}]'] for i in range(N)])
            y = np.array([self.data[f'fx_y_rls_x[{i}]'] for i in range(N)])
            z = np.array([self.data[f'fx_z_rls_x[{i}]'] for i in range(N)])
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
                                        series_labels=[f"Motor {j}" for j in range(N)],
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
                                        solid=p[:N] if axis == 'p' else q[:N] if axis == 'q' else r[:N],
                                        dashed=None,
                                        series_labels=[f"Motor {j}" for j in range(N)],
                                        style_labels=[None, "Onboard", None],
                                        title=f"Fx {axis.upper()}",
                                        ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")
                elif self.craft == "tailsitter":
                    self._plot_timeseries(self.fig.add_subplot(self.gs[i, 2]),
                                        light=None,
                                        solid=p[:N] if axis == 'p' else q[:N] if axis == 'q' else r[:N],
                                        dashed=None,
                                        series_labels=[
                                            "Motors", "Elevons", "Motor Derivative", "Body Rates"
                                        ],
                                        style_labels=[None, "Onboard", None],
                                        title=f"Fx {axis.upper()}",
                                        ylabel="Fx [Nm/(kgm^2)/(rad/s)²]")

                self._plot_timeseries(self.fig.add_subplot(self.gs[i, 3]),
                                    light=None,
                                    solid=p[N:] if axis == 'p' else q[N:] if axis == 'q' else r[N:],
                                    dashed=None,
                                    series_labels=[f"Motor {j}" for j in range(N)],
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


class IndiflightViewport(Viewport):
    """Thin wrapper: extract series from log and intialize base class"""

    def __init__(self, craft: Craft3D, data, follow=False, interpolation="previous", title="Viewport"):
        self.data = data

        # check amount of rotors
        self.Nr = 0
        for i in range(8):
            if f'motor[{i}]' not in self.data.columns or (self.data[f'motor[{i}]'] == 0.).all():
                break
            self.Nr += 1

        # check amount of servos
        self.Ns = -self.Nr
        for i in range(16):
            if f'u[{i}]' not in self.data.columns or (self.data[f'u[{i}]'] == 0.).all():
                break
            self.Ns += 1

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
            quat[np.linalg.norm(quat, axis=1) < 1e-6] = np.array([0,0,0,1])  # avoid NaNs
            rot = R.from_quat(quat)
            accI = rot.apply(accB) + np.array([0, 0, 9.81])

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
