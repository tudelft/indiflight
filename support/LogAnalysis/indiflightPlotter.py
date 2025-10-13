import numpy as np

from pyFlightPlotter import FlightPlotterBase, Viewport, Craft3D

class IndiflightPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for general Indiflight analysis"""
    def __init__(self, data, name="Flight Plotter"):
        # extract time and intialize base class
        self.data = data
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.define_layout(figsize=(12, 8), nrows=3, ncols=3,
                           width_ratios=[1, 1, 1],
                           height_ratios=[1, 1, 1])

        self.plot()

    def _populate(self):
        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 0]),
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
                         title="Body Accelerations",
                         ylabel="Acceleration [m/s²]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[1, 0]),
                         light=None,
                         solid=[self.data[f'alpha[{i}]'].to_numpy() for i in range(3)],
                         dashed=[self.data[f'alphaSp[{i}]'].to_numpy() for i in range(3)],
                         series_labels=["roll", "pitch", "yaw"],
                         style_labels=[None, "Filtered", "Setpoint"],
                         title="Body Angular Accel.",
                         ylabel="Angular Accel. [rad/s²]")

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 0]),
                         light=[self.data[f'omegaUnfiltered[{i}]'].to_numpy() for i in range(4)],
                         solid=[self.data[f'omega[{i}]'].to_numpy() for i in range(4)],
                         dashed=None,
                         series_labels=[f"Motor {i}" for i in [1,2,3,4]],
                         style_labels=["Raw", "Onboard filt.", None],
                         title="Motor Speeds",
                         ylabel="Motor Speed [rad/s]",
                         ylimits=(-100, None))

        self._plot_timeseries(self.fig.add_subplot(self.gs[0, 1]),
                         light=None,
                         solid=[self.data[f'rcCommand[{i}]'].to_numpy() for i in range(4)],
                         dashed=None,
                         series_labels=["RC Roll", "RC Pitch", "RC Yaw", "RC Throttle"],
                         style_labels=[None, "Raw", None],
                         title="RC Commands",
                         ylabel="RC Command",
                         ylimits=(-1.1, +1.1))

        N = 4 if 'motor[2]' in self.data.columns else 2
        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 1]),
                         light=[self.data[f'motor[{i}]'].to_numpy() for i in range(N)],
                         solid=[self.data[f'u_state[{i}]'].to_numpy() for i in range(N)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(N)],
                         series_labels=[f"Motor {str(i)}" for i in range(1,N+1)],
                         style_labels=["Final command", "Est. state", "Command"],
                         title="Motor Commands",
                         ylabel="Motor Commands [-]",
                         ylimits=(-0.05, 1.05))

        if 'servo_feedback[0]' in self.data.columns:
            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 2]),
                         light=None,
                         solid=[self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(2)],
                         series_labels=[f"Servo {i}" for i in [1,2]],
                         style_labels=[None, "Est. state", "Command"],
                         title="Servo State",
                         ylabel="Servo State [rad]",
            )

class IndiflightSysIdPlotter(FlightPlotterBase):
    """Wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
    def __init__(self, data, name="System Identification Plotter", craft="quadrotor"):
        # extract time and intialize base class
        self.data = data
        t = self.data['timeS'].to_numpy()
        super().__init__(t, name)

        self.craft = craft

        self.define_layout(figsize=(12, 8), nrows=6, ncols=4,
                           width_ratios=[1, 1, 1, 1],
                           height_ratios=[1, 1, 1, 1, 1, 1])

        self.plot()

    def _populate(self):
        N = 4

        # motor learning data
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

        # fx learning data
        if 'fx_r_rls_x[15]' in self.data.columns:
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

        if 'sigma_rls[0]' in self.data.columns:
            self._plot_timeseries(self.fig.add_subplot(self.gs[3, 2]),
                                  light=None,
                                  solid=np.array([self.data[f'sigma_rls[{i}]'] for i in range(3)]) / 1000,
                                  dashed=None,
                                  series_labels=["Sigma X", "Sigma Y", "Sigma Z"],
                                  style_labels=[None, "Onboard", None],
                                  title="Principal Inertia Ratios",
                                  ylabel="$\\sigma$ [-]")

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

class IndiflightViewport(Viewport):
    """Thin wrapper: extract series from log and intialize base class"""

    def __init__(self, craft: Craft3D, data, follow=False, interpolation="previous", title="Viewport"):

        super().__init__(
            craft,
            time=data['timeS'].to_numpy(),
            att=data[[f'quat[{str(i)}]' for i in range(4)]].to_numpy(),
            attSet=data[[f'quatSp[{str(i)}]' for i in range(4)]].to_numpy(),
            attMeas=data[[f'localQuat[{str(i)}]' for i in range(4)]].to_numpy() if 'localQuat[0]' in data.columns else None,
            pos=data[[f'pos[{str(i)}]' for i in range(3)]].to_numpy() if 'pos[0]' in data.columns else None,
            posSet=data[[f'posSp[{str(i)}]' for i in range(3)]].to_numpy() if 'posSp[0]' in data.columns else None,
            posMeas=data[[f'localPos[{str(i)}]' for i in range(3)]].to_numpy() if 'localPos[0]' in data.columns else None,
            vel=data[[f'vel[{str(i)}]' for i in range(3)]].to_numpy() if 'vel[0]' in data.columns else None,
            velSet=data[[f'velSp[{str(i)}]' for i in range(3)]].to_numpy() if 'velSp[0]' in data.columns else None,
            velMeas=data[[f'localVel[{str(i)}]' for i in range(3)]].to_numpy() if 'localVel[0]' in data.columns else None,
            acc=data[[f'acc[{str(i)}]' for i in range(3)]].to_numpy() if 'acc[0]' in data.columns else None,
            accSet=data[[f'accSp[{str(i)}]' for i in range(3)]].to_numpy() if 'accSp[0]' in data.columns else None,
            accMeas=None, # not measured
            follow=follow,
            interpolation=interpolation,
            title=title
        )
