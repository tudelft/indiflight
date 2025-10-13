import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

COLORS = plt.rcParams['axes.prop_cycle'].by_key()['color']

local_rc = plt.rcParams.copy()
local_rc.update({
#    "text.usetex": True,
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 10,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 10,
    "axes.titlesize": 14,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper left',
    "legend.fontsize": 8,
    'figure.subplot.bottom': 0.06,
    'figure.subplot.left': 0.05,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.925,
    'figure.subplot.wspace': 0.3,
    'figure.subplot.hspace': 0.433,
})

plt.rcParams.update(local_rc)

class BlittedCursor(object):
    def __init__(self, axes, sharex=True):
        self.axes = axes
        self.backgrounds = []
        self.cursors = []

        ax0 = self.axes[0]

        for i, ax in enumerate(self.axes):
            if i > 0 and sharex:
                ax.sharex(ax0)

            lines = ax.get_lines()
            if lines:
                min_x = min(min(line.get_xdata()) for line in lines)
                self.cursors.append(ax.axvline(x=min_x,
                                               color='black',
                                               linestyle='--',
                                               lw=0.8,
                                               visible=False))
            else:
                self.cursors.append(None)

        # get unique canvasses by iterating over axes
        # this is necessary to avoid multiple connections to the same canvas
        self.canvasses = [self.axes[0].figure.canvas]
        for ax in self.axes[1:]:
            if ax.figure.canvas is not self.canvasses:
                self.canvasses.append(ax.figure.canvas)

        for canvas in self.canvasses:
            # connect the canvas to the draw and motion events
            canvas.mpl_connect('draw_event', self._on_draw)
            canvas.mpl_connect('motion_notify_event', self._on_mouse_move)

    def _on_draw(self, event):
        self.backgrounds.clear()
        for ax in self.axes:
            canvas = ax.figure.canvas
            self.backgrounds.append(canvas.copy_from_bbox(ax.bbox))

    def _on_mouse_move(self, event):
        if event.xdata is None or not self.backgrounds:
            return

        for ax, line, bg in zip(self.axes, self.cursors, self.backgrounds):
            canvas = ax.figure.canvas
            canvas.restore_region(bg)
            if line is not None:
                line.set_xdata([event.xdata])
                line.set_visible(True)
                ax.draw_artist(line)
            canvas.blit(ax.bbox)

class FlightPlotterBase(object):
    def __init__(self, time, name="Flight Plotter"):
        self.t = np.asarray(time, dtype=float)
        # make sure time is a 1D array
        if self.t.ndim != 1:
            raise ValueError("time must be a 1D array")

        self.name = name
        self.all_axes = []

    def define_layout(self, figsize=(12, 8), nrows=3, ncols=3, width_ratios=None, height_ratios=None):
        if width_ratios is None:
            width_ratios = [1] * ncols
        if height_ratios is None:
            height_ratios = [1] * nrows

        self.fig = plt.figure(figsize=figsize)
        self.gs = GridSpec(nrows=nrows, ncols=ncols,
                           width_ratios=width_ratios,
                           height_ratios=height_ratios)

    def plot(self):
        self._populate()
        self._dress()
        # self.curser = BlittedCursor(self.all_axes, self.fig.canvas, sharex=True)
        self.fig.show()

    def _populate(self):
        raise NotImplementedError("Child class should implement this method to populate the plot.")

    def connect_viewport(self, viewport):
        self._add_callback('motion_notify_event', viewport.update)

    def _add_callback(self, event_type, callback):
        # e,g, motion_notify_event
        self.fig.canvas.mpl_connect(event_type, callback)

    def _plot_timeseries(self, ax, light=None, solid=None, dashed=None, series_labels=[], style_labels=[None, None, None], title="", ylabel="", ylimits=(None, None)):
        if solid is None or len(solid) == 0:
            raise ValueError("At least one solid series must be provided.")
        if len(series_labels) != len(solid):
            raise ValueError("series_labels must have the same length as solid series.")
        if len(style_labels) != 3:
            raise ValueError("style_labels must have exactly 3 entries. Set to None if not needed.")
        if light is None or len(light) == 0:
            light = [None] * len(solid)
        if dashed is None or len(dashed) == 0:
            dashed = [None] * len(solid)
        lengths = np.array([len(solid), len(light), len(dashed), len(series_labels)])
        if not (lengths == lengths[0]).all():
            raise ValueError("light, solid, dashed and labels must all have the same length if given.")

        for i, series in enumerate(light):
            if series is not None:
                ax.plot(self.t, series, color=COLORS[i], alpha=0.3, lw=1.0, linestyle='-')

        for i, series in enumerate(solid):
            ax.plot(self.t, series, label=series_labels[i], color=COLORS[i], alpha=0.8, lw=1.0, linestyle='-')

        for i, series in enumerate(dashed):
            if series is not None:
                ax.plot(self.t, series, color=COLORS[i], lw=1.5, linestyle='--')

        self.all_axes.append(ax)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_ylim(ylimits)

        ax.add_artist(ax.legend(loc='upper left'))
        ax.add_artist(self._generate_style_legend(ax, style_labels))

    def _generate_style_legend(self, ax, labels):
        linestyles = []
        if labels[0] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=0.3, lw=1.0, linestyle='-', label=labels[0]))
        if labels[1] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=0.8, lw=1.0, linestyle='-', label=labels[1]))
        if labels[2] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=1.0, lw=1.5, linestyle='--', label=labels[2]))

        leg_styles = ax.legend(handles=linestyles, title='Line Styles', loc='lower left')

        return leg_styles

    def _dress(self):
        for ax in self.all_axes:
            ax.grid(True)
            # only set time label if in lowest row
            if ax.get_subplotspec().is_last_row():
                ax.set_xlabel("Time [s]")

        self.fig.suptitle(self.name)

class IndiflightPlotter(FlightPlotterBase):
    """This wrapper class for FlightPlotterBase that implements the layout and populates the plots for general Indiflight analysis"""
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
    """This wrapper class for FlightPlotterBase that implements the layout and populates the plots for SysId analysis"""
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

class Craft3D(object):
    def __init__(self, body_geometry=[np.array([[0,0,0]])]):

        # check that array is Nx3
        self.Ng = len(body_geometry)
        self.geometry = []
        for element in body_geometry:
            self.geometry.append(np.asarray(element, dtype=float))
            if self.geometry[-1].ndim != 2 or self.geometry[-1].shape[1] != 3:
                raise ValueError("Base geometry must be a list of Nx3 arrays")

        self.rotors = []
        self.surfaces = []

    def generate(self, quat, rotor_controls=None, surface_controls=None):
        body_rotation = R.from_quat(quat)

        xs, ys, zs = np.array([]), np.array([]), np.array([])
        for geo in self.geometry:
            geo_rotated = body_rotation.apply(geo)
            xs = np.concatenate((xs, geo_rotated[:, 0], np.array([np.nan])))
            ys = np.concatenate((ys, geo_rotated[:, 1], np.array([np.nan])))
            zs = np.concatenate((zs, geo_rotated[:, 2], np.array([np.nan])))

        if rotor_controls is None:
            rotor_controls = np.zeros((len(self.rotors), 3))
        elif rotor_controls.shape[0] != len(self.rotors):
            raise ValueError("rotor_controls must have the same length as the number of rotors")

        if surface_controls is None:
            surface_controls = np.zeros((len(self.surfaces),))
        elif surface_controls.shape[0] != len(self.surfaces):
            raise ValueError("surface_controls must have the same length as the number of surfaces")

        arrows = []
        for rotor, controls in zip(self.rotors, rotor_controls):
            # generate circle in the rotor plane by using xyz, axis
            Rr = rotor["R"]
            xyz = rotor["xyz"]
            axis = rotor["axis"]
            tilt_xyz = rotor["tilt_xyz"]

            tilt1_axis = rotor["tilt_axis"]
            tilt1_angle = controls[1]
            tilt1_rotation = R.from_rotvec(tilt1_axis*tilt1_angle)

            tilt2_axis = rotor["tilt2_axis"]
            tilt2_angle = controls[2]
            tilt2_rotation = R.from_rotvec(tilt2_axis*tilt2_angle)
            tilt_rotation = tilt2_rotation * tilt1_rotation

            N = rotor["N"]
            u = np.linspace(0, 2*np.pi, N)

            # generate circle in xy plane
            x = Rr * np.cos(u)
            y = Rr * np.sin(u)
            z = np.zeros_like(u)
            circle = np.vstack((x, y, z)).T

            # rotate circle to align with rotor axis
            circle_tilted = tilt_rotation.apply(circle)

            foot_xyz = xyz - tilt_xyz
            real_xyz = foot_xyz + tilt_rotation.apply(tilt_xyz)

            circle_tilted += real_xyz

            circle_rotated = body_rotation.apply(circle_tilted)


            xs = np.concatenate((xs, circle_rotated[:, 0], np.array([np.nan])))
            ys = np.concatenate((ys, circle_rotated[:, 1], np.array([np.nan])))
            zs = np.concatenate((zs, circle_rotated[:, 2], np.array([np.nan])))

            # arrow for thrust
            thrust = controls[0]
            real_axis = tilt_rotation.apply(axis)

            arrow_start = real_xyz
            arrow_end = real_xyz + real_axis * thrust * 2. * Rr  # scale thrust for visualization
            arrow_max = real_xyz + real_axis * 2. * Rr  # max arrow length for visualization

            arrows.append([arrow_start, arrow_end, arrow_max])

        for surface in self.surfaces:
            geometry = surface["geometry"]
            tilt_xyz = surface["tilt_xyz"]
            tilt_axis = surface["tilt_axis"]
            tilt_angle = surface_controls[0]
            tilt_rotation = R.from_rotvec(tilt_axis*tilt_angle)

            for geo in geometry:
                geo_tilted = tilt_rotation.apply(geo - tilt_xyz) + tilt_xyz
                geo_rotated = body_rotation.apply(geo_tilted)
                xs = np.concatenate((xs, geo_rotated[:, 0], np.array([np.nan])))
                ys = np.concatenate((ys, geo_rotated[:, 1], np.array([np.nan])))
                zs = np.concatenate((zs, geo_rotated[:, 2], np.array([np.nan])))

        return xs, ys, zs, arrows

    def addRotor(self,
                 xyz=[0, 0, 0], axis=[0, 0, 1], 
                 tilt_xyz=[0, 0, 0], tilt_axis=[1, 0, 0],
                 R=0.1, N=20):

        xyz = np.asarray(xyz, dtype=float)
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)

        tilt_xyz = np.asarray(tilt_xyz, dtype=float)
        tilt_axis = np.asarray(tilt_axis, dtype=float)
        tilt_axis /= np.linalg.norm(tilt_axis)

        # compute cross tilt axis, and check that tilt_axis is not parallel to axis
        cross = np.cross(axis, tilt_axis)
        if np.linalg.norm(cross) < 1e-4:
            raise ValueError("tilt_axis cannot be parallel to axis")
        cross /= np.linalg.norm(cross)

        self.rotors.append({
            "xyz": xyz,
            "axis": axis,
            "tilt_xyz": tilt_xyz,
            "tilt_axis": tilt_axis,
            "tilt2_axis": cross,
            "R": R,
            "N": N,
        })

    def addSurface(self,
                   tilt_xyz=[0, 0, 0], tilt_axis=[0, 0, 1],
                   geometry=[np.array([0, 0, 0])]):
        tilt_xyz = np.asarray(tilt_xyz, dtype=float)
        tilt_axis = np.asarray(tilt_axis, dtype=float)
        geometry = [np.asarray(g, dtype=float) for g in geometry]

        self.surfaces.append({
            "tilt_xyz": tilt_xyz,
            "tilt_axis": tilt_axis,
            "geometry": geometry
        })


class Quadrotor(Craft3D):
    def __init__(self, l=0.2, R=0.1):
        body = np.array([
            [    l,   -l/4,     0.],
            [    l,   +l/4,     0.],
            [    l+l/4,   0,     0.],
            [    l,   -l/4,     0.],
        ])
        geometry = [body]
        super().__init__(body_geometry=geometry)

        arms = np.array([
            [ -l, +l, 0.],
            [ +l, +l, 0.],
            [ -l, -l, 0.],
            [ +l, -l, 0.],
        ])

        for arm in arms:
            self.addRotor(xyz=arm, axis=[0, 0, -1], R=R)

class Tailsitter(Craft3D):
    def __init__(self, l=0.3, R=0.1):
        body = np.array([
            [    0,   2*l,     l],
            [    0,   2*l,     0.],
            [    0,    0.,   -1*l],
            [    l,    0.,   -1*l],
            [    0,    0.,   -1*l],
            [    0,  -2*l,   0.],
            [    0,  -2*l,   l],
            [    0,   2*l,   l],
        ])
        geometry = [body]
        super().__init__(body_geometry=geometry)

        self.addRotor(xyz=[0, l, -l] , axis=[0, 0, -1], R=R)
        self.addRotor(xyz=[0, -l, -l], axis=[0, 0, -1], R=R)

        surface = np.array([
            [0, 2.*l, 1.0*l],
            [0, 2.*l, 1.5*l],
            [0, 0.5*l, 1.5*l],
            [0, 0.5*l, 1.0*l],
        ])
        self.addSurface(
            tilt_xyz=[0., 0., 1.*l], tilt_axis=[0., +1., 0.], geometry=[surface]
        )
        self.addSurface(
            tilt_xyz=[0., 0., 1.*l], tilt_axis=[0., -1., 0.], geometry=[surface*np.array([1., -1., 1.])]
        )


class Viewport(object):
    """Generate an updatable 3D viewport for visualizing the state of a craft
    """
    def __init__(self, craft: Craft3D,
                 time,
                 att, attSet=None, attMeas=None,
                 pos=None, posSet=None, posMeas=None,
                 vel=None, velSet=None, velMeas=None,
                 acc=None, accSet=None, accMeas=None,
                 rotorSet=None, surfaceSet=None,
                 follow=False, interpolation="previous", title="Viewport"):
        """Initialize the viewport and open its plot window. Numpy arrays expected.

        Args:
            craft: Craft3D object defining the vehicle geometry
            time: 1D array of time stamps
            att: Nx4 array of attitude quaternions [w, x, y, z]
            attSet: Nx4 array of attitude setpoint quaternions [w, x, y, z]
            attMeas: Nx4 array of attitude measurement quaternions [w, x, y, z]
            pos: Nx3 array of position [x, y, z]
            posSet: Nx3 array of position setpoint [x, y, z]
            posMeas: Nx3 array of position measurement [x, y, z]
            vel: Nx3 array of velocity [vx, vy, vz]
            velSet: Nx3 array of velocity setpoint [vx, vy, vz]
            velMeas: Nx3 array of velocity measurement [vx, vy, vz]
            acc: Nx3 array of acceleration [ax, ay, az]
            accSet: Nx3 array of acceleration setpoint [ax, ay, az]
            accMeas: Nx3 array of acceleration measurement [ax, ay, az]
            rotorSet: NxMx3 array of rotor controls (thrust, tilt1, tilt2) for M rotors
            surfaceSet: NxK array of surface controls for K surfaces
            follow: if True, the camera will follow the vehicle position
            interpolation: interpolation method for data ("previous", "linear", "cubic")
            title: title of the plot

        Returns:
            Viewport object
        """

        self.craft = craft
        self.name = title
        self.t = time
        self.follow = follow
        self.has_pos = pos is not None
        self.interpolation = interpolation

        if self.interpolation not in ["previous", "linear", "cubic"]:
            raise ValueError("interpolation must be one of 'nearest', 'linear', or 'cubic'")

        #%% define data series and their plotting styles
        # attitude
        self.att = {"att": {"raw": att[:, [1,2,3,0]], "style": "solid",  "color": COLORS[0], "marker": None, "width": 2.0, "label": "Attitude Estimate"}}
        if attSet is not None:
            self.att["attSet"] = {"raw": attSet[:, [1,2,3,0]], "style": "dashed", "color": COLORS[1], "marker": None, "width": 0.8,  "label": "Attitude Setpoint"}
        if attMeas is not None:
            self.att["attMeas"] = {"raw": attMeas[:, [1,2,3,0]], "style": "dashed", "color": COLORS[2], "marker": None, "width": 0.8,  "label": "Attitude Measurement"}

        # position
        self.pos = {}
        if pos is not None:
            self.pos["pos"] = {"raw": pos, "style": "solid",  "color": COLORS[0], "marker": ".", "width": 1.5, "label": "Position Estimate"}
        if posSet is not None:
            self.pos["posSet"] = {"raw": posSet, "style": "dashed", "color": COLORS[1], "marker": "o", "width": 1.0,  "label": "Position Setpoint"}
        if posMeas is not None:
            self.pos["posMeas"] = {"raw": posMeas, "style": "solid", "color": COLORS[2], "marker": "o", "width": 3.0, "label": "Position Measurement"}

        # velocity
        self.vel = {}
        if vel is not None:
            self.vel["vel"] = {"raw": vel, "style": "solid",  "color": COLORS[0], "marker": None, "width": 2.0, "label": "Velocity Estimate"}
        if velSet is not None:
            self.vel["velSet"] = {"raw": velSet, "style": "dashed", "color": COLORS[1], "marker": None, "width": 1.0,  "label": "Velocity Setpoint"}
        if velMeas is not None:
            self.vel["velMeas"] = {"raw": velMeas, "style": "dashed", "color": COLORS[2], "marker": None, "width": 1.0,  "label": "Velocity Measurement"}

        # acceleration
        self.acc = {}
        if acc is not None:
            self.acc["acc"] = {"raw": acc, "style": "solid",  "color": COLORS[3], "marker": None, "width": 2.0, "label": "Acceleration Estimate"}
        if accSet is not None:
            self.acc["accSet"] = {"raw": accSet, "style": "dashed", "color": COLORS[4], "marker": None, "width": 1.0,  "label": "Acceleration Setpoint"}
        if accMeas is not None:
            self.acc["accMeas"] = {"raw": accMeas, "style": "dashed", "color": COLORS[5], "marker": None, "width": 1.0,  "label": "Acceleration Measurement"}

        # controls
        self.controls = {}
        if rotorSet is not None:
            self.controls["rotorSet"] = {"raw": rotorSet, "style": "solid",  "color": COLORS[6], "marker": "s", "width": 3.0, "label": "Rotor Controls"}
        if surfaceSet is not None:
            self.controls["surfaceSet"] = {"raw": surfaceSet, "style": "solid",  "color": COLORS[7], "marker": None, "width": 3.0, "label": None}

        # collect all series
        self.series = {**self.att, **self.pos, **self.vel, **self.acc, **self.controls}

        #%% figure setup
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # interpolators and initial plotting lines
        for key, value in self.series.items():
            value['interpolator'] = interp1d(
                self.t,
                np.moveaxis(value['raw'], 0, -1),
                kind=self.interpolation,
                bounds_error=False,
                fill_value=(value["raw"][0], value["raw"][-1]))

            value['line'] = self.ax.plot(
                [np.nan], [np.nan], [np.nan],
                linestyle=value['style'],
                color=value['color'],
                lw=value['width'],
                label=value["label"])[0]

        # set view angle
        self.ax.view_init(elev=-25, azim=150, roll=180)

        # set limits
        if self.has_pos:
            minx, miny, minz = np.min(self.series["pos"]["raw"], axis=0)
            maxx, maxy, maxz = np.max(self.series["pos"]["raw"], axis=0)
        else:
            minx, miny, minz = 0, 0, 0
            maxx, maxy, maxz = 0, 0, 0

        self.ax.set_xlim(minx-0.5, maxx+0.5)
        self.ax.set_ylim(miny-0.5, maxy+0.5)
        self.ax.set_zlim(minz-0.5, maxz+0.5)

        # legend, title and show
        self.ax.legend(loc='upper left')
        self.ax.set_title(self.name)
        self.fig.show()

    def update(self, event):
        """Callback function to update the viewport on mouse hover

        Args:
            event (matplotlib.backend_bases.MouseEvent): mouse event

        Returns:
            None
        """

        # abort if event doesnt contain what we need
        if event.xdata is None:
            return

        # remove old lines
        for _, ser in self.series.items():
            try:
                ser['line'].remove()
            except ValueError:
                pass

        # get interpolates
        interpolates = {}
        for ser in self.series.keys():
            interpolates[ser] = self.series[ser]['interpolator'](event.xdata)

        # plot attitude
        for ser in [x for x in ["att", "attSet", "attMeas"] if x in self.series.keys()]:
            # invoke craft to get the rotated geometry
            rotor_controls = interpolates["rotorSet"] if "rotorSet" in interpolates.keys() else None
            surface_controls = interpolates["surfaceSet"] if "surfaceSet" in interpolates.keys() else None
            xs, ys, zs, q = self.craft.generate(interpolates[ser],
                                                rotor_controls=rotor_controls,
                                                surface_controls=surface_controls,
                                                )

            qs = np.empty((0, 3))
            for arrow in q:
                arrow_start, arrow_end, arrow_max = arrow
                qs = np.concatenate((qs,
                                     arrow_start[np.newaxis],
                                     arrow_end[np.newaxis],
                                     np.array([[np.nan, np.nan, np.nan]]),
                                     arrow_max[np.newaxis],
                                     np.array([[np.nan, np.nan, np.nan]]),
                                     ))

            if "pos" in interpolates.keys():
                qs += interpolates["pos"]

            # translate geometry, if necessary
            if "pos" in interpolates.keys():
                if ser == "attMeas" and "posMeas" in interpolates.keys():
                    # offset by measured position
                    xs += interpolates["posMeas"][0]
                    ys += interpolates["posMeas"][1]
                    zs += interpolates["posMeas"][2]
                else:
                    # offset by estimator position
                    xs += interpolates["pos"][0]
                    ys += interpolates["pos"][1]
                    zs += interpolates["pos"][2]

            # update plot line
            self.series[ser]['line'] = self.ax.plot(xs, ys, zs,
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'])[0]

            # add arrows for rotors
            if ser == "att" and len(q) > 0 and "rotorSet" in self.series.keys():
                self.series["rotorSet"]['line'] = self.ax.plot(qs[:, 0], qs[:, 1], qs[:, 2],
                             linestyle=self.series["rotorSet"]['style'],
                             color=self.series["rotorSet"]['color'],
                             marker=self.series["rotorSet"]['marker'],
                             markersize=1,
                             lw=self.series["rotorSet"]['width'])[0]

        # scatter plot for position
        for ser in [x for x in ["pos", "posSet", "posMeas"] if x in self.series.keys()]:
            self.series[ser]['line'] = self.ax.scatter(
                interpolates[ser][0],
                interpolates[ser][1],
                interpolates[ser][2],
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'],
                marker=self.series[ser]['marker'],
                facecolor='none',
                s=50*self.series[ser]['width'])

        # line to show velocity
        for ser in [x for x in ["vel", "velSet", "velMeas"] if x in self.series.keys()]:
            if ser == "velMeas" and "posMeas" in interpolates.keys():
                offset = interpolates["posMeas"]
            elif "pos" in interpolates.keys():
                offset = interpolates["pos"]
            else:
                offset = np.array([0, 0, 0])

            self.series[ser]['line'] = self.ax.plot(
                [offset[0], 0.2*interpolates[ser][0] + offset[0]],
                [offset[1], 0.2*interpolates[ser][1] + offset[1]],
                [offset[2], 0.2*interpolates[ser][2] + offset[2]],
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'])[0]

        # line to show acceleration
        for ser in [x for x in ["acc", "accSet", "accMeas"] if x in self.series.keys()]:
            if ser == "accMeas" and "posMeas" in interpolates.keys():
                offset = interpolates["posMeas"]
            elif "pos" in interpolates.keys():
                offset = interpolates["pos"]
            else:
                offset = np.array([0, 0, 0])

            self.series[ser]['line'] = self.ax.plot(
                [offset[0], 0.1*interpolates[ser][0] + offset[0]],
                [offset[1], 0.1*interpolates[ser][1] + offset[1]],
                [offset[2], 0.1*interpolates[ser][2] + offset[2]],
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'])[0]

        # make sure all the axes are equal
        if self.follow:
            self.ax.set_aspect('equal', adjustable='datalim')
        else:
            self.ax.set_aspect('equal', adjustable='box')

        # finally, update the canvas
        self.fig.canvas.draw()

class IndiflightViewport(Viewport):
    def __init__(self, craft: Craft3D, data, follow=False, interpolation="previous", title="Viewport"):
        # thin wrapper: extract series from log and intialize base class

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


if __name__ == "__main__":
    import numpy as np
    ts = Tailsitter()
    vp = Viewport(ts,
                  time=np.array([0., 1., 2.]),
                  att=np.array([[1., 0., 0., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]]),
                  attSet=np.array([[1., 0., 0., 0.], [1., 0., 0., 0.1], [0., 0., 0., 1.]]),
                  attMeas=np.array([[1., 0., 0., 0.], [1., 0., 0., -0.1], [0., 0., 0., 1.]]),
                  pos=np.array([[0., 0., 0.], [0.2, 0.2, 0.2], [0.4, 0.4, 0.4]]),
                  posSet=np.array([[0.4, 0.4, 0.4], [0.4, 0.4, 0.4], [0.4, 0.4, 0.4]]),
                  posMeas=np.array([[0., 0., 0.], [0.1, 0.1, 0.1], [0.4, 0.4, 0.4]]),
                  vel=3*np.array([[0., 0., 0.], [0.2, 0.2, 0.2], [0.4, 0.4, 0.4]]),
                  velSet=3*np.array([[0.4, 0.4, 0.4], [0.4, 0.4, 0.4], [0.4, 0.4, 0.4]]),
                  velMeas=3*np.array([[0., 0., 0.], [0.1, 0.1, 0.1], [0.4, 0.4, 0.4]]),
                  acc=5*np.array([[0., 0., 0.], [0.2, -0.2, -0.2], [0.4, -0.4, -0.4]]),
                  accSet=5*np.array([[0.4, 0.4, 0.4], [0.4, -0.4, -0.4], [0.4, -0.4, -0.4]]),
                  accMeas=5*np.array([[0., 0., 0.], [0.1, -0.1, -0.1], [0.4, -0.4, -0.4]]),
                  rotorSet=np.array([
                      [[1., 0., 0.], [1., 0., 0.]],
                      [[0.5, 0.5, 0.], [0.5, 0., 0.5]],
                      [[1., 0.2, 0.2], [1., -0.2, -0.2]]
                  ]),
                  surfaceSet=np.array([[0., 0.], [0.5, -0.5], [1.0, -0.5]]),
                  )

    # put slider onto the plot so we can test the mouse events 
    from matplotlib.widgets import Slider
    from matplotlib.backend_bases import MouseEvent
    axcolor = 'lightgoldenrodyellow'
    axfreq = plt.axes([0.25, 0.02, 0.50, 0.03], facecolor=axcolor)
    sfreq = Slider(axfreq, 'Time', 0.0, 2.0, valinit=0.0)
    def update(val):
        me = MouseEvent('motion_notify_event', vp.fig.canvas, 1.0, 1.0)
        me.xdata = val
        vp.update(me)
    sfreq.on_changed(update)

    update(0)

