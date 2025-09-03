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
    "legend.loc": 'best',
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
            if not lines:
                continue
            min_x = min(min(line.get_xdata()) for line in lines)
            self.cursors.append(ax.axvline(x=min_x,
                                           color='black',
                                           linestyle='--',
                                           lw=0.8,
                                           visible=False))
            
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
            line.set_xdata([event.xdata])
            line.set_visible(True)
            ax.draw_artist(line)
            canvas.blit(ax.bbox)

class FlightPlotterBase(object):
    def __init__(self, data, name="Flight Plotter"):
        self.data = data
        self.name = name
        self.all_axes = []

        # preprocess data
        self.t = self.data['timeS'].to_numpy()

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
        raise NotImplementedError("Subclasses should implement this method to populate the plot.")

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

class FlightPlotter(FlightPlotterBase):
    def __init__(self, data, name="Flight Plotter"):
        super().__init__(data, name)

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

        if 'servo_feedback' in self.data.columns:
            self._plot_timeseries(self.fig.add_subplot(self.gs[2, 2]),
                         light=None,
                         solid=[self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(2)],
                         series_labels=[f"Servo {i}" for i in [1,2]],
                         style_labels=[None, "Est. state", "Command"],
                         title="Servo State",
                         ylabel="Servo State [rad]",
            )

class SysIdPlotter(FlightPlotterBase):
    def __init__(self, data, name="System Identification Plotter"):
        super().__init__(data, name)

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
            self._plot_timeseries(self.fig.add_subplot(self.gs[i, 1]),
                                light=None,
                                solid=x if axis == 'x' else y if axis == 'y' else z,
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(N)],
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
                                solid=p[:N] if axis == 'p' else q[:N] if axis == 'q' else r[:N],
                                dashed=None,
                                series_labels=[f"Motor {j}" for j in range(N)],
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

class Viewport(object):
    def __init__(self, data, follow=False, craft="quad", name="Viewport"):
        self.data = data
        self.name = name
        self.follow = follow
        self.has_pos = "pos[0]" in self.data.columns
        self.fig = plt.figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Quadrotor geometry (in local frame)
        l = 0.2
        if craft == "quad":
            self.arms = np.array([
                [ -l, +l, 0.],
                [ +l, +l, 0.],
                [ -l, -l, 0.],
                [ +l, -l, 0.],
            ])
            self.front = np.array([
                [    l,   -l/4,     0.],
                [    l,   +l/4,     0.],
                [    l+l/4,   0,     0.],
                [    l,   -l/4,     0.],
            ])
        elif craft == "tailsitter":
            self.arms = np.array([
                [ 0., +l, -2*l],
                [ 0., -l, -2*l],
            ])
            self.front = np.array([
                [    0,   2*l,     0.],
                [    0,   2*l,   -  l],
                [    0,    0.,   -2*l],
                [    0,  -2*l,   -  l],
                [    0,  -2*l,   0.],
                [    0,   2*l,   0.],
            ])
        else:
            raise ValueError(f"Unknown craft type: {craft}")

        # preprocess data
        self.t = self.data['timeS'].to_numpy()

        # interpolators
        self.series = {
            "quat": {"raw": self.data[[f"quat[{i}]" for i in [1,2,3,0]]].to_numpy()    , "style": "solid",  "color": COLORS[0], "marker": None, "width": 1.5, "label": "Estimate"},
            "quatSp": {"raw": self.data[[f"quatSp[{i}]" for i in [1,2,3,0]]].to_numpy(), "style": "dashed", "color": COLORS[0], "marker": None, "width": 1.0,  "label": "Setpoint"},
        }

        if self.has_pos:
            self.series.update({
                "pos": {"raw": self.data[[f"pos[{i}]" for i in range(3)]].to_numpy()       , "style": "solid",  "color": COLORS[1], "marker": ".", "width": 1.5, "label": "Estimate"},
                "posSp": {"raw": self.data[[f"posSp[{i}]" for i in range(3)]].to_numpy()   , "style": "dashed", "color": COLORS[1], "marker": "o", "width": 1.0,  "label": "Setpoint"},
                "vel": {"raw": self.data[[f"vel[{i}]" for i in range(3)]].to_numpy()       , "style": "solid",  "color": COLORS[2], "marker": None, "width": 1.5, "label": "Estimate"},
                "velSp": {"raw": self.data[[f"velSp[{i}]" for i in range(3)]].to_numpy()   , "style": "dashed", "color": COLORS[2], "marker": None, "width": 1.0,  "label": "Setpoint"},
                "accSp": {"raw": self.data[[f"accSp[{i}]" for i in range(3)]].to_numpy()   , "style": "dashed", "color": COLORS[3], "marker": None, "width": 1.0,  "label": "Setpoint"},
            })

        for key, value in self.series.items():
            self.series[key]['interpolator'] = interp1d(
                self.t,
                value["raw"].T,
                kind="nearest",
                bounds_error=False,
                fill_value=(value["raw"][0], value["raw"][-1]))

            self.series[key]['line'] = self.ax.plot(
                [np.nan], [np.nan], [np.nan],
                linestyle=value['style'],
                color=value['color'],
                lw=value['width'],
                label=value["label"])[0]

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

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

        self.ax.legend(loc='best')

        self.ax.set_title(self.name)

        self.fig.show()

    def update(self, event):
        if event.xdata is None:
            return

        for ser in self.series.keys():
            try:
                self.series[ser]['line'].remove()
            except ValueError:
                pass

        # get interpolates
        interpolates = {}
        for ser in self.series.keys():
            interpolates[ser] = self.series[ser]['interpolator'](event.xdata)

        for ser in ["quat", "quatSp"]:
            rotation = R.from_quat(interpolates[ser])
            rotated_front = rotation.apply(self.front)

            # Plot circles (representing rotors)
            xs = np.array([])
            ys = np.array([])
            zs = np.array([])
            for arm in self.arms:
                u = np.linspace(0, 2*np.pi, 20)
                x = arm[0] + 0.1 * np.cos(u)
                y = arm[1] + 0.1 * np.sin(u)
                z = np.ones_like(x) * arm[2]
                x, y, z = rotation.apply(np.array([x,y,z]).T).T
                xs = np.concatenate((xs, x, np.array([np.nan])))
                ys = np.concatenate((ys, y, np.array([np.nan])))
                zs = np.concatenate((zs, z, np.array([np.nan])))

            # plot front triangle
            xs = np.concatenate((xs, rotated_front[:, 0]))
            ys = np.concatenate((ys, rotated_front[:, 1]))
            zs = np.concatenate((zs, rotated_front[:, 2]))

            if self.has_pos:
                xs += interpolates["pos"][0]
                ys += interpolates["pos"][1]
                zs += interpolates["pos"][2]

            self.series[ser]['line'] = self.ax.plot(xs, ys, zs,
                linestyle=self.series[ser]['style'],
                color=self.series[ser]['color'],
                lw=self.series[ser]['width'])[0]

        if self.has_pos:
            for ser in ["pos", "posSp"]:
                self.series[ser]['line'] = self.ax.scatter(
                    interpolates[ser][0],
                    interpolates[ser][1],
                    interpolates[ser][2],
                    linestyle=self.series[ser]['style'],
                    color=self.series[ser]['color'],
                    lw=self.series[ser]['width'],
                    marker=self.series[ser]['marker'],
                    facecolor='none',
                    s=50)

            for ser in ["vel", "velSp"]:
                self.series[ser]['line'] = self.ax.plot(
                    [interpolates['pos'][0], 0.2*interpolates[ser][0] + interpolates['pos'][0]],
                    [interpolates['pos'][1], 0.2*interpolates[ser][1] + interpolates['pos'][1]],
                    [interpolates['pos'][2], 0.2*interpolates[ser][2] + interpolates['pos'][2]],
                    linestyle=self.series[ser]['style'],
                    color=self.series[ser]['color'],
                    lw=self.series[ser]['width'])[0]

            self.series["accSp"]['line'] = self.ax.plot(
                [interpolates['pos'][0], 0.1*interpolates['accSp'][0] + interpolates['pos'][0]],
                [interpolates['pos'][1], 0.1*interpolates['accSp'][1] + interpolates['pos'][1]],
                [interpolates['pos'][2], 0.1*interpolates['accSp'][2] + interpolates['pos'][2]],
                linestyle=self.series["accSp"]['style'],
                color=self.series["accSp"]['color'],
                lw=self.series["accSp"]['width'])[0]

        # make sure all the axes are equal
        if self.follow:
            self.ax.set_aspect('equal', adjustable='datalim')
        else:
            self.ax.set_aspect('equal', adjustable='box')

        self.fig.canvas.draw()
