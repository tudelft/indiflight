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
    'figure.subplot.wspace': 0.2,
    'figure.subplot.hspace': 0.3,
})

plt.rcParams.update(local_rc)

class BlittedCursor(object):
    def __init__(self, axes, canvas, sharex=True):
        self.axes = axes
        self.canvas = canvas
        self.backgrounds = []
        self.cursors = []

        ax0 = self.axes[0]

        for i, ax in enumerate(self.axes):
            if i > 0 and sharex:
                ax.sharex(ax0)

            lines = ax.get_lines()
            min_x = min(min(line.get_xdata()) for line in lines)
            self.cursors.append(ax.axvline(x=min_x,
                                           color='black',
                                           linestyle='--',
                                           lw=0.8,
                                           visible=False))

        self.canvas.mpl_connect('draw_event', self._on_draw)
        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)

    def _on_draw(self, event):
        self.backgrounds = [self.canvas.copy_from_bbox(ax.bbox) for ax in self.axes]

    def _on_mouse_move(self, event):
        if event.xdata is None or not self.backgrounds:
            return

        for ax, line, bg in zip(self.axes, self.cursors, self.backgrounds):
            self.canvas.restore_region(bg)
            line.set_xdata([event.xdata])
            line.set_visible(True)
            ax.draw_artist(line)
            self.canvas.blit(ax.bbox)

class FlightPlotter(object):
    def __init__(self, data, name="Flight Plotter"):
        self.data = data
        self.name = name
        self.all_axes = []

        self.fig = plt.figure(figsize=(12, 8))
        self.gs = GridSpec(nrows=3, ncols=3,
                      width_ratios=[2, 2, 2],
                      height_ratios=[1, 1, 1])

        # preprocess data
        self.t = self.data['timeS'].to_numpy()

        # go
        self._populate()
        self._dress()

        self.curser = BlittedCursor(self.all_axes, self.fig.canvas, sharex=True)

        self.fig.show()

    def connect_viewport(self, viewport):
        """
        Connect a viewport to the plotter, allowing it to update on mouse movement.
        """
        self._add_callback('motion_notify_event', viewport.update)

    def _add_callback(self, event_type, callback):
        # e,g, motion_notify_event
        self.fig.canvas.mpl_connect(event_type, callback)

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

        N = 2
        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 1]),
                         light=[self.data[f'motor[{i}]'].to_numpy() for i in range(N)],
                         solid=[self.data[f'u_state[{i}]'].to_numpy() for i in range(N)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(N)],
                         series_labels=[f"Motor {i}" for i in range(N)],
                         style_labels=["Final command", "Est. state", "Command"],
                         title="Motor Speeds",
                         ylabel="Motor Speed [rad/s]",
                         ylimits=(-0.05, 1.05))

        self._plot_timeseries(self.fig.add_subplot(self.gs[2, 2]),
                         light=None,
                         solid=[self.data[f'servo_feedback[{i}]'].to_numpy() for i in range(2)],
                         dashed=[self.data[f'u[{i}]'].to_numpy() for i in range(2)],
                         series_labels=[f"Servo {i}" for i in [1,2]],
                         style_labels=[None, "Est. state", "Command"],
                         title="Servo State",
                         ylabel="Servo State [rad]",
        )

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

        ax.add_artist(ax.legend(loc='upper right'))
        ax.add_artist(self._generate_style_legend(ax, style_labels))

    def _generate_style_legend(self, ax, labels):
        linestyles = []
        if labels[0] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=0.3, lw=1.0, linestyle='-', label=labels[0]))
        if labels[1] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=0.8, lw=1.0, linestyle='-', label=labels[1]))
        if labels[2] is not None:
            linestyles.append(Line2D([0], [0], color='gray', alpha=1.0, lw=1.5, linestyle='--', label=labels[2]))

        leg_styles = ax.legend(handles=linestyles, title='Line Styles', loc='lower right')

        return leg_styles

    def _dress(self):
        for ax in self.all_axes:
            ax.grid(True)
            ax.set_xlabel("Time [s]")

        self.fig.suptitle(self.name)

class Viewport(object):
    def __init__(self, data, follow=False, name="Viewport"):
        self.data = data
        self.name = name
        self.follow = follow
        self.has_pos = "pos[0]" in self.data.columns
        self.fig = plt.figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Quadrotor geometry (in local frame)
        l = 0.2
        # self.arms = np.array([
        #     [ -l, +l, 0.],
        #     [ +l, +l, 0.],
        #     [ -l, -l, 0.],
        #     [ +l, -l, 0.],
        # ])
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
