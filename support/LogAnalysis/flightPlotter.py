
from pyFlightPlotter import BlittedCursor, local_rc
from pyFlightPlotter import Quadrotor, Tailsitter, Craft3D
from indiflightPlotter import IndiflightPlotter, IndiflightViewport
from indiflight_log_tools import IndiflightLog
from logbook_source import add_log_source_args, add_log_options, resolve_log_source

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import numpy as np

import matplotlib.pyplot as plt
plt.close('all')
plt.rcParams.update(local_rc)

parser = ArgumentParser(description="Analyse onboard ID data from a MAVLab logbook flight or a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
add_log_source_args(parser)
add_log_options(parser)
parser.add_argument("--type", type=str, default="multirotor", choices=["tailsitter", "multirotor", "hexarotor"],
                    help="Type of craft for visualization.")

args = parser.parse_args()
(args.logfile,) = resolve_log_source(parser, args, single=True)

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.logId, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])


class Hexarotor(Craft3D):
    def __init__(self, l=0.4, R=0.1):
        body = np.array([
            [    l,   -l/4,     0.],
            [    l,   +l/4,     0.],
            [    l+l/4,   0,     0.],
            [    l,   -l/4,     0.],
        ])
        super().__init__(body_geometry=[body])

        rotor_radius = l/2
        for a in np.deg2rad([0., 60., 120., 180., 240., 300.]):
            self.addRotor(xyz=[rotor_radius*np.cos(a), rotor_radius*np.sin(a), 0.0], axis=[0, 0, -1], R=R)

if args.type == "tailsitter":
    craft = Tailsitter()
    fplt = IndiflightPlotter(log.data, name=f"{args.name} -- Flight Data", Nr=2, Ns=2)
    pplt = IndiflightViewport(craft,
                              log.data,
                              follow=True,
                              Nr=2,
                              Ns=2,
                              interpolation="previous",
                              title=f"{args.name} -- Onboard ID Analysis")
    pplt2 = IndiflightViewport(craft,
                              log.data,
                              follow=False,
                              Nr=2,
                              Ns=2,
                              interpolation="previous",
                              title=f"{args.name} -- Onboard ID Analysis")
else:
    craft, Nr = (Hexarotor(), 6) if args.type == "hexarotor" else (Quadrotor(), 4)
    fplt = IndiflightPlotter(log.data, name=f"{args.name} -- Flight Data", Nr=Nr, Ns=0)
    pplt = IndiflightViewport(craft,
                              log.data,
                              follow=True,
                              Nr=Nr,
                              Ns=0,
                              interpolation="previous",
                              title=f"{args.name} -- Onboard ID Analysis")
    pplt2 = IndiflightViewport(craft,
                              log.data,
                              follow=False,
                              Nr=Nr,
                              Ns=0,
                              interpolation="previous",
                              title=f"{args.name} -- Onboard ID Analysis")

cursor = BlittedCursor(fplt.all_axes, sharex=True)
fplt.connect_viewport(pplt)
fplt.connect_viewport(pplt2)

plt.show()
