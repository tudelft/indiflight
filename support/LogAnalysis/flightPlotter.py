
from pyFlightPlotter import BlittedCursor, local_rc
from pyFlightPlotter import Quadrotor, Tailsitter
from indiflightPlotter import IndiflightPlotter, IndiflightViewport
from indiflight_log_tools import IndiflightLog

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

import matplotlib.pyplot as plt
plt.close('all')
plt.rcParams.update(local_rc)

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

args = parser.parse_args()

if args.name is None:
    args.name = args.logfile.split("/")[-1].split(".")[0]

log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
if args.crop:
    log.data, _ = log.crop(args.crop[0], args.crop[1])

fplt = IndiflightPlotter(log.data, name=f"{args.name} -- Flight Data", Nr=2, Ns=2)
cursor = BlittedCursor(fplt.all_axes, sharex=True)

craft = Tailsitter() if args.type == "tailsitter" else Quadrotor()
pplt = IndiflightViewport(craft,
                          log.data,
                          follow=False,
                          Nr=2,
                          Ns=2,
                          interpolation="previous",
                          title=f"{args.name} -- Onboard ID Analysis")
fplt.connect_viewport(pplt)

plt.show()
