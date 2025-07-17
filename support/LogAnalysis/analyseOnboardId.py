
from plotting import FlightPlotter, SysIdPlotter, Viewport, BlittedCursor
from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt

plt.close('all')

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter


parser = ArgumentParser(description="Analyse onboard ID data from a log file.",
                        formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("logfile", type=str, help="Path to the log file.")
parser.add_argument("--logId", type=int, default=1, help="Log ID to use.")
parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
args = parser.parse_args()

log = IndiflightLog(args.logfile, logId=args.logId, resetTime=args.resetTime)

fplt = FlightPlotter(log.data, name="Flight Data")
splt = SysIdPlotter(log.data, name="Onboard Sys ID Analysis")

pplt = Viewport(log.data, follow=False, name="Onboard ID Analysis")
fplt.connect_viewport(pplt)
splt.connect_viewport(pplt)

cursor = BlittedCursor(fplt.all_axes + splt.all_axes, sharex=True)
