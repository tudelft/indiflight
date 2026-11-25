# Command line tools bfl2csv and bfl2rerun (installed as console scripts)
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from os import path

from .indiflight_log_importer import IndiflightLog


def bfl2csv(argv=None):
    """Convert .BFL binary logs to .csv"""
    parser = ArgumentParser(description=bfl2csv.__doc__, formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", metavar="x.BFL", help="BFL log to be converted")
    parser.add_argument("-i", "--id", required=False, type=int, default=1, help="If multiple logs combined in one file, select this one.")
    parser.add_argument("-o", "--output-dir", required=False, type=str, default=None, help="Output Directory. If not passed, same directory as logfile.")
    parser.add_argument("-u", "--units", required=False, type=str, default="raw", help="raw or si")
    args = parser.parse_args(argv)

    outdir = args.output_dir if args.output_dir else path.dirname(path.abspath(args.logfile))

    log = IndiflightLog(args.logfile, logId=args.id, timeRange=None, resetTime=False)
    log.outputCsv(units=args.units, path=outdir)


def bfl2rerun(argv=None):
    """Convert .BFL binary logs and send them to rerun"""
    parser = ArgumentParser(description=bfl2rerun.__doc__, formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", metavar="x.BFL", help="BFL log to be added")
    parser.add_argument("-i", "--id", required=False, type=int, default=1, help="If multiple logs combined in one file, select this one.")
    parser.add_argument("-n", "--name", required=False, type=str, default="fc", help="Entity name for rerun.")
    parser.add_argument("-c", "--clock-offset", required=False, type=float, default=0., help="Clock offset in seconds to add to the log time.")
    parser.add_argument("-s", "--spawn", required=False, action='store_true', default=False, help="Spawn new recording and, if necessary, a new rerun window")
    parser.add_argument("-r", "--recording-id", required=False, type=str, default=None, help="Write to this recording_id")
    args = parser.parse_args(argv)

    try:
        import rerun as rr
    except ImportError:
        parser.exit(1, "bfl2rerun needs rerun: pip install 'indiflight_log_tools[rerun]'\n")

    rr.init("droneracing", recording_id=args.recording_id, spawn=args.spawn)
    rr.connect()

    log = IndiflightLog(args.logfile, logId=args.id, timeRange=None, resetTime=False)
    log.addToRerun(args.name, clockOffsetSeconds=args.clock_offset)
