"""Shared command-line options and log loading for the LogAnalysis scripts.

A log is a MAVLab logbook flight number (downloaded and cached, several at once are fine) or a
local file given with --file. Scripts that analyse one log take the target as `flight`/--file;
scripts that take a second log (a source of an actuation model) take `source_flight`/--source-file.

Single target log:

    add_log_source_args(parser)
    add_log_options(parser)                     # --logId, --resetTime (required), --crop, --name
    args = parser.parse_args()
    (args.logfile,) = resolve_log_source(parser, args, single=True)
    log = open_log(args.logfile, args)

Collection of logs (e.g. a flight numbers list or a folder of files):

    add_log_source_args(parser, multi=True)
    args = parser.parse_args()
    logfiles = resolve_log_source(parser, args, multi=True)

Optional second log, with `--source-logId`, `--source-resetTime`, `--source-crop`:

    add_log_source_args(parser, prefix="source", positional=True)
    add_log_options(parser, prefix="source")
    src = resolve_log_source(parser, args, prefix="source", required=False, single=True)

Scripts with hardcoded paths can use `fetch_logs([FLIGHT])[0]` in their commented-out code.
"""

import argparse
import os


class LogbookError(RuntimeError):
    pass


def _dest(prefix, name):
    # argparse dest for option `name` under `prefix`, e.g. ("source", "id") -> "source_id"
    return f"{prefix}_{name}" if prefix else name


def _opt(prefix, name):
    # command-line option for `name` under `prefix`, e.g. ("source", "file") -> "--source-file"
    return f"--{prefix}-{name}" if prefix else f"--{name}"


def add_log_source_args(parser, prefix="", multi=False, positional=True):
    """Positional flight number(s), --file (repeatable) and --server.

    multi=True: the positional takes any number of flight numbers.
    positional=False: no positional flight argument, only --file/--server.
    """
    label = "log" if not prefix else f"{prefix} log"
    if positional:
        parser.add_argument(_dest(prefix, "flights" if multi else "flight"),
                            type=int, nargs="*" if multi else "?",
                            metavar="FLIGHT" if not prefix else "SOURCE_FLIGHT",
                            help="Flight number(s) in the MAVLab logbook. Their raw logs are downloaded and cached.")
    parser.add_argument(_opt(prefix, "file"), dest=_dest(prefix, "file"), action="append", default=None,
                        help=f"Read this local {label} file instead of downloading a flight.")
    parser.add_argument(_opt(prefix, "server"), dest=_dest(prefix, "server"), type=str, default=None,
                        help="MAVLab logbook server (default: $LOGDB_SERVER).")


def add_log_options(parser, prefix="", logId=True, resetTime=True, crop=True, name=True, crop_group=None):
    """The options every log loader shares: --logId and --resetTime (both required, no defaults),
    --crop and --name. With `prefix` (e.g. "source") the first two are optional and checked by
    resolve_log_source only when that log is given."""
    required = not prefix
    if logId:
        parser.add_argument(_opt(prefix, "logId"), dest=_dest(prefix, "logId"), type=int, default=None,
                            required=required, metavar="ID", help="Log ID in the log file.")
    if resetTime:
        parser.add_argument(_opt(prefix, "resetTime"), dest=_dest(prefix, "resetTime"),
                            action=argparse.BooleanOptionalAction, default=None, required=required,
                            help="Reset the time to start of the log (--no-resetTime to keep it).")
    if crop:
        group = crop_group if crop_group is not None else parser
        group.add_argument(_opt(prefix, "crop"), dest=_dest(prefix, "crop"), required=False, nargs=2,
                           metavar=("START", "END"), type=float,
                           help="Crop the log to the given time range (in seconds).")
    if name and not prefix:
        parser.add_argument("--name", required=False, help="Name for the analysis, used in plots.")


def fetch_logs(flights, server=None):
    """Download (or take from the cache) the raw logs of flight numbers, in the order given."""
    import mavlab_logbook_client as mavlab

    server = server or os.environ.get("LOGDB_SERVER")
    if not server:
        raise LogbookError("no logbook server: pass --server or set LOGDB_SERVER")
    try:
        mavlab.connect(server)
        return [str(p) for p in mavlab.fetch_raw(list(flights))]
    except mavlab.Error as exc:
        raise LogbookError(f"mavlab-logbook-client: {exc}") from exc


def resolve_log_source(parser, args, prefix="", multi=False, required=True, single=False):
    """Return the list of local log paths from the flight numbers and --file options.

    single=True: exactly one log is required (or zero if required=False).
    """
    flights = getattr(args, _dest(prefix, "flights" if multi else "flight"), None)
    if flights is None:
        flights = []
    elif isinstance(flights, int):
        flights = [flights]
    files = getattr(args, _dest(prefix, "file"), None) or []
    server = getattr(args, _dest(prefix, "server"), None)

    paths = list(files)
    if flights:
        try:
            paths += fetch_logs(flights, server)
        except LogbookError as exc:
            parser.exit(1, f"{exc}\n")
    paths = list(dict.fromkeys(paths))

    label = "log" if not prefix else f"{prefix} log"
    if paths and prefix:
        for name in ("logId", "resetTime"):
            if getattr(args, _dest(prefix, name)) is None:
                parser.error(f"the {label} needs --{prefix}-{name}")
    if required and not paths:
        parser.error(f"give a flight number or --file for the {label}")
    if single and len(paths) > 1:
        parser.error(f"give exactly one {label} (flight number or --file), got {len(paths)}")
    return paths


def open_log(path, args, prefix=""):
    """IndiflightLog of a path with --logId, --resetTime and --crop of `prefix` applied."""
    from indiflight_log_tools import IndiflightLog

    log = IndiflightLog(path, logId=getattr(args, _dest(prefix, "logId")),
                        resetTime=getattr(args, _dest(prefix, "resetTime")))
    crop = getattr(args, _dest(prefix, "crop"), None)
    if crop:
        log.data, _ = log.crop(crop[0], crop[1])
    return log


def default_name(args, path):
    """args.name, or the file name of the log without its extension when --name is not given."""
    if getattr(args, "name", None) is None:
        args.name = path.split("/")[-1].split(".")[0]
    return args.name
