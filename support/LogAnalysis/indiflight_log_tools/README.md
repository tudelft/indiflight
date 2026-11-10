# indiflight_log_tools

Decode [Indiflight](https://github.com/tudelft/indiflight) blackbox logs
(`.BFL`) quickly into pandas dataframes, in raw or SI units. Includes the
`bfl2csv` and `bfl2rerun` command line tools.

A C decoder (based on Betaflight's `blackbox_decode`) does the parsing, and
Python then applies the unit conversions for Indiflight.

## Installation

Supported: **Linux** (x86_64, aarch64) and **macOS** (Intel, Apple Silicon)
with **Python 3.10 or newer**. PyPI has prebuilt wheels for these, so you
don't need a compiler.

**Windows: use WSL.** The decoder doesn't build natively on Windows yet.
Install WSL with Ubuntu (`wsl --install` in PowerShell, see
[the Microsoft guide](https://learn.microsoft.com/en-us/windows/wsl/install)),
then follow the Linux instructions inside the Ubuntu terminal. Plot windows
open on the Windows desktop (WSLg, Windows 10 21H2+ or Windows 11).

A virtual environment is recommended, so the package doesn't conflict with
your system Python:
```sh
python3 -m venv ~/indiflight-venv
source ~/indiflight-venv/bin/activate     # run this again in every new terminal
```
On Ubuntu/Debian/WSL you may first need `sudo apt install python3-venv python3-tk`
(`python3-tk` provides the matplotlib plot window backend).

From PyPI:
```sh
pip install indiflight_log_tools
pip install "indiflight_log_tools[rerun]"   # optional, needed for bfl2rerun / addToRerun()
```

From the git repo (compiles the C decoder, so needs `gcc` or `clang`):
```sh
pip install "git+https://github.com/tudelft/indiflight#subdirectory=support/LogAnalysis/indiflight_log_tools"
```

From a local checkout of the indiflight repo (`-e` for development):
```sh
pip install -e support/LogAnalysis/indiflight_log_tools
```

Upgrading from a version installed before 0.3.0? Run
`pip uninstall indiflight_log_tools` first, so that the old `bfl2csv` and
`bfl2rerun` scripts are removed.

Check the installation:
```sh
cd ~/    # or anywhere else, but not the source folder
python -c "from indiflight_log_tools import IndiflightLog; print(IndiflightLog.LIBRARY_SO)"
```

## Usage

```python
from indiflight_log_tools import IndiflightLog

log = IndiflightLog("/path/to/log.bfl", logId=3)                                  # if multiple logs in the same logfile, use this to select the one you want. default 1
log = IndiflightLog("/path/to/log.bfl", resetTime=False)                          # resetTime maintains the flight controller time as the time basis in the dataframe (default True)
log = IndiflightLog("/path/to/log.bfl", resetTime=False, timeRange=(1000, 3000))  # crop the data to timeRange (given in ms)

log.raw                                                                           # pandas dataframe in raw units
log.data                                                                          # pandas dataframe in SI units
log.parameters                                                                    # dict of the settings in the log header
log.events                                                                        # list of logged events
log.modeToText(log.data["flightModeFlags"])                                       # convert flight mode flag(s) into human-readable format
log.flags                                                                         # flight mode changes with timestamp
[(t, log.modeToText(x), log.modeToText(y)) for t, x, y in zip(log.flags["timeUs"], log.flags["enable"], log.flags["disable"])] # flight mode changes in human readable format

log.outputCsv("raw")                                                              # output raw data as csv, comparable to outputting from BB log explorer
log.outputCsv("si")                                                               # output SI data as csv
log.outputCsv("si", path="my/path/for/logOutput/")                                # output SI data into a specific path

log.addToRerun(name="BFL", clockOffsetSeconds=0.)                                 # send data to Rerun (does not do init, connect, etc. just adding data)
```

Installing the package also installs two command line tools:
```sh
bfl2csv /path/to/LOG00001.BFL              # writes CSVs next to the log (shorthand for log.outputCsv())
bfl2csv /path/to/LOG00001.BFL -u si        # SI units instead of raw
bfl2csv --help
bfl2rerun --help                           # shorthand for log.addToRerun()
```

### Errors and temporary files

- If a log is corrupt or `logId` doesn't exist, `IndiflightLog(...)` raises
  `RuntimeError`, and the decoder's message is printed to stderr. Your
  Python session keeps running, so you can catch the error and continue,
  e.g. when you loop over a folder of logs.
- Each `IndiflightLog` instance decodes into its own folder in the user cache
  directory (`~/.cache/indiflight_logs/` on Linux,
  `~/Library/Caches/indiflight_logs/` on macOS), so several processes can
  parse logs at the same time. The folder is deleted when the object is
  garbage collected. Folders left behind by killed processes are deleted
  after 7 days.
- Threads can create `IndiflightLog` objects too, but the C decoder handles
  one log at a time. Use processes if you want to decode in parallel.

## Releasing

Bump `version` in `pyproject.toml`, then push a tag `log-tools-v<version>`.
The GitHub workflow `.github/workflows/log-tools-wheels.yml` in the indiflight
repo builds the wheels and uploads them to PyPI.

## License

GPL-3.0-or-later (see the headers of the source files).
