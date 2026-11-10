

## Installation

From PyPI (prebuilt wheels for Linux x86_64/aarch64 and macOS, any
Python >= 3.10, no compiler needed). On Windows, use WSL; see
[`../README.md`](../README.md).
```
pip install indiflight_log_tools            # add [rerun] for bfl2rerun
```

From the git repo (compiles the C decoder, so needs `gcc`/`clang`):
```
pip install "git+https://github.com/tudelft/indiflight#subdirectory=support/LogAnalysis/indiflight_log_tools"
```

From a local checkout (`-e` for development):
```
pip install -e support/LogAnalysis/indiflight_log_tools
```

Test it
```
cd ~/    # or anywhere else, but not the source folder
python -c "from indiflight_log_tools import IndiflightLog; print(IndiflightLog.LIBRARY_SO)"
```

### Releasing

Bump `version` in `pyproject.toml`, then push a tag `log-tools-v<version>`.
`.github/workflows/log-tools-wheels.yml` builds the wheels and uploads them to PyPI.

## Usage

```
from indiflight_log_tools import IndiflightLog

log = IndiflightLog("/path/to/log.bfl", logId=3)                                  # if multiple logs in the same logfile, use this to select the one you want. default 1
log = IndiflightLog("/path/to/log.bfl", resetTime=False)                          # resetTime maintains the flight controller time as the time basis in the dataframe (default True)
log = IndiflightLog("/path/to/log.bfl", resetTime=False, timeRange=(1000, 3000))  # crop the data to timeRange (given in ms)

log.raw                                                                           # pandas dataframe in raw units
log.data                                                                          # pandas dataframe in SI units
log.modeToText(log.data["flightModeFlags"])                                       # convert flight mode flag(s) into human-readable format
log.flags                                                                         # flight mode changes with timestamp
[(t, log.modeToText(x), log.modeToText(y)) for t, x, y in zip(log.flags["timeUs"], log.flags["enable"], log.flags["disable"])] # flight mode changes in human readable format

log.outputCsv("raw")                                                              # output raw data as csv, comparable to outputting from BB log explorer
log.outputCsv("si")                                                               # output SI data as csv
log.outputCsv("si", path="my/path/for/logOutput/")                                # output SI data into a specific path

log.addToRerun(name="BFL", clockOffsetSeconds=0.)                                 # send data Rerun (does not do init, connect, etc. just adding data)
```

There are also command line tools you can use after installing the wheel:
```
$ bfl2csv --help     # shorthand for the log.outputCsv() above
$ bfl2rerun --help   # shorthand for the log.addToRerun() above
```
