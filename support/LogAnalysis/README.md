# Log Analysis

Python tools to decode and plot Indiflight blackbox logs (`.BFL`).

- [`indiflight_log_tools`](indiflight_log_tools/) decodes logs into pandas
  dataframes in SI units, and provides the `bfl2csv` and `bfl2rerun` commands.
  Published on [PyPI](https://pypi.org/project/indiflight_log_tools/); the
  source is in this folder.
- [`pyFlightPlotter`](https://github.com/tudelft/pyFlightPlotter) is the
  plotting engine: synchronised time-series plots, 3D renders of the craft,
  video sync.
- [`indiflightPlotter.py`](indiflightPlotter.py) builds on pyFlightPlotter with
  ready-made layouts for Indiflight logs.

## Installation

Supported: **Linux** (x86_64, aarch64) and **macOS** (Intel, Apple Silicon)
with **Python 3.10 or newer**. Prebuilt packages are available for these, so
no compiler is needed.

**Windows: use WSL.** The log decoder is C code that does not build natively
on Windows. Install WSL with Ubuntu
(`wsl --install` in PowerShell, see
[the Microsoft guide](https://learn.microsoft.com/en-us/windows/wsl/install)),
then follow the Linux instructions below inside the Ubuntu terminal. Plot
windows open on the Windows desktop (WSLg, Windows 10 21H2+ or Windows 11).

### 1. Create a virtual environment (recommended)

This keeps the packages separate from your system Python and avoids version
conflicts.

```sh
python3 -m venv ~/indiflight-venv
source ~/indiflight-venv/bin/activate     # run this again in every new terminal
```

On Ubuntu/Debian/WSL you may first need `sudo apt install python3-venv python3-tk`
(`python3-tk` provides the plot window backend).

### 2. Install the packages

```sh
pip install indiflight_log_tools
pip install "pyFlightPlotter @ git+https://github.com/tudelft/pyFlightPlotter"
```

Optional extras:

```sh
pip install "indiflight_log_tools[rerun]"                                    # for bfl2rerun
pip install "pyFlightPlotter[video] @ git+https://github.com/tudelft/pyFlightPlotter"  # video sync (OpenCV)
```

Upgrading from a version installed before 0.3.0? Run
`pip uninstall indiflight_log_tools` first, so the old `bfl2csv`/`bfl2rerun`
scripts are removed.

### 3. Check

```sh
python -c "from indiflight_log_tools import IndiflightLog; import pyFlightPlotter; print('ok')"
```

## Usage

Plot a log with the Indiflight layout (run from this folder):

```sh
python indiflightPlotter.py /path/to/LOG00001.BFL                    # multirotor
python indiflightPlotter.py /path/to/LOG00001.BFL --type tailsitter
python indiflightPlotter.py --help
```

Convert a log to CSV:

```sh
bfl2csv /path/to/LOG00001.BFL              # writes CSVs next to the log
bfl2csv /path/to/LOG00001.BFL -u si        # SI units instead of raw
```

Load a log in your own scripts:

```python
from indiflight_log_tools import IndiflightLog

log = IndiflightLog("/path/to/LOG00001.BFL")
log.data          # pandas dataframe in SI units
log.raw           # raw units
log.parameters    # settings from the log header
```

See [`indiflight_log_tools/README.md`](indiflight_log_tools/README.md) for the
full API.

The other scripts in this folder are research scripts for specific
experiments. Some need extra packages (e.g. `tqdm`, `torch`, `sympy`,
`numpy-quaternion`); install them with `pip` as needed.
