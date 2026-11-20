# Notes for agents: support/LogAnalysis

## Plotters
- `flightPlotter.py` (matplotlib, through `indiflightPlotter.py`) and `webPlotter.py` (browser) show the same panels and
  3D view of an Indiflight log. Both use the `pyFlightPlotter` submodule. When you change a panel in one, port the
  change to the other.
- `webPlotter.py` is only a consumer. It decodes with `indiflight_log_tools`, adds derived signals (body velocities,
  inertial acceleration, servo rates), declares the layout, and calls `WebPlotter.show()`. Generic plotting features
  belong in the library: `pyFlightPlotter/web/` on the submodule's `js-version` branch. Keep Indiflight knowledge out of
  the library (see `pyFlightPlotter/AGENTS.md`).
- The configurator (`../indiflight-configurator-web`, Flight Plotter tab) has its own JS port of this layout in
  `src/js/indiflight/flightPlotterLayout.js`. It uses the blackbox viewer's decoder and unit conversions, not
  `indiflight_log_tools`. The signal and unit definitions are duplicated on purpose for now and will be harmonised
  later, either as a shared definition or by encoding them in the BFL. `indiflight_log_tools` v0.7.0 is the reference
  for SI conversions.

## Environment
- The venv is `~/.indiflight-venv`. Install the submodule as editable in compat mode:
  `pip install -e support/LogAnalysis/pyFlightPlotter --config-settings editable_mode=compat`.
  The default editable mode installs an import finder that runs after the normal path search. Scripts started from this
  folder have the submodule directory `pyFlightPlotter/` on `sys.path[0]`, so the empty namespace package wins and
  `from pyFlightPlotter import Craft3D` fails with "unknown location". A non-editable install also works.
- `webPlotter.py LOG.BFL --id N --no-browser --port P` prints the URL and serves until Ctrl+C. The session data is at
  `/s/<id>/meta.json` and `/s/<id>/data.bin`, which you can fetch with curl to check a run headlessly.
