# Notes for agents: support/LogAnalysis

## Plotters
- `flightPlotter.py` (matplotlib, through `indiflightPlotter.py`) and `webPlotter.py` (browser) show the same panels and
  3D view of an Indiflight log. Both use the `pyFlightPlotter` package (tudelft/pyFlightPlotter, branch `js-version`, installed with pip from GitHub). When you change a panel in one, port the
  change to the other.
- `webPlotter.py` is only a consumer. It decodes with `indiflight_log_tools`, adds derived signals (body velocities,
  inertial acceleration, servo rates), declares the layout, and calls `WebPlotter.show()`. Generic plotting features
  belong in the library: `pyFlightPlotter/web/` on its `js-version` branch. Keep Indiflight knowledge out of
  the library (see `AGENTS.md` in that repo).
- The configurator (`../indiflight-configurator-web`, Flight Plotter tab) has its own JS port of this layout in
  `src/js/indiflight/flightPlotterLayout.js`. It uses the blackbox viewer's decoder and unit conversions, not
  `indiflight_log_tools`. The signal and unit definitions are duplicated on purpose for now and will be harmonised
  later, either as a shared definition or by encoding them in the BFL. `indiflight_log_tools` v0.7.0 is the reference
  for SI conversions.

## Environment
- The venv is `~/.indiflight-venv`. Run `pip install -r support/LogAnalysis/requirements.txt` to install
  pyFlightPlotter from GitHub. It used to be a submodule here, which shadowed the installed package. To pick up new
  library commits, add `--force-reinstall --no-deps`, because pip doesn't re-resolve a branch reference that is already
  installed. When working on the library, clone it outside this folder and `pip install -e` the clone. A clone inside
  `support/LogAnalysis/` brings the shadowing back.
- `webPlotter.py LOG.BFL --id N --no-browser --port P` prints the URL and serves until Ctrl+C. The session data is at
  `/s/<id>/meta.json` and `/s/<id>/data.bin`, which you can fetch with curl to check a run headlessly.
