# Notes for coding agents

Indiflight is a Betaflight fork. Settings live in parameter groups (PGs). The CLI,
blackbox header and defaults are wired up by hand, so a new setting has to be added
in several places.

## Adding or modifying a profile setting

Example: the position profile (`positionProfile_t`, `PG_POSITION_PROFILE`), e.g.
`position_use_thrust_attenuation` / `use_spf_attenuation`. The INDI (`indiProfile_t`,
`PG_INDI_PROFILE`), PID (`pidProfile_t`) and rate profiles follow the same pattern.
To add a setting, `grep` for an existing sibling and copy each of its occurrences.

1. **`src/main/fc/parameter_names.h`**: add the CLI name next to its siblings:
   `#define PARAM_NAME_POSITION_FOO "position_foo"`.
   CLI names are lowercase snake_case and start with their group's prefix.

2. **Profile struct header** (e.g. `src/main/flight/pos_ctl.h`):
   - Add the stored field to the profile struct (`positionProfile_t`). Use only
     integer types (`uint8_t`/`int8_t`/`uint16_t`/`int16_t`/`uint32_t`); the CLI
     has no float type. Store scaled integers (e.g. `* 10`, `cm/s`) and write the
     unit in a comment.
   - Fields above the `// --- inaccessible from CLI` comment are exposed; fields
     below it are not.
   - If the code uses a runtime copy in SI units/`float`/`bool`, also add the field
     to the runtime struct (`positionRuntime_t`).

3. **Profile source** (e.g. `src/main/flight/pos_ctl.c`):
   - Set the default in the reset function (`pgResetFn_positionProfiles`). This
     must happen for every profile index.
   - Convert into the runtime struct in `initPositionRuntime()`, e.g.
     `posRuntime.foo = p->foo * 0.01f;`. This function runs at init and on
     profile change.
   - Use the runtime value (`posRuntime.foo`) in the control code, not the PG.
   - **Bump the PG version** (last argument of `PG_REGISTER_ARRAY_WITH_RESET_FN(...)`)
     whenever the struct layout changes (field added, removed, reordered or retyped).
     Users' stored values for that PG then reset to defaults; mention this to the
     user.

4. **`src/main/cli/settings.c`**: add an entry to `valueTable[]` in the matching
   `#ifdef` block, next to its siblings:
   ```c
   { PARAM_NAME_POSITION_FOO, VAR_UINT8 | PROFILE_POSITION_VALUE, .config.minmaxUnsigned = { 0, 1 }, PG_POSITION_PROFILE, offsetof(positionProfile_t, foo) },
   ```
   - The `VAR_*` type must match the struct field's type.
   - The section flag must match the profile: `PROFILE_POSITION_VALUE`,
     `PROFILE_INDI_VALUE`, `PROFILE_VALUE` (PID), `PROFILE_RATE_VALUE`, or
     `MASTER_VALUE` for non-profile configs.
   - Use `.config.minmax` for signed types and `.config.minmaxUnsigned` for unsigned.
   - Booleans: this codebase uses `uint8_t` with `minmaxUnsigned = { 0, 1 }`.
     `MODE_LOOKUP` with `TABLE_OFF_ON` (ON/OFF strings) is the alternative.

5. **`src/main/blackbox/blackbox.c`**: log the value in the header, next to its
   siblings (position profile values use `posProfile->...`):
   ```c
   BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_FOO, "%d", posProfile->foo);
   ```

To **modify** a setting (rename, change range, type or default), apply the change
consistently across all of these places. A retype also needs a PG version bump.
A rename changes the CLI name, so old `diff`/`dump` files will no longer apply it.

## Adding or modifying a blackbox log signal

Logged signals (per-loop fields, as opposed to header values) are defined only in
`src/main/blackbox/blackbox.c`, but each one appears in **five places**. All five
must agree in order, count, type and condition. A mismatch does not fail to
compile; it silently corrupts the log, because the decoder expects one field per
table row, in table order. Example: `d` (`indiRun.d`) next to `u` under
`CONDITION(INDI)`. To add a signal, grep for a sibling (e.g. `u_state`) and copy
each of its occurrences.

1. **Field definition table `blackboxMainFields[]`**: add one row per array
   element, with the index as the second column:
   ```c
   {"d", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
   ```
   - The position in the table defines the order in the log. Place the rows
     where the writes happen in steps 3 and 4.
   - `SIGNED`/`UNSIGNED` must match the struct type, and so must the
     `SIGNED_VB`/`UNSIGNED_VB` I-encoding.
   - P-frames usually use `PREDICT(PREVIOUS)` (delta to the last frame).
     `.Pencode` must match the writer used in step 4: `SIGNED_VB` for
     `blackboxWriteSigned16VBArray`/`blackboxWriteSignedVB`, `TAG8_8SVB` for
     `blackboxWriteTag8_8SVB` (at most 8 values per call; the field group
     must be contiguous in the table).
   - `CONDITION(...)` controls when the field is logged (e.g. `INDI`, `ALWAYS`).
     The same condition must guard the writes in steps 3 and 4.
   - Rows for `MAXU`-sized arrays are hardcoded as 8 rows (assumes
     `MAX_SUPPORTED_MOTORS == 8`).
2. **Frame struct `blackboxMainState_t`**: add the storage field, inside the
   same `#ifdef` (e.g. `USE_INDI`). Use an integer type (`int16_t`, `uint16_t`,
   `int32_t`).
3. **I-frame writer `writeIntraframe()`**: write the absolute value inside the
   matching `if (testBlackboxCondition(CONDITION(...)))` block, e.g.
   `blackboxWriteSigned16VBArray(blackboxCurrent->d, MAXU);`.
4. **P-frame writer `writeInterframe()`**: write the delta at the same relative
   position, e.g. `arraySubInt16(deltas16, current, last, n)`, then copy into
   `deltas[]` (int32) and call `blackboxWriteTag8_8SVB(deltas, n)`.
5. **Fill function `loadMainState()`**: convert the float source to the
   integer field with a fixed scale, e.g.
   `blackboxCurrent->d[i] = lrintf(indiRun.d[i] * UNIT_FLOAT_TO_SIGNED16VB);`.
   Common scales: `UNIT_FLOAT_TO_SIGNED16VB` for [-1, 1] values, `*100` for
   N/kg to cm/s², `blackboxHighResolutionScale` for deg/s. Choose a scale that
   keeps the value inside the int16 range.

Then update the Python decoder
(`support/LogAnalysis/indiflight_log_tools/indiflight_log_tools/indiflight_log_importer.py`):
add an unscaling rule for the new column (the `col.startswith(...)` chain).
Watch out for prefix clashes (`d[` vs `dv`, `u[` vs `u_state`). If the column
should end up in the exported time series, add it to `timeseries` as well.

To **modify** a signal, change all five places together. Removing or reordering
rows changes the log format for every field after it.

Header values (constant per log, e.g. settings) are a separate mechanism: a single
`BLACKBOX_PRINT_HEADER_LINE(name, fmt, value)` in `blackboxWriteSysinfo()`.

## Building

- Run `make STM32H743 YES=y -j16` as the compile check (`YES=y` skips the
  dirty-tree prompt). Output goes to `obj/`.
- The SITL target currently fails in unrelated driver code (`atomic.h`,
  `dshot_dpwm.c`), so don't use it as a compile check.
