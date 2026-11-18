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

6. **`configs/profiles/*.txt`**: Add the new config option(s) with default setting
   at the appropriate places (if any).

To **modify** a setting (rename, change range, type or default), apply the change
consistently across all of these places. A retype also needs a PG version bump.
A rename changes the CLI name, so old `diff`/`dump` files will no longer apply it.

## Position control structure

`flight/pos_ctl.c` holds the position profile, waypoints and the
position -> velocity -> acceleration setpoint cascade (`updatePosCtl()`).
The mapping from the inertial acceleration setpoint (`accSpNedFromPos`) to
attitude/thrust/rate setpoints is selected per position profile with
`position_attitude_mapping` and lives in its own file:

- `NDI`  `flight/pos_att_ndi.c`: multirotor dynamic inversion (default).
- `INDI` `flight/pos_att_indi.c`: incremental with yaw constraint, hybrids, untested.
- `DF`   `flight/pos_att_df.c`: differential flatness, Tal (2022). Its craft
  model (`PHI`, `K`, `n`) is hardcoded for the DarkO tailsitter. Default with
  `INDI_IS_TAILSITTER` or `LEARNER_IS_TAILSITTER`.

New mappings get a file `pos_att_<name>.c`, a prototype in `flight/pos_att.h`,
a `position_attitude_mapping_t` value with a matching entry in
`lookupTablePositionAttitudeMapping` (`cli/settings.c`), a case in
`updatePosCtl()`, and both lists in `make/source.mk`.

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

## Adding or modifying a flight mode (RC mode box)

A flight mode is a switch ("box", `boxId_e`) plus a flag (`flightModeFlags_e`).
Example: `BOXWAYPOINTS` / `WAYPOINT_MODE`, which sits on top of `BOXPOSCTL` /
`POSITION_MODE`. Grep for a sibling (e.g. `BOXPOSCTL`, `POSITION_MODE`) and
copy each occurrence.

1. **`src/main/fc/rc_modes.h`**: add the box to `boxId_e` in the flight mode
   block (before `BOXID_FLIGHTMODE_LAST`, update it if you append at the end).
   - Mode switch assignments are stored in EEPROM by `boxId_e` index. Inserting,
     removing or reusing a slot remaps users' switches, so **bump the PG version**
     of `modeActivationConditions` in `src/main/fc/rc_modes.c` (users' `aux`
     settings then reset; mention this to the user).
2. **`src/main/fc/runtime_config.h`**: add the flag to `flightModeFlags_e` (one
   bit) and map the box to it in `BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER`. A
   static assert checks that the map has `BOXID_FLIGHTMODE_LAST + 1` entries.
3. **`src/main/msp/msp_box.c`**: add `{ .boxId, .boxName, .permanentId }` to
   `boxes[]` and enable it with `BME(...)` in `initActiveBoxIds()` under the
   matching `#ifdef`.
   - `permanentId` is what the configurator and CLI `aux` lines use. Use a
     new, unused id; never reuse the id of a removed box (old `diff` files would
     silently map to the new mode).
4. **`src/main/fc/core.c`**, `processRxModes()`
   (near `BOXPOSCTL`): set/clear the flag from `IS_RC_MODE_ACTIVE(BOX...)` with
   `ENABLE_FLIGHT_MODE`/`DISABLE_FLIGHT_MODE`. Modes that depend on another mode
   (e.g. `WAYPOINT_MODE` needs `POSITION_MODE`) are gated here, so code
   elsewhere can test `FLIGHT_MODE(POSITION_MODE)` alone.
5. **Users of the flag**: `FLIGHT_MODE(...)` checks in e.g. `flight/pos_ctl.c`,
   `flight/indi.c`, `flight/ekf.c` (`ekf_should_use`), `flight/throw.c`
   (`autoMode`).
6. **Tooling**: `src/utils/indiflight_mockup_interface.py` (`flightModeFlags`
   bits) and the `modeToText()` box names in
   `support/LogAnalysis/indiflight_log_tools/indiflight_log_tools/indiflight_log_importer.py`.
   The blackbox `flightModeFlags` field logs the *box* (switch) states
   (`rcModeActivationMask`, `boxId_e` bit order), not `flightModeFlags_e`.

Waypoint mode specifics: the plan is the `positionWaypoints[]` table in
`flight/pos_ctl.c`. Edge handling (start at the first waypoint, resume, arrest
motion on switch-off, auto-off when finished) is in `updateWaypointMode()`.
`core.c` only enables `WAYPOINT_MODE` on the rising edge of
`BOXWAYPOINTS && POSITION_MODE`, so the auto-off sticks until the switch is cycled.

## Adding a serial peripheral driver (e.g. Feetech servos, T4 board)

Example: `USE_SERVOS_FEETECH` (`io/feetech.c`), modelled on `USE_ACTUATORS_T4`
(`io/t4.c`). A driver that owns a UART is enabled by a compile flag and is
bound to a port by a serial function bit. Grep for a sibling (e.g.
`FUNCTION_SERVOS_FEETECH`, `USE_SERVOS_FEETECH`) and copy each occurrence.

1. **`src/main/io/serial.h`**: add a bit to `serialPortFunction_e`
   (`FUNCTION_SERVOS_FEETECH = (1 << 22)`). The bit index is stored in
   `serial` CLI lines and used by the configurator, so never renumber or reuse
   one. `functionMask` is `uint32_t`, so bits up to 31 are fine for CLI and
   MSP2 (the legacy MSP `MSP_CF_SERIAL_CONFIG` only carries 16 bits). Only add
   it to `ALL_FUNCTIONS_SHARABLE_WITH_MSP` / `TELEMETRY_*_MASK` in
   `io/serial.c` if the port can really be shared.
2. **Driver files** in `src/main/io/`: `<name>.c` (wrapped in
   `#if defined(USE_<FLAG>)`, with `#error` checks for required flags),
   `<name>.h` (public API), and for a protocol `<name>_protocol.h` holding
   only instruction/register/frame defines. API pattern:
   `init<Name>()` (looks up `findSerialPortConfig(FUNCTION_...)`),
   `configure<Name>Port()` (`openSerialPort(...)`), `free<Name>Port()`,
   and loop functions that return early when the port is `NULL`.
   Half-duplex buses use
   `SERIAL_BIDIR | SERIAL_BIDIR_PP | SERIAL_BIDIR_NOPULL`; the receiver then
   sees an echo of every frame it sends, so the parser must reject it.
3. **`make/source.mk`**: add `io/<name>.c` next to `io/t4.c`. It appears in
   **two** lists (the common sources and the speed-optimised list). A file
   that is missing here is silently never compiled.
4. **`src/main/fc/init.c`**: call `init...()` and `configure...Port()` under
   the flag, next to `initActuatorsT4()`.
5. **`src/main/fc/core.c`**, `subTaskInnerLoopApplyToActuators()`: call the
   receive/send functions under the flag, inside the non-`HIL_BUILD` branch
   next to the T4 block. `currentTimeUs` is in scope; time-slice inside the
   driver (`cmpTimeUs`) instead of counting loop iterations.
6. **Servo interface** (`flight/servos.h`): drivers read commands from
   `servo_normalized[]` (±1, zeroed by `core.c` when disarmed) and write
   measured angles to `servo_feedback[]` in centidegrees, where 10000 cdeg =
   normalized 1 (`core.c` computes `servo_normalized_feedback = servo_feedback * 1e-4`,
   used by INDI and the learner). Only one driver may fill `servo_feedback[]`;
   `feetech.c` `#error`s if `USE_ACTUATORS_T4` is also set.
   `USE_SERVO_FEEDBACK` (defined in `servos.h` when any such driver is
   enabled) guards the `servo_feedback` blackbox fields; add new drivers to
   that `#if`. Blackbox logs 4 feedback fields regardless of the driver's
   servo count.
7. **Configurator** (separate repo
   [tudelft/indiflight-configurator](https://github.com/tudelft/indiflight-configurator),
   not part of this repo): add the function to `src/js/msp/MSPHelper.js`
   (`SERIALPORT_FUNCTIONS`, value = bit index), `src/js/tabs/ports.js`
   (`functionRules`, `groups` decides the dropdown) and
   `locales/en/messages.json` (`portsFunction_<NAME>`).
8. **`configs/profiles/*.txt`**: list the flag (commented `## #define ...`
   if off) next to `USE_ACTUATORS_T4`, and the `serial <port> <mask> ...`
   line uses the bit value (`4194304` for Feetech).

Compile check: add `-DUSE_SERVOS -DUSE_SERVOS_FEETECH` (or
`-DUSE_ACTUATORS_T4`) to `EXTRA_FLAGS`; `make/local_.mk` doesn't enable
either.

### Feetech STS specifics (`io/feetech.c`)

- 1 Mbaud, frames `FF FF ID LEN INSTR PARAMS... CHK`, `LEN = params + 2`,
  `CHK = ~sum(ID..last param)`. Registers are little-endian.
- Servo `i` has bus ID `i + 1` and maps to `servo_normalized[i]` /
  `servo_feedback[i]`. `FEETECH_SERVO_COUNT` (in `feetech.h`) sets how many.
- One frame per 1 ms slot, rotating write-goal(servo 1), read(servo 1),
  write(servo 2), read(servo 2), ... so each servo updates at
  `1000 / (2 * FEETECH_SERVO_COUNT)` Hz.
- Telemetry is a READ of 8 registers from `PRESENT_POSITION_L` (position,
  speed, load, voltage, temperature), reply `LEN == 10`. The parser accepts
  only that length, which also rejects the half-duplex echoes of our own
  READ (`LEN 4`) and WRITE (`LEN 9`) and the WRITE status replies (`LEN 2`).
  If you change the request size or the write frame, make sure these lengths
  stay distinct.
- Position, speed and load are sign-magnitude (sign bit 15, 15 and 10).
- Command scaling: normalized ±1 = ±100 deg; 4096 steps/rev, centre 2048.

## Building

- Run `make STM32H743 YES=y -j16` as the basic compile check (`YES=y` skips the
  dirty-tree prompt). Output goes to `obj/`.
- The bare `STM32H743` target defines no gyro/acc and none of the indiflight
  features (`USE_EKF`, `USE_LOCAL_POSITION`, `USE_BARO`, ...), so it fails in
  `gyro_init.c`/`acceleration_init.c` and doesn't compile most indiflight code.
  To compile with the features enabled, pass the flags from `make/local_.mk`:
  ```sh
  FLAGS=$(make -s -f make/local_.mk -f <(printf 'p:\n\t@echo $(EXTRA_FLAGS)\n') p)
  make STM32H743 YES=y -j16 EXTRA_FLAGS="$FLAGS"
  ```
- The SITL target currently fails in unrelated driver code (`atomic.h`,
  `dshot_dpwm.c`), so don't use it as a compile check.

The configurator lives in a separate repo (`tudelft/indiflight-configurator`, branch `indi-web`). It has its own `AGENTS.md`.

## Building with a board config

```sh
make BOARD=MTKS-H743 YES=y          # board configs are in configs/boards/*.txt
```

`YES=y` skips the "uncommitted changes" prompt. Bare MCU targets such as `make STM32H743` don't build,
because they define no sensors. Use a `BOARD=`.

## Adding an MSP command

**Pick a code.** Indiflight-specific commands are MSPv2 codes in the `0x40xx` range, defined in
`src/main/msp/msp_protocol_v2_indiflight.h`. Upstream Betaflight uses `0x30xx`
(`msp_protocol_v2_betaflight.h`). Don't reuse or renumber a code, because released configurators depend on it.
Codes above 255 are sent as MSPv2 automatically, on both sides.

**Handle it in `src/main/msp/msp.c`.** `mspFcProcessCommand()` tries these handlers in order:

| Function | Use for |
|---|---|
| `mspCommonProcessOutCommand()` / `mspProcessOutCommand()` | Replies that don't need any request payload |
| `mspFcProcessOutCommandWithArg()` | Replies that read a request payload (`src`) and write a reply (`dst`) |
| `mspProcessInCommand()` / `mspCommonProcessInCommand()` | "Set" commands with no reply data |

- **Unknown or failed commands:** returning `MSP_RESULT_ERROR`, or falling through as unknown, sends an MSP error reply (direction `!`). The configurator treats that as "not supported". For anything that can fail at runtime, put a status byte first in the reply instead, so the client can tell what went wrong.
- **Sizes:** use `sbufBytesRemaining()` on both buffers, because the reply buffer size depends on the transport. `MSP_PORT_OUTBUF_SIZE` (see `msp_serial.h`) is 4112 bytes on targets with flash or SD logging, and 400 bytes otherwise. Requests are at most `MSP_PORT_INBUF_SIZE` (192 bytes).
- **Larger features:** put the logic in their own module and keep only a `case` in `msp.c`, guarded by a feature macro. See `blackbox/blackbox_download.c` and `USE_MSP_LOG_DOWNLOAD` in `target/common_post.h`.
- **Payload format:** document it next to the code. All integers are little-endian (`sbufWriteU16/U32`).

**Update the configurator.** Add the code to `src/js/msp/MSPCodes.js`, using the same name as the firmware.
Put the encode/decode in `src/js/msp/MSPHelper.js`, or in a pure module it imports so the code can be
unit-tested. Call it only from a store or composable, never from a component.

**Version.** Bump `API_VERSION_MINOR` in `msp_protocol.h` only when releasing. Detect new commands by their
reply, not by the API version.

## Unit tests

The tests are in `src/test/unit/*_unittest.cc`, use googletest, and are built by `src/test/Makefile`.

- **Adding a test:** any new `foo_unittest.cc` is picked up automatically. Declare what it compiles in the Makefile:

  ```make
  foo_unittest_SRC := \
          $(USER_DIR)/path/to/foo.c \
          $(USER_DIR)/common/streambuf.c
  foo_unittest_DEFINES := \
          USE_SOMETHING=
  ```

- **Headers:** only the listed sources are compiled, against `src/test/unit/platform.h` and `target.h`. `common_pre.h` and `common_post.h` are *not* included, so feature macros must be set in `_DEFINES`.
- **Stubs:** everything else the code calls has to be stubbed in the test file, inside `extern "C"`. For parameter groups, define the `xxx_System` variable (e.g. `blackboxConfig_t blackboxConfig_System;`).
- **Running:** `make test` (or `test-all`, as CI runs it with `EXTRA_FLAGS=-Werror`) from the repo root. A single test is `make -C src/test test_foo_unittest`. The binary ends up in `obj/test/foo_unittest/`, and accepts `--gtest_filter=...`.
- **Compiler:** the Makefile requires clang 7–14 (the newer `clang` 18 is rejected). On a machine with clang-14 and the GCC 13 C++ headers:

  ```sh
  make -C src/test CC=clang-14 \
    CXX="clang++-14 -isystem /usr/include/c++/13 -isystem /usr/include/x86_64-linux-gnu/c++/13 -L<dir>" \
    test_foo_unittest
  ```

  `<dir>` must contain a `libBlocksRuntime.a`. An empty one is enough if the tests don't use blocks: `ar rcs libBlocksRuntime.a empty.o`.
- **Known failures on this branch:** `blackbox_unittest` currently doesn't build, because `solveActiveSet.h` is not on the include path.
- **What makes a useful test:** see `blackbox_download_unittest.cc` for a test that replaces a driver with an adversarial fake. Its fake asyncfatfs randomly delays, splits and stalls operations, and the test checks the module's invariants, such as that no file handles leak and the working directory is restored.

## Blackbox log transfer over MSP (brief)

Implemented in `src/main/blackbox/blackbox_download.c`. The wire format is documented in `blackbox_download.h`.
The configurator side is the "Log Download" tab.

The commands are `MSP2_INDI_BLACKBOX_LOG_LIST/READ/CLOSE/ERASE_ALL` (`0x4006`–`0x4009`). Every reply starts with
a status byte.

- **Where logs come from:** the configured blackbox device. If that is neither flash nor SD, the SD card is used if one is inserted, otherwise the flash.
  - SD card: the files in `/logs`.
  - Flash: logs are stored back to back. They are found by scanning for the blackbox header line, in steps of about 250 ms (status `IN_PROGRESS` with a percentage until done), and are named `LOG00001.BFL`, ….
- **Reading:** a READ returns a chunk at a given offset, in the same layout as `MSP_DATAFLASH_READ`, optionally Huffman-compressed.
  - The SD file stays open between requests, so sequential reads don't seek.
  - asyncfatfs is asynchronous, so the handlers poll it themselves, for at most 250 ms per request.
- **Safety:** requests are refused while armed or while blackbox is logging. Blackbox always wins:
  - Before starting a log it calls `blackboxDownloadReleaseForLogging()`, which releases the card without blocking.
  - The card is also released after 2 s without requests.
  - While a download holds the card, the working directory is `/logs`. It is restored to the root afterwards, so blackbox still finds `/logs`.
- **Erase all:**
  - Flash: a chip erase.
  - SD card: deletes the files in `/logs` (only while the client keeps polling), then calls `afatfs_rebuildFreeFile()`. Without that, deleted files' space never returns to `FREESPAC.E`, the contiguous region new logs are allocated from.
- **Integrity:** each MSP frame has a CRC8, and the client re-requests a chunk on a CRC error, a wrong offset or a timeout. There is no whole-file checksum; the client only compares the received size with the listed size.
