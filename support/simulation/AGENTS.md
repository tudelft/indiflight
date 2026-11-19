# Notes for agents: PyNDIflight simulation and firmware SIL

These notes cover how the Python sim (`PyNDIflight/`, `example*Sim.py`) and
the firmware MOCKUP target (`src/main/mockup/`, `src/main/target/MOCKUP/`)
fit together. They also cover what to check when you make a sim craft match
a profile in `configs/profiles/`.

## Data flow between Python and firmware

- The Python bindings are `src/utils/indiflight_mockup_interface.py` (ctypes on
  `obj/main/indiflight_MOCKUP.so`). The glue code is
  `PyNDIflight/interfaces.py:IndiflightSITLWrapper`.
- `setImu(gyro, acc)` takes values in the **sensor frame**: rad/s and N/kg.
  The firmware then applies the sensor alignment (`GYRO_1_ALIGN`, default `CW0_DEG`)
  and the board alignment to both gyro and acc
  (`sensors/gyro.c:gyroUpdate`, `sensors/acceleration.c:accUpdate`).
  This means the sim's `IMU(qBody=...)` must equal the board alignment rotation.
- `setMotorSpeed` takes rad/s and feeds the fake DShot telemetry
  (`isDshotTelemetryActive()` always returns true in MOCKUP).
  `getMotorOutputCommands` returns `motor_normalized` in [0, 1].
- Both calls are limited to `getMotorCount()`, which is `mixerRuntime.motorCount`.
  The profile's `mixer` line therefore has to match the rotor count of the sim craft.
- MOCKUP does **not** apply `motor_output_reordering`. That setting only takes effect in
  the hardware DShot/PWM drivers. Sim rotor `i` is INDI actuator `i`, which is
  column `i` of the `indi_act_g1_*` settings.

## Frames and conventions

- The body frame is FRD everywhere, and the inertial frame is NED.
- Quaternions are scalar-first `[w, x, y, z]`, in the sim, in `setMocap`, and in
  `IMU(qBody=...)`. `quatRotate(q, v)` is the active rotation `R(q) v`
  (from body to parent).
- Board alignment: the firmware computes `v_body = Rz(yaw) Ry(pitch) Rx(roll) v_sensor`
  (`common/maths.c:rotationMatrix_of_fp_euler`). In scipy terms, that is
  `Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True)`, followed by
  reordering the quaternion to scalar-first.
- `acc_offset` is the IMU position relative to the CG in mm, in the **FRD body
  frame**. This follows from how `flight/learner.c` uses it and how CineRat matches the quad sim.
  The FLU comment in `sensors/acceleration.h` is wrong.
  The sim's `IMU(r=...)` equals `acc_offset * 1e-3`.
- Rotor yaw torque in the sim is `+cm * T` (`helpers.py:rotorForcesMoments`).
  The sign of `cm` equals the sign of `indi_act_g1_yaw`. `cm < 0` means the prop spins CW seen from above.

## Accelerometer calibration in MOCKUP

- `acc.dev.acc_1G` is **256** in MOCKUP (the default from `acceleration_init.c`;
  the fake driver doesn't override it). Real boards use about 2048.
- `acc_calibration = x,y,z,1` is subtracted from the body-aligned raw readings
  in counts. A real-board calibration therefore becomes a bias roughly 8x larger in SIL.
  Example: HexDemo's `63,-92,5` gives about 0.25 g.
- Either zero the setting for SIL or give the sim IMU the matching bias in the sensor
  frame, `R_board^T * trims * 9.81 / 256`. `exampleHexSim.py` does the latter.

## Matching a sim craft to a profile's effectiveness

The firmware only sees specific (per-mass or per-inertia) effectiveness for each unit
of motor command, so the absolute mass and size of the craft are free.
`G1 = Tmax / m` or `Tmax * lever / I`, with `Tmax = k * wmax^2`:

| profile setting           | sim quantity                 |
|---------------------------|------------------------------|
| `indi_act_g1_fz / 100`    | `-Tmax / m` [N/kg]           |
| `indi_act_g1_roll / 10`   | `-y_i * Tmax / Ixx` [rad/s²] |
| `indi_act_g1_pitch / 10`  | `+x_i * Tmax / Iyy` [rad/s²] |
| `indi_act_g1_yaw / 10`    | `+cm_i * Tmax / Izz` [rad/s²]|
| `indi_act_g2_yaw * 1e-5`  | `I_rotor / Izz` [-]          |
| `indi_act_max_rpm`        | `wmax` (rpm to rad/s)        |
| `indi_act_time_constant_ms` | `tau`                      |
| `indi_act_nonlinearity / 100` | `kESC`                   |

- You can read the rotor geometry directly off the signs and ratios of
  `g1_roll` and `g1_pitch`. A worked example is `exampleHexSim.py`.
- Check that the profile is self-consistent:
  `n_motors * (-g1_fz/100) * (hover_rpm/max_rpm)^2` should be about 9.81.
  HexDemo_post_fix fails this check: -700 with 21600/37000 gives about 14.3.
  The hover rpm only seeds the INDI linearization when `indi_use_rpm_dot_feedback = 1`,
  so match G1 first.
- The sim does not model rotor inertia torques (they are commented out in
  `Craft.tick`). G2 only matters on the firmware side.
- `MultiRotor.calculateG1G2()` and `checkHover()` refer to a nonexistent
  `self.rotors`. They are stale, so don't call them. Compute G1 from `r_X`, `r_ax`, `r_k`,
  `r_wmax`, `r_cm` and the mass/inertia instead.

## Learner build flags that override the profile

These flags are set in the profile build-flag headers (for example HexDemo):

- `LEARNER_HARDCODE_MAXRPM` forces the learned max rpm to **37000**
  (`flight/learner.c`). Keep the sim's `wmax` at 37000 rpm, or the learned G1 will be scaled wrong.
- `LEARNER_HARDCODE_LINEAR_FORCES` forces the learned `g1_fz` to **-438**, whatever
  the profile says. The value is per motor and fits a hex hovering at about 21600 of 37000 rpm.

## Running and testing

- Docker: `-e SIM=<script name without .py> -e PROFILE=<profile name without .txt>`
  (see `entrypoint.sh`). Example: `SIM=exampleHexSim PROFILE=HexDemo_post_fix`.
- Build the SIL library with `make TARGET=MOCKUP so` from the repo root.
- On the local venv (`~/.indiflight-venv`), numba fails to import with NumPy > 2.0.
  For offline checks of craft parameters, stub `numba.njit` as a pass-through
  decorator on `PYTHONPATH`. That is slow, but fine for one-off evaluations.
