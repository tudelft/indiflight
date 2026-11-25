# Indiflight MSP extensions

Indiflight keeps every upstream Betaflight bit where upstream has it. Its own
arming-disable reasons and flight modes live in separate 32-bit words, which are
appended to the end of existing messages. A stock configurator reads the
upstream fields and ignores the extra bytes. A configurator that knows about
indiflight reads them only when they are present.

## MSP_STATUS (101) and MSP_STATUS_EX (150)

The upstream payload is unchanged, up to and including the API 1.48 fields
`BATTERY_PROFILE_COUNT` (u8) and the current battery profile (u8). Indiflight
builds (`USE_INDI`) then append:

| type | content |
|------|---------|
| u8   | number of indiflight arming-disable flags (`INDIFLIGHT_ARMING_DISABLE_FLAGS_COUNT`) |
| u32  | indiflight arming-disable flags, bit 0 = `EKF_OR_SP` (EKF needed but not converged, or geofence error/action) |
| u32  | indiflight flight modes: bit 0 = INDI, bit 1 = POSITION CONTROL, bit 2 = WAYPOINTS (bits 3..5 reserved for learner, catapult, NN) |

### What a configurator has to do

- Parse the upstream fields exactly as for upstream firmware.
- If at least 9 bytes remain after the battery profile index, read the three
  fields above. Otherwise the firmware is stock Betaflight, or an indiflight
  build without `USE_INDI`: there are no indiflight flags.
- Arming is disabled if the upstream arming-disable flags **or** the indiflight
  arming-disable flags are non-zero. Upstream `ARM_SWITCH` is still bit 29, and it
  is also set when only an indiflight reason blocks arming.

### Active modes (boxes)

The active-box bitmask in MSP_STATUS(_EX) is upstream's variable-length field:
32 bits, then a byte count (low 4 bits), then that many further bytes. It is
indexed by the order of MSP_BOXIDS / MSP_BOXNAMES, so boxes beyond the first
32 (such as INDI, POSITION CONTROL and WAYPOINTS) are already reported. For the
indiflight boxes the bit shows the engaged mode, not just the switch position:
for example, POSITION CONTROL stays off while the EKF has not converged.

Indiflight box permanent ids: INDI 100, POSITION CONTROL 101, WAYPOINTS 102,
RESET HOME 103. 100 and up are kept clear of upstream ids.

## MSP2 ids

`src/main/msp/msp_protocol_v2_indiflight.h` reserves 0x4000..0x4005 (INDI
config/profile, learning config). They have no handlers yet, and neither did
the 4.4 fork. 0x4006..0x4009 are the blackbox log download commands.

## INDI settings

The INDI profile has no fixed MSP payload. The configurator's INDI tab reads
and writes each `indi_*` setting by name with the upstream MSP2_CLI_SETTING
(0x3010) and reads its type, range and default with MSP2_CLI_SETTING_INFO
(0x3011). Both act on the active INDI profile. Only the profile selection has
its own messages (`USE_INDI` builds):

| id | name | payload |
|----|------|---------|
| 0x400A | MSP2_INDI_PROFILE_INDEX | out: u8 active INDI profile, u8 `INDI_PROFILE_COUNT` |
| 0x400B | MSP2_SET_INDI_PROFILE_INDEX | in: u8 profile to activate. Error if armed or out of range |

A stock firmware answers 0x400A with an error, which is how the configurator
decides whether to show the INDI tab.

MSP_EEPROM_WRITE reinitialises the INDI runtime (`initIndiRuntime()`), so
settings written over MSP take effect on save, without a reboot.
