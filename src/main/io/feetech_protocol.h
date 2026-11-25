/*
 * Feetech STS serial bus servo protocol: instructions and register map
 * (STS3032 and other servos of the STS series).
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// Frame layout (both directions):
//   [0xFF] [0xFF] [ID] [LEN] [INSTR or ERROR] [PARAMS...] [CHECKSUM]
// LEN = number of params + 2; CHECKSUM = ~(ID + LEN + INSTR/ERROR + PARAMS).
// Multi-byte registers are little-endian (_L at the lower address).
#define FEETECH_HEADER_BYTE             0xFF
#define FEETECH_BROADCAST_ID            0xFE

// Instructions
#define FEETECH_INST_PING               0x01
#define FEETECH_INST_READ               0x02
#define FEETECH_INST_WRITE              0x03
#define FEETECH_INST_REG_WRITE          0x04
#define FEETECH_INST_REG_ACTION         0x05
#define FEETECH_INST_SYNC_WRITE         0x83

// EPROM (read only)
#define FEETECH_REG_VERSION_L           3
#define FEETECH_REG_VERSION_H           4

// EPROM (read/write)
#define FEETECH_REG_ID                  5
#define FEETECH_REG_BAUD_RATE           6
#define FEETECH_REG_MIN_ANGLE_LIMIT_L   9
#define FEETECH_REG_MIN_ANGLE_LIMIT_H   10
#define FEETECH_REG_MAX_ANGLE_LIMIT_L   11
#define FEETECH_REG_MAX_ANGLE_LIMIT_H   12
// Phase / "setting byte". Not written by the driver; set it once per servo,
// e.g. with FD.exe through `serialpassthrough <uart index> 1000000`.
// To reverse a servo, set BIT0 (+1, drive direction) and BIT7 (+128, position
// feedback direction) together. Setting only one of them flips the position
// loop into positive feedback and the servo runs into its end stop.
// Example (tested): 108 (as shipped) -> 237 reverses goal and feedback.
// Other bits are undocumented; leave them as they are.
#define FEETECH_REG_PHASE               18
#define FEETECH_REG_CW_DEAD             26
#define FEETECH_REG_CCW_DEAD            27

// SRAM (read/write)
#define FEETECH_REG_TORQUE_ENABLE       40
#define FEETECH_REG_GOAL_POSITION_L     42
#define FEETECH_REG_GOAL_POSITION_H     43
#define FEETECH_REG_GOAL_TIME_L         44
#define FEETECH_REG_GOAL_TIME_H         45
#define FEETECH_REG_GOAL_SPEED_L        46
#define FEETECH_REG_GOAL_SPEED_H        47
#define FEETECH_REG_LOCK                48

// SRAM (read only)
#define FEETECH_REG_PRESENT_POSITION_L  56
#define FEETECH_REG_PRESENT_POSITION_H  57
#define FEETECH_REG_PRESENT_SPEED_L     58
#define FEETECH_REG_PRESENT_SPEED_H     59
#define FEETECH_REG_PRESENT_LOAD_L      60
#define FEETECH_REG_PRESENT_LOAD_H      61
#define FEETECH_REG_PRESENT_VOLTAGE     62
#define FEETECH_REG_PRESENT_TEMPERATURE 63
#define FEETECH_REG_MOVING              66
#define FEETECH_REG_PRESENT_CURRENT_L   69
#define FEETECH_REG_PRESENT_CURRENT_H   70
