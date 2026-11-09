/*
 * Driver for Feetech STS serial bus servos (e.g. STS3032) on a half-duplex UART.
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

#include <stdint.h>

#include "common/time.h"

// Number of servos on the bus. Servo i has bus ID i + 1, takes its command
// from servo_normalized[i] and writes its position to servo_feedback[i].
#define FEETECH_SERVO_COUNT 2

typedef struct feetechTelemetry_s {
    int16_t position;    // centidegrees, 0 at mid-travel
    int16_t speed;       // steps/s
    int16_t load;        // 0.1 % of max torque
    uint8_t voltage;     // 0.1 V
    uint8_t temperature; // deg C
    uint8_t error;       // status byte of the last reply, 0 = ok
} feetechTelemetry_t;

extern feetechTelemetry_t feetechTelemetry[FEETECH_SERVO_COUNT];

void initServosFeetech(void);
void configureServosFeetechPort(void);
void freeServosFeetechPort(void);

// Call every inner loop. Sending is internally time-sliced.
void handleServosFeetech(timeUs_t currentTimeUs);
void sendServosFeetech(timeUs_t currentTimeUs);
