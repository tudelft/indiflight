/*
 * This file is part of Indiflight.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Indiflight is distributed in the hope that they will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Hardware in the loop (HIL_BUILD): the simulator sends IMU, motor rpm and
// servo angles over pi-protocol (HIL_IN) on the serial port with FUNCTION_HIL,
// the firmware sends motor and servo commands back (HIL_OUT).
// (C) tblaha 2023, ported to Betaflight 2026.

#pragma once

#include "platform.h"

#if defined(HIL_BUILD) || defined(MOCKUP)

#include "drivers/accgyro/accgyro.h"
#include "common/sensor_alignment.h"

// raw (FLU) sensor vector from the simulator's FRD vector in the sensor frame;
// the alignment chain then applies sensor/board alignment and FLU -> FRD
void hilRawFromSensorFrd(const float frd[3], float raw[3]);
#endif

#ifdef HIL_BUILD

#include "drivers/dshot.h"

typedef struct hilInput_s {
    float gyro[3];                          // deg/s, body FRD
    float acc[3];                           // g, specific force, body FRD (level: z = -1)
    float rpm[MAX_SUPPORTED_MOTORS];        // rpm
    float servo_angle[MAX_SUPPORTED_SERVOS];// centidegree
} hilInput_t;

extern hilInput_t hilInput;

void initHil(void);
void checkHilState(void);
void handleHil(void);           // TASK_HIL: parse HIL_IN
void hilSendActuators(void);    // actuator apply: send HIL_OUT, fill servo_feedback[]

// Sensor injection at device level, called right after the gyro/acc readFn:
// overwrites the raw counts so that the normal alignment chain (sensor and
// board alignment, FLU -> FRD in sensors/boardalignment.c) yields hilInput.
void hilInjectGyroRaw(gyroDev_t *gyroDev);
void hilInjectAccRaw(accDev_t *accDev);
// replaces dshot_decode_telemetry_value(): eRPM of motor from the simulator
void hilGetDshotTelemetry(uint8_t motorIndex, uint32_t *value, dshotTelemetryType_t *type);

#endif
