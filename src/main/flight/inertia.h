/*
 * State machine to detect throwing using the IMU. Intended to arm the craft.
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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

#include "fc/runtime_config.h"
#include "platform.h"
#include "pg/pg.h"
#include <stdbool.h>
#include "common/time.h"

typedef struct inertiaConfig_s {
    uint16_t lowTime;     // ms
    uint8_t  lowPerc;   // %
    uint16_t highTime;       // ms
    uint8_t  highPerc;   // %
    uint16_t lowAgainTime;       // ms
    uint8_t  lowAgainPerc;   // %
} inertiaConfig_t;

PG_DECLARE(inertiaConfig_t, inertiaConfig);

typedef enum {
    INERTIA_STATE_DISABLED = -1,
    INERTIA_STATE_ENABLED = 0,
    INERTIA_STATE_ARMED_LOW,
    INERTIA_STATE_ARMED_HIGH,
    INERTIA_STATE_ARMED_LOW_AGAIN,
} inertiaState_t;

#define INERTIA_SAFETY_TIMEOUT 2000000  // 2 seconds

extern float outputFromInertiaCommands[MAX_SUPPORTED_MOTORS];
extern inertiaState_t inertiaState;
bool inertiaTryArm(void);
void inertiaDisarm(void);
void inertiaUpdateStateMachine(timeUs_t currentTimeUs);
