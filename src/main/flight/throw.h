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
#include "pg/pg.h"

typedef struct throwConfig_s {
    uint8_t accHighThresh;     // m/s/s
    uint8_t accClipThresh;     // m/s/s
    uint8_t accLowAgainThresh; // m/s/s
    uint16_t gyroHighThresh;   // deg/s
    uint16_t momentumThresh;   // cm/s
    uint16_t releaseDelayMs;   // ms
} throwConfig_t;

PG_DECLARE(throwConfig_t, throwConfig);


float totalAccSq(void);
float totalGyroSq(void);
void updateThrowFallStateMachine(timeUs_t currentTimeUs);

typedef enum {
    THROW_STATE_IDLE = -1,
    THROW_STATE_WAITING_FOR_THROW = 0,
    THROW_STATE_ACC_HIGH,
    THROW_STATE_ENOUGH_MOMENTUM,
    THROW_STATE_LEFT_HAND,
    THROW_STATE_THROWN,
    THROW_STATE_ARMED_AFTER_THROW,
} throwState_t;

#define FALL_ACC_LOW_THRESH 3.f
#define FALL_ACC_LOW_TIME_MS 400

typedef enum {
    FALL_STATE_IDLE = -1,
    FALL_STATE_READY = 0,
    FALL_STATE_ACC_LOW,
    FALL_STATE_FALLING
} fallState_t;

extern fallState_t fallState;
extern throwState_t throwState;
extern float momentumAtLeavingHand;
extern armingDisableFlags_e doNotTolerateDuringThrow;
