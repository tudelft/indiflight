/*
 * Flight mode that launches a multicopter to a specified height and direction
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

#include "common/maths.h"
#include <math.h>

#include "config/config.h"

#define CATAPULT_MAX_HORIZONTAL_DISTANCE_CM 600
#define CATAPULT_MAX_FIRETIME_US 1000000

typedef struct catapultConfig_s {
    uint16_t altitude;        // in cm
    int16_t xNed;             // in cm
    int16_t yNed;             // in cm
    int16_t rotationRoll;    // in deg/s
    int16_t rotationPitch;   // in deg/s
    int16_t rotationYaw;     // in deg/s
    uint8_t randomizeRotation; // bool
    uint16_t rotationTimeMs;  // in ms
    uint8_t upwardsAccel;     // in m/s/s
} catapultConfig_t;

PG_DECLARE(catapultConfig_t, catapultConfig);

typedef struct catapultRuntime_s {
    float altitude; // m
    float xyNed[2]; // m
    fp_vector_t rotationRate; // rad/s
    timeDelta_t rotationTimeUs;
    timeDelta_t fireTimeUs;
    float upwardsAccel; // m/s/s
    fp_quaternion_t attitude; // quat
    float spfSpBodyZ;
} catapultRuntime_t;

extern catapultRuntime_t catapultRuntime;
void initCatapultRuntime(void);

typedef enum catapult_state_s {
    CATAPULT_IDLE = -1,
    CATAPULT_WAITING_FOR_ARM = 0,
    CATAPULT_DELAY,
    CATAPULT_LAUNCHING,
    CATAPULT_ROTATING,
    CATAPULT_DONE,
} catapult_state_t;

extern catapult_state_t catapultState;

extern fp_quaternion_t attSpNedFromCat;
extern fp_vector_t spfSpBodyFromCat;
extern fp_vector_t rateSpBodyFromCat;
extern bool controlAttitudeFromCat;

void runCatapultStateMachine(timeUs_t current);
void resetCatapult(void);
