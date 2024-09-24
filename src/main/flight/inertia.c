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

#define USE_CLI_DEBUG_PRINT

#include "platform.h"
#include "fc/core.h"
#include "drivers/time.h"
#include "flight/throw.h"
#include "cli/cli.h"

#include "pg/pg_ids.h"

#include "inertia.h"


inertiaState_t inertiaState = INERTIA_STATE_DISABLED; 

#ifdef USE_INERTIA_BY_THROWING

// config
PG_REGISTER_WITH_RESET_TEMPLATE(inertiaConfig_t, inertiaConfig, PG_INERTIA_CONFIG, 0);
PG_RESET_TEMPLATE(inertiaConfig_t, inertiaConfig,
    .lowTime      = 200, // ms
    .lowPerc      = 0,   // %
    .highTime     = 200,   // ms
    .highPerc     = 50,     // %
    .lowAgainTime = 200,   // ms
    .lowAgainPerc = 0,   // %
);

float outputFromInertiaCommands[MAX_SUPPORTED_MOTORS] = {0.f};

bool inertiaTryArm(void) {
    if (inertiaState == INERTIA_STATE_DISABLED) {
        inertiaState = INERTIA_STATE_ENABLED;
        return true;
    } else {
        return false;
    }
}

void inertiaDisarm(void) {
    inertiaState = INERTIA_STATE_DISABLED;
    disarm(DISARM_REASON_ARMING_DISABLED);
}

void inertiaUpdateStateMachine(timeUs_t currentTimeUs) {
    for (int i=0; i < MAX_SUPPORTED_MOTORS; i++) 
        outputFromInertiaCommands[i] = 0.f;

    static bool informedUserReady = false;
    static timeUs_t armedAt;
    static timeUs_t stateChangeAt;

doMore:
    switch (inertiaState) {
        case INERTIA_STATE_DISABLED:
            informedUserReady = false;
            break;
        case INERTIA_STATE_ENABLED:
            if (!informedUserReady && throwState == THROW_STATE_WAITING_FOR_THROW) {
                cliPrintLinef("THROW_STATE_WAITING_FOR_THROW");
                informedUserReady = true;
            }
            if (throwState == THROW_STATE_THROWN) {
                cliPrintLinef("Throw Detected, arming.");
                tryArm();
                if (ARMING_FLAG(ARMED)) {
                    inertiaState = INERTIA_STATE_ARMED_LOW;
                    armedAt = currentTimeUs;
                    stateChangeAt = currentTimeUs;
                    goto doMore;
                } else {
                    inertiaState = INERTIA_STATE_DISABLED;
                }
            }
            break;
        case INERTIA_STATE_ARMED_LOW:
            outputFromInertiaCommands[0] = 0.01f * inertiaConfig()->lowPerc;

            if (cmpTimeUs(currentTimeUs, stateChangeAt) > (1000*inertiaConfig()->lowTime)) {
                inertiaState = INERTIA_STATE_ARMED_HIGH;
                stateChangeAt = currentTimeUs;
                goto doMore;
            }
            break;
        case INERTIA_STATE_ARMED_HIGH:
            outputFromInertiaCommands[0] = 0.01f * inertiaConfig()->highPerc;

            if (cmpTimeUs(currentTimeUs, stateChangeAt) > (1000*inertiaConfig()->highTime)) {
                inertiaState = INERTIA_STATE_ARMED_LOW_AGAIN;
                stateChangeAt = currentTimeUs;
                goto doMore;
            }
            break;
        case INERTIA_STATE_ARMED_LOW_AGAIN:
            outputFromInertiaCommands[0] = 0.01f * inertiaConfig()->lowAgainPerc;

            if (cmpTimeUs(currentTimeUs, stateChangeAt) > (1000*inertiaConfig()->lowAgainTime)) {
                disarm(DISARM_REASON_CRASH_PROTECTION);
                inertiaState = INERTIA_STATE_DISABLED;
            }
            break;
    }

    if ((inertiaState >= INERTIA_STATE_ARMED_LOW) && (cmpTimeUs(micros(), armedAt) > INERTIA_SAFETY_TIMEOUT)) {
        disarm(DISARM_REASON_RUNAWAY_TAKEOFF);
        inertiaState = INERTIA_STATE_DISABLED;
    }
}

#endif