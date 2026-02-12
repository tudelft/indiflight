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

// TODO: check if learner is ready, if learner is enabled


#include "platform.h"

#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "io/beeper.h"
#include "io/local_pos.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "flight/indi.h"
#include "flight/ekf.h"
#include "flight/pos_ctl.h"

#include "pg/pg_ids.h"

#include "throw.h"

throwState_t throwState = THROW_STATE_IDLE;

// throwing mode
#if defined(USE_THROW_TO_ARM)
#ifndef USE_ACC
#error "Can only use USE_THROW_TO_ARM with USE_ACC"
#endif

#pragma message "You are compiling with dangerous code!"


// config
PG_REGISTER_WITH_RESET_TEMPLATE(throwConfig_t, throwConfig, PG_THROW_CONFIG, 1);
PG_RESET_TEMPLATE(throwConfig_t, throwConfig,
    .accHighThresh = 20,
    .accClipThresh = 45,     // m/s/s
    .accLowAgainThresh = 20, // m/s/s
    .gyroHighThresh = 600,   // deg/s
    .momentumThresh = 400,   // cm/s
    .releaseDelayMs = 250,   // ms
    .idleBeforeThrow = 0,    // boolean
    .allowManualModes = 0    // boolean
);

// ---externs

// If some of these flags are set, then we cannot arm from throw. Disable 
// throwing mode, which also disables the beeper so we notice.
// If the pilot has already started throwing, then too bad.
armingDisableFlags_e doNotTolerateDuringThrow = (
    ARMING_DISABLED_NO_GYRO 
    | ARMING_DISABLED_FAILSAFE
    | ARMING_DISABLED_RX_FAILSAFE
    | ARMING_DISABLED_BAD_RX_RECOVERY
    | ARMING_DISABLED_BOXFAILSAFE
    | ARMING_DISABLED_PARALYZE
);

float momentumAtLeavingHand = 0.f;

#ifdef THROW_TO_ARM_USE_FALL_LOGIC
fallState_t fallState = FALL_STATE_IDLE
#endif


// implementations
float totalAccSq(void) {
    return sq(GRAVITYf) * sq(acc.dev.acc_1G_rec) * 
        ( sq(acc.accADCf[X]) + sq(acc.accADCf[Y]) + sq(acc.accADCf[Z]) );
}

float totalGyroSq(void) {
    return sq(gyro.gyroADCf[X]) + sq(gyro.gyroADCf[Y]) + sq(gyro.gyroADCf[Z]);
}

void updateThrowFallStateMachine(timeUs_t currentTimeUs) {
    static float momentum = 0;
    static timeUs_t leftHandSince = 0;

    // handle timings
    static timeUs_t lastCall = 0;
    timeDelta_t delta = cmpTimeUs(currentTimeUs, lastCall);
    bool timingValid = ((delta < 100000) && (delta > 0) && (lastCall > 0));
    lastCall = currentTimeUs;

    // disable state machines (and possibly abort throw/fall if in progress)
    bool disableConditions = false
        || FLIGHT_MODE(CATAPULT_MODE | NN_MODE) // downright dangerous to accidentally throw with catapult?
        || (getArmingDisableFlags() & doNotTolerateDuringThrow) // any critical arming inhibitor?
        || !IS_RC_MODE_ACTIVE(BOXTHROWTOARM) || !IS_RC_MODE_ACTIVE(BOXARM) || IS_RC_MODE_ACTIVE(BOXPARALYZE); // any critical RC setting (may be redundant)

    if (disableConditions && (throwState >= THROW_STATE_WAITING_FOR_THROW) && (throwState < THROW_STATE_THROWN)) {
        // abort if in progress
        throwState = THROW_STATE_IDLE;
        beeper(BEEPER_SILENCE);
    }

    // throwing state machine
    bool enableConditions;
    bool autoMode = FLIGHT_MODE(POSITION_MODE | VELOCITY_MODE); // NN/catapult disabled already
    timeDelta_t timeSinceRelease;
    switch(throwState) {
        case THROW_STATE_IDLE:
            // enable if we dont disable, have accel, and no disable flags than angle, arm and prearm 
            enableConditions = 
                !disableConditions
                && acc.isAccelUpdatedAtLeastOnce
                && isTouchingGround()
#ifdef USE_LOCAL_POSITION
                && ( (!autoMode && throwConfig()->allowManualModes)
                        || (autoMode && isConvergedEkf()) )
#else
                && !autoMode
                && throwConfig()->allowManualModes
#endif
                ;

           if (enableConditions && timingValid) { 
                throwState = THROW_STATE_WAITING_FOR_THROW;
            }
            break;
        case THROW_STATE_WAITING_FOR_THROW:
            if (totalAccSq() > sq((float)throwConfig()->accHighThresh)) {
                throwState = THROW_STATE_ACC_HIGH;
            }
            momentum = 0.f;
            break;
        case THROW_STATE_ACC_HIGH: {
            float addedMomentumMS = constrainf( sqrtf(totalAccSq()), 0., (float) throwConfig()->accClipThresh );
            addedMomentumMS -= GRAVITYf;
            momentum += addedMomentumMS * 1e-6f * ((float) delta);

            if (momentum > (throwConfig()->momentumThresh * 0.01f)) {
                throwState = THROW_STATE_ENOUGH_MOMENTUM;
            } else if (totalAccSq() < sq((float)throwConfig()->accHighThresh)) {
                throwState = THROW_STATE_WAITING_FOR_THROW;
            }
            break;
        }
        case THROW_STATE_ENOUGH_MOMENTUM: { // will definitely arm, just waiting for release
            float addedMomentumMS = constrainf( sqrtf(totalAccSq()), 0., (float) throwConfig()->accClipThresh );
            addedMomentumMS -= GRAVITYf;
            momentum += addedMomentumMS * 1e-6f * ((float) delta);

            if ( (totalAccSq() < sq((float)throwConfig()->accLowAgainThresh))
                    || (totalGyroSq() > sq((float)throwConfig()->gyroHighThresh)) ) {
                momentumAtLeavingHand = momentum;
                leftHandSince = currentTimeUs;
                throwState = THROW_STATE_LEFT_HAND;
            }
            break;
        }
        case THROW_STATE_LEFT_HAND:
            timeSinceRelease = cmpTimeUs(currentTimeUs, leftHandSince);
            if (timeSinceRelease > (1e3 * throwConfig()->releaseDelayMs)) {
                throwState = THROW_STATE_THROWN;
#ifdef USE_LOCAL_POSITION
                if (autoMode && isConvergedEkf()) {
                    if (posSpNed.valid) {
                        posArrestZMotionOnly();
                    } else {
                        posArrestMotion();
                    }
                } 
#endif
                beeper(BEEPER_SILENCE);
            }
            break;
        case THROW_STATE_THROWN:
            if (ARMING_FLAG(ARMED)) {
                throwState = THROW_STATE_ARMED_AFTER_THROW;
            } else if (cmpTimeUs(currentTimeUs, leftHandSince) > 3e6) {
                // 3 seconds timeout
                throwState = THROW_STATE_IDLE;
            }
            break;
        case THROW_STATE_ARMED_AFTER_THROW:
            if (!ARMING_FLAG(ARMED))
                throwState = THROW_STATE_IDLE;
            break;
    }

    if (throwState == THROW_STATE_WAITING_FOR_THROW) { beeper(BEEPER_THROW_TO_ARM); }
}

#endif // #if defined(USE_ACC) && defined(USE_THROW_TO_ARM)
