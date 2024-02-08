
#include "platform.h"

// throwing mode
#if defined(USE_THROW_TO_ARM)
#ifndef USE_ACC
#error "Can only use USE_THROW_TO_ARM with USE_ACC"
#endif

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"
#include "io/beeper.h"
#include "io/external_pos.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "pg/pg_ids.h"

#include "throw.h"

// config
PG_REGISTER_WITH_RESET_TEMPLATE(throwConfig_t, throwConfig, PG_THROW_CONFIG, 0);
PG_RESET_TEMPLATE(throwConfig_t, throwConfig,
    .accHighThresh = 20,
    .accClipThresh = 45,     // m/s/s
    .accLowAgainThresh = 20, // m/s/s
    .gyroHighThresh = 600,   // deg/s
    .momentumThresh = 400,   // cm/s
    .releaseDelayMs = 250   // ms
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

throwState_t throwState = THROW_STATE_DISABLED;
float momentumAtLeavingHand = 0.f;

#ifdef THROW_TO_ARM_USE_FALL_LOGIC
fallState_t fallState = FALL_STATE_DISABLED
#endif


// implementations
float totalAccSq(void) {
    return sq(GRAVITYf) * sq(acc.dev.acc_1G_rec) * 
        ( sq(acc.accADC[X]) + sq(acc.accADC[Y]) + sq(acc.accADC[Z]) );
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
    bool disableConditions = ARMING_FLAG(ARMED)
        || !FLIGHT_MODE(POSITION_MODE)
        || (getArmingDisableFlags() & doNotTolerateDuringThrow) // any critical arming inhibitor?
        || !IS_RC_MODE_ACTIVE(BOXTHROWTOARM) || !IS_RC_MODE_ACTIVE(BOXARM) || IS_RC_MODE_ACTIVE(BOXPARALYZE); // any critical RC setting (may be redundant)

    if (disableConditions && (throwState >= THROW_STATE_READY)) {
        throwState = THROW_STATE_DISABLED;
        beeper(BEEPER_SILENCE);
    }

    // throwing state machine
    bool enableConditions;
    timeDelta_t timeSinceRelease;
    switch(throwState) {
        case THROW_STATE_DISABLED:
            // enable if we dont disable, have accel, and no other disables than angle, arm and prearm 
            enableConditions = 
                !disableConditions
                && acc.isAccelUpdatedAtLeastOnce
                #ifdef USE_GPS_PI
                    && (extPosState >= EXT_POS_STILL_VALID)
                    && (posSetpointState >= EXT_POS_STILL_VALID)
                #endif
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ANGLE | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_NOPREARM));

            if (enableConditions && timingValid) { 
                throwState = THROW_STATE_READY;
            }
            break;
        case THROW_STATE_READY:
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
                throwState = THROW_STATE_READY;
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
            }
            break;
        case THROW_STATE_THROWN:
            break;
    }

    if (throwState >= THROW_STATE_READY) { beeper(BEEPER_THROW_TO_ARM); }

#ifdef THROW_TO_ARM_USE_FALL_LOGIC
    // falling state machine
    if (throwState == THROW_STATE_DISABLED) { fallState = FALL_STATE_DISABLED; }

    switch(fallState) {
        case FALL_STATE_DISABLED:
            if (throwState >= THROW_STATE_READY) { fallState = FALL_STATE_READY; }
            break;
        case FALL_STATE_READY:
            if (totalAccSq() < sq(FALL_ACC_LOW_THRESH)) {
                accLowSince = currentTimeUs;
                fallState = FALL_STATE_ACC_LOW;
            }
            break;
        case FALL_STATE_ACC_LOW:
            if (totalAccSq() > sq(FALL_ACC_LOW_THRESH)) { fallState = FALL_STATE_READY; }

            timeDelta_t timeAccLow = cmpTimeUs(currentTimeUs, accLowSince);
            if (timeAccLow > (1e3 * FALL_ACC_LOW_TIME_MS)) { fallState = FALL_STATE_FALLING; }
            break;
        case FALL_STATE_FALLING:
            break;
    }
#endif
}

#endif // #if defined(USE_ACC) && defined(USE_THROW_TO_ARM)
