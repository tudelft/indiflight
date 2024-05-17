
#include "pg/pg_ids.h"
#include "io/beeper.h"
#include "io/external_pos.h"
#include "flight/imu.h"
#include "flight/indi.h"
#include "fc/runtime_config.h"
#include "common/rng.h"

#include "catapult.h"

catapult_state_t catapultState = CATAPULT_IDLE;

#ifdef USE_CATAPULT
#pragma message "You are compiling with dangerous code!"

#ifndef USE_INDI
#error "must use catapult with USE_INDI"
#endif

#ifndef USE_POS_CTL
#error "muse use catapult with USE_POS_CTL"
#endif

// extern
fp_quaternion_t attSpNedFromCat = { 1.f, 0.f, 0.f, 0.f };
fp_vector_t spfSpBodyFromCat = { .A = { 0.f, 0.f, 0.f } };
fp_vector_t rateSpBodyFromCat = { .A = { 0.f, 0.f, 0.f } };
bool controlAttitudeFromCat = false;

void resetCatapult(void) {
    if (!ARMING_FLAG(ARMED)) {
        beeper(BEEPER_SILENCE);
        catapultState = CATAPULT_IDLE;
    }
}

PG_REGISTER_WITH_RESET_TEMPLATE(catapultConfig_t, catapultConfig, PG_CATAPULT_CONFIG, 0);
PG_RESET_TEMPLATE(catapultConfig_t, catapultConfig, 
    .altitude = 300,
    .xNed = 0,
    .yNed = 0,
    .rotationRoll = 400,
    .rotationPitch = 400,
    .rotationYaw = 400,
    .randomizeRotation = 1,
    .rotationTimeMs = 150,
    .upwardsAccel = 20,
);

#include "flight/learner.h"

catapultRuntime_t catapultRuntime;
void initCatapultRuntime(void) {
    catapultRuntime.altitude = constrainu(catapultConfig()->altitude, 1, 1000) * 0.01f;
    catapultRuntime.xyNed[0] = catapultConfig()->xNed * 0.01f;
    catapultRuntime.xyNed[1] = catapultConfig()->yNed * 0.01f;

    if (catapultConfig()->randomizeRotation) {
        catapultRuntime.rotationRate.V.X = DEGREES_TO_RADIANS(ABS(catapultConfig()->rotationRoll)) * rngFloat();
        catapultRuntime.rotationRate.V.Y = DEGREES_TO_RADIANS(ABS(catapultConfig()->rotationPitch)) * rngFloat();
        catapultRuntime.rotationRate.V.Z = DEGREES_TO_RADIANS(ABS(catapultConfig()->rotationYaw)) * rngFloat();
    } else {
        catapultRuntime.rotationRate.V.X = DEGREES_TO_RADIANS(catapultConfig()->rotationRoll);
        catapultRuntime.rotationRate.V.Y = DEGREES_TO_RADIANS(catapultConfig()->rotationPitch);
        catapultRuntime.rotationRate.V.Z = DEGREES_TO_RADIANS(catapultConfig()->rotationYaw);
    }

    catapultRuntime.rotationTimeUs = constrainu(catapultConfig()->rotationTimeMs, 0, 1000) * 1e3;
    catapultRuntime.fireTimeUs = 0;
    catapultRuntime.upwardsAccel = (float) constrainu(catapultConfig()->upwardsAccel, 4, 100); // real thrust setting can be up to 17% higher if higher than 20m/s/s
    catapultRuntime.attitude.w = 1.f;
    catapultRuntime.attitude.x = 0.f;
    catapultRuntime.attitude.y = 0.f;
    catapultRuntime.attitude.z = 0.f;
    catapultRuntime.spfSpBodyZ = 1.f; // 1m/s/s downwards
}

static bool calculateCatapult(void) {
#define IGf (1.f / GRAVITYf)

    initCatapultRuntime(); // to ensure variables are initialized

    float h = catapultRuntime.altitude; // always greater 0 m
    float a = catapultRuntime.upwardsAccel; // always greater 4 m/s/s

    // get axis to rotate thrust vector around
    fp_vector_t rotationAxis = { .A = {
        catapultRuntime.xyNed[1] - posEstNed.V.Y,
        - (catapultRuntime.xyNed[0] - posEstNed.V.X),
        0.f,
    }};

    // get lateral distance to be covered
    float distance = hypotf(rotationAxis.V.X, rotationAxis.V.Y);

    // limit distance to achieve approx 40deg launch angle from vertical and abort if too far
    distance = constrainf(distance, 0.f, 2.f * catapultRuntime.altitude);
    if (distance > CATAPULT_MAX_HORIZONTAL_DISTANCE_CM) {
        return false;
    }

    // get firetime to reach vertical height
    float fireTimeSec = sqrtf( (2.f * h) / (a * (a * IGf + 1.f)) );
    catapultRuntime.fireTimeUs = 1e6f * fireTimeSec; // always below 2.8sec

    // get launch angle to get enough lateral velocity to cover distance
    float vVertMax = a * fireTimeSec;
    float freefallTimeSec = vVertMax * IGf + sqrtf(2.f * h * IGf);
    float phi = atanf( distance / (0.5f*a*sq(fireTimeSec) + a*fireTimeSec*freefallTimeSec) );

    // normalize axis and then calculate quaternion setpoint
    if (distance > 0.01f) {
        rotationAxis.V.X /= distance;
        rotationAxis.V.Y /= distance;
        quaternion_of_axis_angle(&catapultRuntime.attitude, &rotationAxis, phi);
    }
    // else {
    //      quaternion remains "up" from initcatapultruntime
    //}

    // correct thrust
    catapultRuntime.spfSpBodyZ = -(catapultRuntime.upwardsAccel + GRAVITYf) / cosf(phi);

    return true;
}

void runCatapultStateMachine(timeUs_t current) {
#define CATAPULT_DELAY_TIME 1000000 // 1sec
#define CATAPULT_SAFETY_TIME_LIMIT 750000 // 0.75sec, should be set to 3sec at some point, that's the max that calculateCatapult should output
    static timeUs_t launchTime = 0;
    static timeUs_t cutoffTime = 0;

    bool disableConditions = !FLIGHT_MODE(POSITION_MODE)
                || !FLIGHT_MODE(CATAPULT_MODE)
                || (extPosState == EXT_POS_NO_SIGNAL)
                || (posSetpointState == EXT_POS_NO_SIGNAL)
#ifdef USE_INDI
                || (systemConfig()->indiProfileIndex == (INDI_PROFILE_COUNT-1)) // cannot guarantee safe launch here
#endif
#if defined(USE_EKF) && false
                || !ekf_is_healthy() // TODO: implement this!
#endif
                ;
    bool enableConditions = !disableConditions && !ARMING_FLAG(ARMED);

    attSpNedFromCat.w = 1.;
    attSpNedFromCat.x = 0.;
    attSpNedFromCat.y = 0.;
    attSpNedFromCat.z = 0.;

    spfSpBodyFromCat.V.X = 0.f;
    spfSpBodyFromCat.V.Y = 0.f;
    spfSpBodyFromCat.V.Z = 0.f;

    rateSpBodyFromCat.V.X = 0.f;
    rateSpBodyFromCat.V.Y = 0.f;
    rateSpBodyFromCat.V.Z = 0.f;

    controlAttitudeFromCat = true;

doMore:
    switch (catapultState) {
        case CATAPULT_IDLE:
            launchTime = 0;
            cutoffTime = 0;

            if (enableConditions && calculateCatapult())
                catapultState = CATAPULT_WAITING_FOR_ARM;

            break;
        case CATAPULT_WAITING_FOR_ARM:
            beeper(BEEPER_THROW_TO_ARM);
            if (ARMING_FLAG(ARMED)) {
                launchTime = current + CATAPULT_DELAY_TIME;
                catapultState = CATAPULT_DELAY; goto doMore;
            }
            if (disableConditions)
                catapultState = CATAPULT_IDLE;
            break;
        case CATAPULT_DELAY:
            if (cmpTimeUs(current, launchTime) > 0) {
                cutoffTime = current + catapultRuntime.fireTimeUs;
                catapultState = CATAPULT_LAUNCHING; goto doMore;
            }
            break;
        case CATAPULT_LAUNCHING:
            if (cmpTimeUs(current, cutoffTime) <= 0) {
                spfSpBodyFromCat.V.Z = catapultRuntime.spfSpBodyZ;
                attSpNedFromCat = catapultRuntime.attitude;
            } else { 
                catapultState = CATAPULT_ROTATING; goto doMore;
            }
            break;
        case CATAPULT_ROTATING:
            if ( cmpTimeUs(current, cutoffTime) <= catapultRuntime.rotationTimeUs ) {
                controlAttitudeFromCat = false;
                rateSpBodyFromCat = catapultRuntime.rotationRate;
            } else {
                beeper(BEEPER_SILENCE);
                catapultState = CATAPULT_DONE;
            }
            break;
        case CATAPULT_DONE:
            if (!ARMING_FLAG(ARMED) && !FLIGHT_MODE(CATAPULT_MODE))
                catapultState = CATAPULT_IDLE;
            break;
    }
    if ((catapultState > CATAPULT_WAITING_FOR_ARM) 
            && (cmpTimeUs(current, launchTime) > CATAPULT_SAFETY_TIME_LIMIT)) {
        catapultState = CATAPULT_DONE;
    }
}

#endif // USE_CATAPULT

