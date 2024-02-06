
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
    uint16_t rotationTimeMs;  // in ms
    uint8_t upwardsAccel;     // in m/s/s
} catapultConfig_t;

PG_DECLARE(catapultConfig_t, catapultConfig);

typedef struct catapultRuntime_s {
    float altitude; // m
    float xyNed[2]; // m
    t_fp_vector rotationRate; // rad/s
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
extern t_fp_vector spfSpBodyFromCat;
extern t_fp_vector rateSpBodyFromCat;
extern bool controlAttitudeFromCat;

void runCatapultStateMachine(timeUs_t current);
void resetCatapult(void);
