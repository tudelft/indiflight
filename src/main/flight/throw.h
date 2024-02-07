
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
    THROW_STATE_DISABLED = -1,
    THROW_STATE_READY = 0,
    THROW_STATE_ACC_HIGH,
    THROW_STATE_ENOUGH_MOMENTUM,
    THROW_STATE_LEFT_HAND,
    THROW_STATE_THROWN,
} throwState_t;

#define FALL_ACC_LOW_THRESH 3.f
#define FALL_ACC_LOW_TIME_MS 400

typedef enum {
    FALL_STATE_DISABLED = -1,
    FALL_STATE_READY = 0,
    FALL_STATE_ACC_LOW,
    FALL_STATE_FALLING
} fallState_t;

extern fallState_t fallState;
extern throwState_t throwState;
extern float momentumAtLeavingHand;
extern armingDisableFlags_e doNotTolerateDuringThrow;
