
#pragma once

#include "common/time.h"
#include "config/config.h"

// --- config
typedef struct learnerConfig_s {
    uint8_t mode;
    uint8_t numAct;
    uint16_t delayMs;
    uint16_t stepMs;
    uint16_t rampMs;
    uint16_t overlapMs;
    uint8_t stepAmp;
    uint8_t rampAmp;
    uint16_t gyroMax;
} learnerConfig_t;

PG_DECLARE(learnerConfig_t, learnerConfig);

typedef enum learning_mode_e {
    LEARNING_OFF = 0,
    LEARN_DURING_FLIGHT = 1 << 0,
    LEARN_AFTER_CATAPULT = 1 << 1,
} learning_mode_t;

void initLearningRuntime(void);


// --- states and functions
typedef enum query_state_e {
    LEARNING_QUERY_IDLE = -1,
    LEARNING_QUERY_DELAY = 0,
    LEARNING_QUERY_ACTIVE,
    LEARNING_QUERY_DONE
} learning_query_state_t;

extern learning_query_state_t learningQueryState;
extern float outputFromLearningQuery[MAX_SUPPORTED_MOTORS];

typedef enum motor_query_state_e {
    MOTOR_QUERY_ZERO = 0,
    MOTOR_QUERY_STEP,
    MOTOR_QUERY_RAMP,
    MOTOR_QUERY_DONE
} motor_query_state_t;

typedef struct motor_state_s {
    timeUs_t startTime;
    float minGyro[3];
    float maxGyro[3];
    motor_query_state_t queryState;
} motor_state_t;

void runLearningQueryStateMachine(timeUs_t current);
void resetLearningQuery(void);
