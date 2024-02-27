
#pragma once

#include "common/time.h"
#include "common/rls.h"
#include "config/config.h"

#include "flight/indi.h"
#include "flight/pos_ctl.h"

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
    uint8_t imuFiltHz;
    uint8_t fxFiltHz;
    uint8_t motorFiltHz;
    uint8_t zetaRate;
    uint8_t zetaAttitude;
    uint8_t zetaVelocity;
    uint8_t zetaPosition;
    uint8_t applyIndiProfileAfterQuery;
    uint8_t applyPositionProfileAfterQuery;
} learnerConfig_t;

PG_DECLARE(learnerConfig_t, learnerConfig);

typedef enum learner_loops_e {
    LEARNER_LOOP_RATE = 0,
    LEARNER_LOOP_ATTITUDE,
    LEARNER_LOOP_VELOCITY,
    LEARNER_LOOP_POSITION,
    LEARNER_LOOP_COUNT
} learner_loops_t;

typedef enum learning_mode_e {
    LEARNING_OFF         = 0,
    LEARN_DURING_FLIGHT  = 1 << 0,
    LEARN_AFTER_CATAPULT = 1 << 1,
    LEARN_AFTER_THROW    = 1 << 2,
} learner_mode_t;

typedef struct learningRuntime_s {
    t_fp_vector imuRate;
    t_fp_vector imuRateDot;
    t_fp_vector imuSpf;
    float fxOmega[MAX_SUPPORTED_MOTORS];
    float fxOmegaDiff[MAX_SUPPORTED_MOTORS];
    float fxOmegaDotDiff[MAX_SUPPORTED_MOTORS];
    t_fp_vector fxRateDotDiff;
    t_fp_vector fxSpfDiff;
    float motorOmega[MAX_SUPPORTED_MOTORS];
    float motorOmegaDot[MAX_SUPPORTED_MOTORS];
    float motorD[MAX_SUPPORTED_MOTORS];
    float motorSqrtD[MAX_SUPPORTED_MOTORS];
    float zeta[LEARNER_LOOP_COUNT];
    float gains[LEARNER_LOOP_COUNT];
    bool applyIndiProfileAfterQuery;
    bool applyPositionProfileAfterQuery;
} learnerRuntime_t;

extern learnerRuntime_t learnRun;

extern rls_parallel_t motorRls[MAX_SUPPORTED_MOTORS];
extern rls_t imuRls;
extern rls_parallel_t fxSpfRls;
extern rls_parallel_t fxRateDotRls;

void initLearnerRuntime(void);
float dumbRng(void);


// --- states and functions

// learning stuff
void initLearner(void);
void testLearner(void);
void updateLearner(timeUs_t current);
void updateGains(void);
void updateLearnedParameters(indiProfile_t* indi, positionProfile_t* pos);

// query stuff
typedef enum query_state_e {
    LEARNING_QUERY_IDLE = -1,
    LEARNING_QUERY_WAITING_FOR_LAUNCH = 0,
    LEARNING_QUERY_DELAY,
    LEARNING_QUERY_ACTIVE,
    LEARNING_QUERY_APPLYING,
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
