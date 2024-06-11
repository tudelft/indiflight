
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
    uint8_t actLimit;
    uint8_t applyIndiProfileAfterQuery;
    uint8_t applyPositionProfileAfterQuery;
    uint8_t applyHoverRotationAfterQuery;
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
    fp_vector_t imuRate;
    fp_vector_t imuRateDot;
    fp_vector_t imuSpf;
    float fxOmega[MAX_SUPPORTED_MOTORS];
    float fxOmegaDiff[MAX_SUPPORTED_MOTORS];
    float fxOmegaDotDiff[MAX_SUPPORTED_MOTORS];
    fp_vector_t fxRateDotDiff;
    fp_vector_t fxSpfDiff;
    float motorOmega[MAX_SUPPORTED_MOTORS];
    float motorOmegaDot[MAX_SUPPORTED_MOTORS];
    float motorD[MAX_SUPPORTED_MOTORS];
    float motorSqrtD[MAX_SUPPORTED_MOTORS];
    float zeta[LEARNER_LOOP_COUNT];
    float gains[LEARNER_LOOP_COUNT];
    bool applyIndiProfileAfterQuery;
    bool applyPositionProfileAfterQuery;
    bool applyHoverRotationAfterQuery;
} learnerRuntime_t;

extern learnerRuntime_t learnRun;

extern rls_t motorRls[MAX_SUPPORTED_MOTORS];
extern rls_t imuRls;
//extern rls_parallel_t fxSpfRls;
//extern rls_parallel_t fxRateDotRls;
extern rls_t fxRls[6];
extern fp_quaternion_t hoverAttitude;

void initLearnerRuntime(void);


// --- states and functions

// learning stuff
#define LEARNER_TIMINGS_NUM 7
typedef struct learnerTimings_s {
    timeUs_t start;
    timeDelta_t filters;
    timeDelta_t imu;
    timeDelta_t fx;
    timeDelta_t motor;
    timeDelta_t gains;
    timeDelta_t updating;
    timeDelta_t hover;
} learnerTimings_t;

extern learnerTimings_t learnerTimings;

void initLearner(void);
void testLearner(void);
void updateLearner(timeUs_t current);
void updateLearnedParameters(indiProfile_t* indi, positionProfile_t* pos);

// query stuff
typedef enum query_state_e {
    LEARNING_QUERY_IDLE = -1,
    LEARNING_QUERY_WAITING_FOR_LAUNCH = 0,
    LEARNING_QUERY_DELAY,
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

#define LEARNER_NULLEX_STEP_AMP 0.3f
#define LEARNER_NULLEX_STEP_TIME 100000
#define LEARNER_NULLEX_ARREST_TIME 200000
#define LEARNER_NULLEX_SAFETY_TIMEOUT 400000

typedef enum nullex_state_e {
    LEARNER_NULLEX_STATE_IDLE = -1,
    LEARNER_NULLEX_STATE_ARREST = 0,
    LEARNER_NULLEX_STATE_ACTIVE,
    LEARNER_NULLEX_STATE_DONE
} nullex_state_t;

extern nullex_state_t nullexState;
extern float uDeltaFromNullex[MAX_SUPPORTED_MOTORS];
extern fp_vector_t spfBodyDeltaFromNullex;

void runRotNullspaceExcitation(timeUs_t current);
