/*
 * Learn UAV dynamics from sequence of input commands and synthesize controller
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

#include "common/time.h"
#include "common/rls.h"
#include "config/config.h"

#include "flight/indi.h"
#include "flight/pos_ctl.h"

// --- config
typedef struct learnerConfig_s {
    uint8_t modeProbing;
    uint8_t modeFx;
    uint8_t modeAct;
    uint8_t modeHover;
    uint16_t mixControlAfterMs;
    uint8_t initFromProfileFx;
    uint8_t initFromProfileAct;
    uint16_t actMask;
    uint8_t imuFiltHz;
    uint8_t fxFiltHz;
    uint8_t motorFiltHz;
    uint8_t servoFiltHz;
    uint8_t zetaRate;
    uint8_t zetaAttitude;
    uint8_t zetaVelocity;
    uint8_t zetaPosition;
    int16_t rollMisalignment;
    int16_t pitchMisalignment;
    int16_t yawMisalignment;
    uint8_t randomizeMisalignment;
    uint8_t applyIndi;
    uint8_t applyPosition;
    uint8_t applyHoverRotation;
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
    LEARNING_OFF                    = 0,
    LEARN_DURING_FLIGHT             = 1 << 0,
    LEARN_DURING_PROBING            = 1 << 1,
} learner_mode_t;

typedef enum learning_probing_mode_e {
    LEARN_PROBING_OFF               = 0,
    LEARN_PROBING_DURING_FLIGHT     = 1 << 0, // allow query during flight
    LEARN_PROBING_AFTER_THROW       = 1 << 1, // after throw
    LEARN_PROBING_AFTER_CATAPULT    = 1 << 2, // after catapult
} laerner_probing_mode_t;

typedef struct learningRuntime_s {
    bool initialized;
    bool filtersInitialized;
    bool mixControl;
    int numActuators; // number of actuators (motors + servos)
    int numMotors; // number of motors
    int numServos; // number of servos
    fp_vector_t imuRate;
    fp_vector_t imuRateDot;
    fp_vector_t imuSpf;
    float fxOmega[MAXU];
    float fxOmegaDiff[MAXU];
    float fxOmegaDotDiff[MAXU];
    fp_vector_t fxRateDotDiff;
    fp_vector_t fxSpfDiff;
    float motorOmega[MAXU];
    float motorOmegaDot[MAXU];
    float motorD[MAXU];
    float motorSqrtD[MAXU];
    float zeta[LEARNER_LOOP_COUNT];
    float gains[LEARNER_LOOP_COUNT];
} learnerRuntime_t;

extern learnerRuntime_t learnRun;

extern rls_t actRls[MAXU];
extern rls_t imuRls;
//extern rls_parallel_t fxSpfRls;
//extern rls_parallel_t fxRateDotRls;
extern rls_t fxRls[6];
extern fp_quaternion_t hoverAttitude;


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

void initLearnerFilters(void);
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
