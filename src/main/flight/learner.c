
#include "common/maths.h"
#include "common/axis.h"
//#include "common/filter.h"

#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "flight/att_ctl.h"
#include "flight/catapult.h"

#include <stdbool.h>

#include "learner.h"

learning_query_state_t learningQueryState = LEARNING_QUERY_IDLE;

#ifdef USE_LEARNER

#ifndef USE_CATAPULT
#error "so far, learner has to be used with USE_CATAPULT"
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(learnerConfig_t, learnerConfig, PG_LEARNER_CONFIG, 0);
PG_RESET_TEMPLATE(learnerConfig_t, learnerConfig, 
    .mode = (uint8_t) LEARNING_OFF,
    .numAct = 4,
    .delayMs = 250,
    .stepMs = 50,
    .rampMs = 100,
    .overlapMs = 50,
    .stepAmp = 35,
    .rampAmp = 70,
    .gyroMax = 1600,
);

void initLearningRuntime(void) {
    // for future extension
}

// externs
float outputFromLearningQuery[MAX_SUPPORTED_MOTORS];

#define LEARNING_SAFETY_TIME_MAX ((timeUs_t) 1000000) // 1 sec

void resetLearningQuery(void) {
    if (catapultState == CATAPULT_IDLE)
        learningQueryState = LEARNING_QUERY_IDLE;
}

static void resetMotorQueryState(motor_state_t* motorStates, int motor, bool enable) {
    const learnerConfig_t* c = learnerConfig();
    if ( motor >= c->numAct ) return;

    motor_state_t* p = motorStates + motor;
    if (enable) {
        p->startTime = micros();
        p->queryState = MOTOR_QUERY_STEP;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            p->minGyro[axis] = gyro.gyroADCf[axis]
                + (-c->gyroMax - gyro.gyroADCf[axis]) / (c->numAct - motor);

            p->maxGyro[axis] = gyro.gyroADCf[axis]
                + (+c->gyroMax - gyro.gyroADCf[axis]) / (c->numAct - motor);
        }
    } else {
        p->queryState = MOTOR_QUERY_ZERO;
    } // do nothing if enable, but already enabled
}

void runLearningQueryStateMachine(timeUs_t current) {
    static timeUs_t startAt;
    static timeUs_t safetyTimeoutUs = 0;
    static motor_state_t motorStates[MAX_SUPPORTED_MOTORS];

    for (int i=0; i < MAX_SUPPORTED_MOTORS; i++) 
        outputFromLearningQuery[i] = 0.f;

    const learnerConfig_t* c = learnerConfig();

    if ((c->numAct > MAX_SUPPORTED_MOTORS) || (c->numAct == 0)) {
        learningQueryState = LEARNING_QUERY_IDLE;
        return;
    }

    bool enableConditions = (ARMING_FLAG(ARMED)) 
            && (catapultState == CATAPULT_DONE)
            && (learnerConfig()->mode & LEARN_AFTER_CATAPULT);

doMore:
    switch (learningQueryState) {
        case LEARNING_QUERY_IDLE:
            if (enableConditions) {
                startAt = current + MIN(c->delayMs, 1000)*1e3;
                // 10ms grace period, then cutoff, even if learning is not done, because that means error in the state machine
                safetyTimeoutUs = 10000
                    + 1e3 * c->numAct * (c->stepMs + c->rampMs)
                    - 1e3 * (c->numAct-1) * c->overlapMs;
                safetyTimeoutUs = MIN(safetyTimeoutUs, LEARNING_SAFETY_TIME_MAX);
                safetyTimeoutUs += micros();

                learningQueryState = LEARNING_QUERY_DELAY; goto doMore;
            }
            break;
        case LEARNING_QUERY_DELAY:
            // wait for start. then reset motor query states and transition
            if (cmpTimeUs(current, startAt) > 0) {
                resetMotorQueryState(motorStates, 0, true);
                for (int motor = 1; motor < c->numAct; motor++)
                    resetMotorQueryState(motorStates, motor, false);

                learningQueryState = LEARNING_QUERY_ACTIVE; goto doMore;
            }
            break;
        case LEARNING_QUERY_ACTIVE:
            // cycles through motors
            for (int motor = 0; motor < c->numAct; motor++) {
                if (motorStates[motor].queryState == MOTOR_QUERY_ZERO) { break; } // no more motors possible

                timeDelta_t time_in_query = cmpTimeUs(current, motorStates[motor].startTime);

                // trigger next motor, if overlap time reached
                if ( (motor+1 < c->numAct)
                        && (motorStates[motor+1].queryState == MOTOR_QUERY_ZERO)
                        && ( (time_in_query + 1e3 * c->overlapMs) > 1e3 * (c->stepMs + c->rampMs) ) )
                    resetMotorQueryState(motorStates, motor + 1, true);

                // protect somewhat against gyro overrun, probably wont work because of filter delays
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    float val = gyro.gyroADCf[axis];
                    if ( (val > motorStates[motor].maxGyro[axis]) || (val < motorStates[motor].minGyro[axis]) ) {
                        motorStates[motor].queryState = MOTOR_QUERY_DONE;
                    }
                }

doMoreMotors:
                switch (motorStates[motor].queryState) {
                    case MOTOR_QUERY_ZERO: { break; } // should never happen
                    case MOTOR_QUERY_STEP:
                        if (cmpTimeUs(current, motorStates[motor].startTime + c->stepMs*1e3) > 0) {
                            motorStates[motor].queryState = MOTOR_QUERY_RAMP; goto doMoreMotors;
                        }
                        outputFromLearningQuery[motor] = c->stepAmp * 0.01f;
                        break;
                    case MOTOR_QUERY_RAMP: {
                        timeDelta_t time_in_ramp = cmpTimeUs(current, motorStates[motor].startTime + c->stepMs*1e3);
                        if (time_in_ramp > c->rampMs*1e3) {
                            motorStates[motor].queryState = MOTOR_QUERY_DONE; goto doMoreMotors;
                        }
                        outputFromLearningQuery[motor] = c->rampAmp * 0.01f * ( 1.f - ( ((float) time_in_ramp) / ((float) c->rampMs * 1e3) ) );
                        break;
                    }
                    case MOTOR_QUERY_DONE: { break; }
                }
            }

            bool allMotorsDone = true;
            for (int motor = 0; motor < c->numAct; motor++)
                allMotorsDone &= (motorStates[motor].queryState == MOTOR_QUERY_DONE);

            // safety cutoff even when not fully done
            if (allMotorsDone || (cmpTimeUs(micros(), safetyTimeoutUs) > 0) ) {
                learningQueryState = LEARNING_QUERY_DONE; goto doMore;
            }

            break;
        case LEARNING_QUERY_DONE: { break; }
    }

    for (int motor = 0; motor < c->numAct; motor++) {
        outputFromLearningQuery[motor] = constrainf(outputFromLearningQuery[motor], 0.f, 1.f);
        indiRuntime.d[motor] = outputFromLearningQuery[motor]; // for logging purposes. TODO: also log du
    }
}
#endif
