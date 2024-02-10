
#include "common/maths.h"
#include "common/rls.h"
#include "common/axis.h"
#include "common/filter.h"

#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "flight/indi.h"
#include "flight/catapult.h"

#include <stdbool.h>

#include "learner.h"

learning_query_state_t learningQueryState = LEARNING_QUERY_IDLE;

#ifdef USE_LEARNER

#ifndef USE_INDI
#error "must use learner with USE_INDI"
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
    .imuFiltHz = 10,
    .fxFiltHz = 20,
    .motorFiltHz = 40
);

// extern
learningRuntime_t learnRun = {0};

void initLearningRuntime(void) {
    // for future extension
}

// externs
float outputFromLearningQuery[MAX_SUPPORTED_MOTORS];
static timeUs_t learningQueryEnabledAt = 0;
static rls_parallel_t motorRls[MAXU];
static rls_t imuRls;
static rls_parallel_t fxSpfRls;
static rls_parallel_t fxRateDotRls;

static biquadFilter_t imuRateFilter[3];
static biquadFilter_t imuSpfFilter[3];

static biquadFilter_t motorOmegaFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t motorDFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxOmegaFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxUFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxRateFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxSpfFilter[MAX_SUPPORTED_MOTORS];

#define LEARNING_MAX_ACT ((int) RLS_MAX_N / 2)

// learning
void initLearner(void) {
    // limited to 4 for now
    learnerConfigMutable()->numAct = MIN(LEARNING_MAX_ACT, learnerConfigMutable()->numAct);

    rlsInit(&imuRls, 3, 3, 1e5f, 1.f);

    // init filters and other rls
    rlsParallelInit(&fxSpfRls, learnerConfig()->numAct, 3, 1e0f, 0.997f); // forces
    rlsParallelInit(&fxRateDotRls, 2.f*learnerConfig()->numAct, 3, 1e0f, 0.997f); // rotations
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        //rlsParallelInit(&fxRls[axis], learnerConfig()->numAct, 1, 1e0f, 0.997f); // forces
        //rlsParallelInit(&fxRls[axis+3], 2*learnerConfig()->numAct, 1, 1e0f, 0.997f); // rotational
        biquadFilterInitLPF(&imuRateFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&imuSpfFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxRateFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxSpfFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
    }

    for (int act = 0; act < learnerConfig()->numAct; act++) {
        rlsParallelInit(&motorRls[act], 4, 1, 1e7, 0.997);
        biquadFilterInitLPF(&fxOmegaFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxUFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&motorOmegaFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&motorDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
    }
}

#ifdef STM32H7
FAST_CODE
#endif
static void updateLearningFilters(void) {
    static t_fp_vector imuPrevRate = {0};
    static t_fp_vector fxPrevRateDot = {0};
    static t_fp_vector fxPrevSpf = {0};

#pragma message "TODO: implement imu location correction"
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        learnRun.imuRate.A[axis] = biquadFilterApply(&imuRateFilter[axis], indiRun.rate.A[axis]);
        learnRun.imuSpf.A[axis] = biquadFilterApply(&imuSpfFilter[axis], indiRun.spf.A[axis]);

        learnRun.imuRateDot.A[axis] = indiRun.indiFrequency * (learnRun.imuRate.A[axis] - imuPrevRate.A[axis]);
        imuPrevRate.A[axis] = learnRun.imuRate.A[axis];

        float fxRateDot = biquadFilterApply(&fxRateFilter[axis], indiRun.rateDot.A[axis]);
        learnRun.fxRateDotDiff.A[axis] = fxRateDot - fxPrevRateDot.A[axis];
        fxPrevRateDot.A[axis] = fxRateDot;

        float fxSpf = biquadFilterApply(&fxSpfFilter[axis], indiRun.spf.A[axis]);
        learnRun.fxSpfDiff.A[axis] = fxSpf - fxPrevSpf.A[axis];
        fxPrevSpf.A[axis] = fxSpf;
    }

    static float fxPrevOmega[MAX_SUPPORTED_MOTORS] = {0};
    static float motorPrevOmega[MAX_SUPPORTED_MOTORS] = {0};
    static float fxPrevOmegaDot[MAX_SUPPORTED_MOTORS] = {0};
    for (int act = 0; act < learnerConfig()->numAct; act++) {
        learnRun.fxOmega[act] = biquadFilterApply(&fxOmegaFilter[act], indiRun.omega[act]);
        learnRun.fxOmegaDiff[act] = learnRun.fxOmega[act] - fxPrevOmega[act];
        fxPrevOmega[act] = learnRun.fxOmega[act];

        float fxOmegaDot = indiRun.indiFrequency * learnRun.fxOmegaDiff[act];
        learnRun.fxOmegaDotDiff[act] = fxOmegaDot - fxPrevOmegaDot[act];
        fxPrevOmegaDot[act] = fxOmegaDot;

        learnRun.motorOmega[act] = biquadFilterApply(&motorOmegaFilter[act], indiRun.omega[act]);
        learnRun.motorOmegaDot[act] = indiRun.indiFrequency * (learnRun.motorOmega[act] - motorPrevOmega[act]);
        motorPrevOmega[act] = learnRun.motorOmega[act];

        learnRun.motorD[act] = biquadFilterApply(&motorDFilter[act], indiRun.d[act]);
        // second order filters could the signal to exceed bounds of the input
        // but we need to ensure [0, 1] for a square root later on
        learnRun.motorD[act] = constrainf(learnRun.motorD[act], 0.f, 1.f);
    }
}

#ifdef STM32H7
FAST_CODE
#endif
void updateLearner(timeUs_t current) {
    UNUSED(current);
    // last profile is for learning
    const indiProfile_t* p = indiProfiles(INDI_PROFILE_COUNT-1);
    UNUSED(p);

    // update learning sync filters
    updateLearningFilters();

    // wait for motors to spool down before learning imu position
    bool imuLearningConditions = ((learningQueryState == LEARNING_QUERY_DELAY) 
            && (cmpTimeUs(current, learningQueryEnabledAt) > 75000));

    if (imuLearningConditions) {
        // accIMU  =  accB  +  rateDot x R  +  rate x (rate x R)
        // assume accB = 0
        t_fp_vector_def* w = &learnRun.imuRate.V;
        t_fp_vector_def* dw = &learnRun.imuRateDot.V;
        t_fp_vector_def* a = &learnRun.imuSpf.V;

        // regressors

        // remember: column major formulation! So this looks like the transpose, but isn't
        float A[3*3] = {
            -(w->Y * w->Y + w->Z * w->Z),   w->X * w->Y + dw->Z,           w->X * w->Z - dw->Y,
             w->X * w->Y + dw->Z,          -(w->X * w->X + w->Z * w->Z),   w->Z * w->Z + dw->X,
             w->X * w->Z + dw->Y,           w->Z * w->Z - dw->X,          -(w->X * w->X + w->Y * w->Y)
        };
        float y[3] = { a->X, a->Y, a->Z };

        // perform rls step
        rlsNewSample(&imuRls, A, y);
    }

    bool fxLearningConditions = FLIGHT_MODE(LEARNER_MODE) && ARMING_FLAG(ARMED) && !isTouchingGround()
        && (
            (learnerConfig()->mode & LEARN_DURING_FLIGHT)
            || ((learnerConfig()->mode & LEARN_AFTER_CATAPULT) && (catapultState == CATAPULT_DONE))
           );

    if (fxLearningConditions) {
        //setup regressors
        float A[RLS_MAX_N];
        for (int act = 0; act < learnerConfig()->numAct; act++) {
            A[act] = 2 * learnRun.fxOmega[act] * learnRun.fxOmegaDiff[act];
            A[act + learnerConfig()->numAct] = learnRun.fxOmegaDotDiff[act];
        }

        // perform rls step
        rlsParallelNewSample(&fxSpfRls, A, learnRun.fxSpfDiff.A);
        rlsParallelNewSample(&fxRateDotRls, A, learnRun.fxRateDotDiff.A);
    }

    // same for now
    bool motorLearningConditions = fxLearningConditions;

    if (motorLearningConditions) {
        for (int act = 0; act < learnerConfig()->numAct; act++) {
            float A[4] = {
                learnRun.motorD[act],
                sqrtf(learnRun.motorD[act]),
                1.f,
                -learnRun.motorOmegaDot[act]
            };
            float y = learnRun.motorOmega[act];
            rlsParallelNewSample(&motorRls[act], A, &y);
        }
    }

    bool hoverAttitudeLearningConditions = false;

    if (hoverAttitudeLearningConditions) {
        // for QR: call sgegr2 and sorgr2 instead of sgeqrf and sorgrf.
        // this avoids code bloat and likely the blocking will not help
        // us anyway for our sizes of matrices
        // for the unblocked cholesky and cholesky-solve needed for W != I
        // use the dependency-less chol routines from maths.h

        // 1. Get Nullspace of Gr = indiRun.G1[3:][:]
        //      - Qh, R = QR(Gr.T)
        //      - Qh.T, L = LQ(Gr) // (sgelq2)
        //      --> Nr = last n - 3 columns of Qh
        //    Potentially more efficient hover solutions can be found if we use
        //    the last n - rk(Br) columns (to take into account the extra freedom)
        //    from not being able to satisfy Br u = 0)? This would require using 
        //    a rank-revealing factorization, such as sgeqp3
        // 2. form  Q = Nr.T W Nr. if W == I, then Q == I
        // 3. form  A = Nr.T Bf.T Bf Nr.  Maybe this is faster without sorgr2?
        // 4. find the only eigenvector of  Av = sigma Qv
        //      - probably best with power iteration
        //      - v <-- Q-1 A v / sqrt(vT AT QT-1 Q-1 A v)
        //      - Q-1 A v best done with cholesky solve
        //      -- 1 / sqrt could be fast-inverse-square-root
        // 5. find if  we need v or -v by  checking ensuring  sum(Nr v) > 0
    }
}


// query
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
                learningQueryEnabledAt = current;
                startAt = current + MIN(c->delayMs, LEARNING_SAFETY_TIME_MAX)*1e3;
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
        indiRun.d[motor] = outputFromLearningQuery[motor]; // for logging purposes. TODO: also log du
    }
}

#endif
