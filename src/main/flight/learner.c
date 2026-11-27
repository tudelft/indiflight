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


#include "common/maths.h"
#include "common/rls.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/rng.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "flight/ahrs.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "flight/catapult.h"
#include "flight/pos_ctl.h"
#include "flight/throw.h"
#include "ekf_calc.h"   // for dirty override
#include "flight/gain_schedule.h"

#include "f2c.h"
#include "clapack.h"

#include <stdbool.h>

#include "learner.h"
#include "flight/learning_prober.h"

learning_query_state_t learningQueryState = LEARNING_QUERY_IDLE;

#ifdef USE_LEARNER
#pragma message "You are compiling with dangerous code!"

#ifndef USE_INDI
#error "must use learner with USE_INDI"
#endif

// defaults that differ between multirotors (modular_drone) and tailsitters (testing_tailsitter_idflights_beun)
#ifdef LEARNER_IS_TAILSITTER
#define LEARNER_DEFAULT(multirotor, tailsitter) (tailsitter)
#else
#define LEARNER_DEFAULT(multirotor, tailsitter) (multirotor)
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(learnerConfig_t, learnerConfig, PG_LEARNER_CONFIG, 6);
PG_RESET_TEMPLATE(learnerConfig_t, learnerConfig, 
    .modeProbing = (uint8_t) (LEARN_PROBING_AFTER_CATAPULT | LEARN_PROBING_AFTER_THROW),
    // in-flight learning (LEARN_DURING_FLIGHT) is available, but off by default for all crafts
    .modeFx    = (uint8_t) (LEARN_DURING_PROBING),
    .modeAct   = (uint8_t) (LEARN_DURING_PROBING),
    .modeHover = (uint8_t) (LEARN_DURING_PROBING),
    .mixControlAfterMs = 1000,
    .initFromProfileFx = false,
    .initFromProfileAct = false,
    .actMask = 0xFFFF, // all motors and servos
    .imuFiltHz = 10,
    .fxFiltHz = 15,
    .motorFiltHz = 40,
    .servoFiltHz = 20,
    .useFortescue = false,
    .zetaRate = 80,
    .zetaAttitude = 80,
    .zetaVelocity = 60,
    .zetaPosition = 80,
    .zetaVelocityVert = 90,
    .zetaPositionVert = 100,
    .rollMisalignment = 0,
    .pitchMisalignment = 0,
    .yawMisalignment = 0,
    .randomizeMisalignment = false,
    .applyIndi = false,
    .applyPosition = false,
    .applyHoverRotation = false,
    .simpleMotorModel = LEARNER_DEFAULT(true, false), // modular_drone: since f5d3392 "simplify motor model"
    .rlsLambda = 1000000,     // lambda = 1 (modular_drone since 5bf60bd). 0: bandwidth equation, see learnerSampleLambda
    .hardcodeMaxRpm = 0,      // 0: learned (was build flag LEARNER_HARDCODE_MAXRPM -> 37000)
    .hardcodeG1Fz = 0,
    .hardcodeLinearForces = false, // 0: learned (was build flag LEARNER_HARDCODE_LINEAR_FORCES -> 0, 0, -438)
    .hardcodeG1Fx = 0,
    .hardcodeG1Fy = 0,
    .fxIncremental = LEARNER_DEFAULT(true, false),
);

// extern
learnerRuntime_t learnRun = {0};

float outputFromLearningQuery[MAXU];
static timeUs_t learningQueryEnabledAt = 0;
rls_t actRls[MAXU];
rls_t imuRls;
//rls_parallel_t fxSpfRls;
//rls_parallel_t fxRateDotRls;
rls_t fxRls[6];
fp_quaternion_t hoverAttitude = {.w=1.f, .x=0.f, .y=0.f, .z=0.f};
learnerTimings_t learnerTimings = {0};

static biquadFilter_t imuRateFilter[3];
static biquadFilter_t imuSpfFilter[3];

static biquadFilter_t actOmegaFilter[MAXU];
static biquadFilter_t actAngleFilter[MAXU];
static biquadFilter_t actDFilter[MAXU];
static biquadFilter_t actSqrtDFilter[MAXU];
static biquadFilter_t fxOmegaFilter[MAXU];
// static biquadFilter_t fxAngleFilter[MAXU];
static biquadFilter_t fxRateFilter[3];
static biquadFilter_t fxSpfFilter[3];

static fp_vector_t hoverThrust;

#define LEARNING_MAX_ACT (RLS_MAX_N >> 1) // divide by 2
#define LEARNER_OMEGADOT_SCALER 1e-5f // for numerical stability
#define LEARNER_OMEGADOTDIFF_SCALER 10.f // for numerical stability
#define LEARNER_NULLSPACE_ABS_THRESH 5.f // fx matrix is in integer format, so 5 seems reasonable
#define LEARNER_NULLSPACE_REL_THRESH 1e-2f // 1% of other axes
#define LEARNER_NUM_POWER_ITERATIONS 3

static fp_vector_t actG1linIMU[LEARNING_MAX_ACT] = {0}; // in IMU frame, in indiConfig units
static fp_vector_t actG1rotIMU[LEARNING_MAX_ACT] = {0};
static fp_vector_t actG2rotIMU[LEARNING_MAX_ACT] = {0};

static indiProfile_t* indiProfileLearned;
#ifdef USE_LOCAL_POSITION
static positionProfile_t* positionProfileLearned;
#else
static positionProfile_t dummy;
static positionProfile_t* positionProfileLearned = &dummy;
#endif

// idle speed subtracted in the simple motor model (learner_simple_motor_model = 1)
#define LEARNER_HARDCODED_IDLE_OMEGA_RAD_PER_S (500.f)
// lower bound on a user-set forgetting factor, below this the covariance blows up within a query
#define LEARNER_RLS_LAMBDA_MIN 0.9f

static float learnerIdleOmega(void) {
    return learnerConfig()->simpleMotorModel ? LEARNER_HARDCODED_IDLE_OMEGA_RAD_PER_S : 0.f;
}

// learner_fx_incremental. Tailsitters only support the global model
static bool learnerFxIncremental(void) {
#ifdef LEARNER_IS_TAILSITTER
    return false;
#else
    return learnerConfig()->fxIncremental;
#endif
}

// forgetting factor from the bandwidth equation of modular_drone's rlsInit,
// set by initLearnerRls (depends on query vs. in-flight restart)
static float learnerBandwidthLambda = 1.f;

static float rlsBandwidthLambda(float bandwidthHz) {
    const float sampleFreqHz = 1e6f / ((float) gyro.targetLooptime);
    return powf(1.f - (float) M_LN2f, (2.f * M_PIf * bandwidthHz) / sampleFreqHz);
}

// forgetting factor for one sample of the effectiveness (fx) RLS. modular_drone semantics:
//   learner_rls_lambda > 0:    that value * 1e-6, always
//   learner_rls_lambda == 0:   from the bandwidth equation (about 0.995 during a query)
// except learner_use_fortescue = 1 with learner_rls_lambda == 0 (tailsitter branch):
//   no forgetting while probing, fortescue afterwards
static float learnerSampleLambda(bool probing) {
    const learnerConfig_t *config = learnerConfig();
    if (config->rlsLambda > 0) {
        return constrainf(1e-6f * (float) config->rlsLambda, LEARNER_RLS_LAMBDA_MIN, 1.f);
    }
    if (config->useFortescue) {
        return probing ? 1.f : 0.f; // 0: fortescue (see rlsNewSample)
    }
    return learnerBandwidthLambda;
}

// same for the imu and actuator RLS, which never use fortescue
static float learnerFixedLambda(void) {
    const learnerConfig_t *config = learnerConfig();
    if (config->rlsLambda > 0) {
        return constrainf(1e-6f * (float) config->rlsLambda, LEARNER_RLS_LAMBDA_MIN, 1.f);
    }
    return config->useFortescue ? 1.f : learnerBandwidthLambda;
}

// learning
void initLearnerFilters(void) {
    learnRun.filtersInitialized = false;

    // get number of actuators
    int numActuators = 0;
    for (int i = 0; i < indiRun.actNum; i++) {
        if (learnerConfig()->actMask & (1 << i)
                && (indiRun.actType[i] != INDI_ACT_TYPE_OFF)) {
            numActuators++;
        }
    }

    if (numActuators > LEARNING_MAX_ACT) {
        return;
    }

    learnRun.numActuators = numActuators;

    indiProfileLearned = indiProfilesMutable(INDI_PROFILE_COUNT-1);
#ifdef USE_LOCAL_POSITION
    positionProfileLearned = positionProfilesMutable(POSITION_PROFILE_COUNT-1);
#endif

    learnRun.zeta[LEARNER_LOOP_HORIZONTAL_RATE]     = constrainf(0.01f * learnerConfig()->zetaRate    , 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_HORIZONTAL_ATTITUDE] = constrainf(0.01f * learnerConfig()->zetaAttitude, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_HORIZONTAL_VELOCITY] = constrainf(0.01f * learnerConfig()->zetaVelocity, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_HORIZONTAL_POSITION] = constrainf(0.01f * learnerConfig()->zetaPosition, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_VERTICAL_VELOCITY]   = constrainf(0.01f * learnerConfig()->zetaVelocityVert, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_VERTICAL_POSITION]   = constrainf(0.01f * learnerConfig()->zetaPositionVert, 0.5f, 1.0f);

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        //rlsParallelInit(&fxRls[axis], learnerConfig()->numAct, 1, 1e0f, 0.997f); // forces
        //rlsParallelInit(&fxRls[axis+3], 2*learnerConfig()->numAct, 1, 1e0f, 0.997f); // rotational
        biquadFilterInitLPF(&imuRateFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&imuSpfFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxRateFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxSpfFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
    }

    int m = 0, s = 0;
    for (int act = 0; act < indiRun.actNum; act++) {
        if (!(learnerConfig()->actMask & (1 << act))) {
            continue;
        }

        switch(indiRun.actType[act]) {
            case INDI_ACT_TYPE_MOTOR:
                if (m >= MAX_SUPPORTED_MOTORS) { return; } // abort if too many motors
                biquadFilterInitLPF(&actDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
                biquadFilterInitLPF(&actSqrtDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);

                biquadFilterInitLPF(&fxOmegaFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
                biquadFilterInitLPF(&actOmegaFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
                m++;
                break;
            case INDI_ACT_TYPE_SERVO:
                if (s >= MAX_SUPPORTED_SERVOS) { return; } // abort if too many servos
                // not implemented yet
                biquadFilterInitLPF(&actDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
                biquadFilterInitLPF(&actSqrtDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);

                biquadFilterInitLPF(&fxOmegaFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
                biquadFilterInitLPF(&actOmegaFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
                s++;
                break;
            case INDI_ACT_TYPE_OFF:
            default:
                break; // skip unsupported actuator types
        }
    }

    // todo: implement servo filters

    learnRun.filtersInitialized = true;
    learnRun.mixControl = false;
    learnRun.numMotors = m;
    learnRun.numServos = s;
}

#define LEARNER_INIT_COV_QUERY 1e2f
#define LEARNER_INIT_COV_RESET 1e1f

// not FAST_CODE: runs once per query/takeoff, and with two callers LTO no
// longer inlines it, so it would take 2.3kB of ITCM
// fromProfileInFlight: (re)start in-flight learning from the current profile
// (modular_drone setFxRlsToProfile). Leaves the IMU and servo estimates alone.
static void initLearnerRls(bool fromProfileInFlight) {
    if (!learnRun.filtersInitialized) {
        learnRun.initialized = false;
        return; 
    }

    const indiProfile_t *p = indiProfiles(systemConfig()->indiProfileIndex);
    const learnerConfig_t *config = learnerConfig();
    const bool incremental = learnerFxIncremental();
    const bool initFx  = config->initFromProfileFx  || fromProfileInFlight;
    const bool initAct = config->initFromProfileAct || fromProfileInFlight;

    float actionBandwidthHz;
    float covImu, covSpf, covRot, covAct;
    if (incremental) {
        // modular_drone. Fortescue only estimates the error variance (logged),
        // the forgetting factor comes from learnerSampleLambda
        actionBandwidthHz = 680.f / ((float) gyro.targetLooptime);
        covImu = 1e2f;
        covSpf = covRot = covAct = fromProfileInFlight ? LEARNER_INIT_COV_RESET : LEARNER_INIT_COV_QUERY;
    } else {
        actionBandwidthHz = 0.1f * ( 1. / (2.f * M_PIf * 0.025f) ); // 5 times slower than assumed fastest actuator
        if (!config->useFortescue && (config->rlsLambda == 0)) {
            actionBandwidthHz = 0; // default to RLS with constant forgetting
        }
        covImu = 1e2f;
        covSpf = 1e-4f;
        covRot = 1e-2f;
        covAct = 1e0f;
    }
    const float actBandwidthHz = incremental ? actionBandwidthHz : 0.f;

    // learner_rls_lambda = 0: modular_drone's bandwidth equation. A query used
    // 680 / looptime Hz (lambda ~0.995), an in-flight restart from the profile
    // (setFxRlsToProfile) 0.2 / (2 pi 15ms) Hz
    learnerBandwidthLambda = rlsBandwidthLambda(fromProfileInFlight
            ? 0.2f * ( 1.f / (2.f * M_PIf * 0.015f) )
            : 680.f / ((float) gyro.targetLooptime));

    if (!fromProfileInFlight) {
        rlsInit(&imuRls, 3, 3, covImu, gyro.targetLooptime, actionBandwidthHz);
    }

    // init filters and other rls
    //rlsParallelInit(&fxSpfRls, learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // forces
    //rlsParallelInit(&fxRateDotRls, 2*learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // rotations need twice the parameters

    // Spf
    for (int i = 0; i < 3; i++) {
        rlsInit(&fxRls[i], learnRun.numActuators, 1, covSpf, gyro.targetLooptime, actionBandwidthHz);
    }

    // RateDot
    for (int i = 3; i < 6; i++) {
        rlsInit(&fxRls[i], 1 + 2*learnRun.numActuators, 1, covRot, gyro.targetLooptime, actionBandwidthHz);
    }

    // princ. inertia ratios set to zero: all princ. inertias are the same
    fxRls[3].x[0] = 0.f;
    fxRls[4].x[0] = 0.f;
    fxRls[5].x[0] = 0.f;

    // the effectiveness RLS only hold the selected motors, packed: motor m (the
    // m-th selected motor, not the actuator index) is x[m] of the spf RLS and
    // x[1 + m], x[1 + numMotors + m] of the rotation RLS. Same as the regressors
    // in updateLearner and the read-back in updateLearnedParameters.
    int m = 0;
    for (int act = 0; act < indiRun.actNum; act++) {
        if (!(config->actMask & (1 << act))) {
            continue;
        }

        switch(indiRun.actType[act]) {
            case INDI_ACT_TYPE_MOTOR:
                rlsInit(&actRls[act], 4, 1, covAct, gyro.targetLooptime, actBandwidthHz);

                // inverse of updateLearnedParameters
                float maxOmega = 2.f * M_PIf / 60.f  *  p->actMaxRpm[act];
                float isq = 1.f / sq(maxOmega);

                if (initAct) {
                    if (config->simpleMotorModel) {
                        // omega - idle = a*d  ->  a = maxOmega - idle
                        actRls[act].x[0] = 1e-3f * (maxOmega - learnerIdleOmega());
                        actRls[act].x[1] = 0.f;
                    } else {
                        float k = 0.01f * p->actNonlinearity[act];
                        actRls[act].x[0] = 1e-3f * k * maxOmega;
                        actRls[act].x[1] = 1e-3f * (1.f - k) * maxOmega;
                    }
                    actRls[act].x[2] = 0.;
                    actRls[act].x[3] = 1e-3f * 1e4f * 1e-3f * (p->actTimeConstMs[act]);
                }

                // todo: rotate with hover rotation
                if (initFx) {
                    fxRls[0].x[m] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fx[act];
                    fxRls[1].x[m] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fy[act];
                    fxRls[2].x[m] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fz[act];

                    fxRls[3].x[1 + m] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_roll[act];
                    fxRls[4].x[1 + m] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_pitch[act];
                    fxRls[5].x[1 + m] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_yaw[act];

                    fxRls[3].x[1 + learnRun.numMotors + m] = 1.f  * 1e3f * 1e-5f * p->actG2_roll[act];
                    fxRls[4].x[1 + learnRun.numMotors + m] = 1.f  * 1e3f * 1e-5f * p->actG2_pitch[act];
                    fxRls[5].x[1 + learnRun.numMotors + m] = 1.f  * 1e3f * 1e-5f * p->actG2_yaw[act];
                }
                m++;
                break;
            case INDI_ACT_TYPE_SERVO:
                if (!fromProfileInFlight) { // keep the servo identification from before the launch
                    rlsInit(&actRls[act], 4, 1, 1e0f, gyro.targetLooptime, 0);
                }
                break;
            case INDI_ACT_TYPE_OFF:
            default:
                break; // skip unsupported actuator types
        }
    }

    if (initAct) {
        learnRun.gains[LEARNER_LOOP_HORIZONTAL_RATE] = 0.1f * p->rateGains[0];
        learnRun.gains[LEARNER_LOOP_HORIZONTAL_ATTITUDE] = ((float) p->attGains[0]) / ((float) p->rateGains[0]);
    }

    learnRun.initialized = true;
}

#include "flight/servos.h"

#ifdef STM32H7
FAST_CODE
#endif
static void updateLearningFilters(void) {
    if (!learnRun.initialized) {
        return; // no can do
    }

    const learnerConfig_t *config = learnerConfig();

    static fp_vector_t imuPrevRate = {0};

    // IMU rls filters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        learnRun.imuRate.A[axis] = biquadFilterApply(&imuRateFilter[axis], indiRun.rateIMU.A[axis]);
        learnRun.imuSpf.A[axis] = biquadFilterApply(&imuSpfFilter[axis], indiRun.spfIMU.A[axis]);

        learnRun.imuRateDot.A[axis] = indiRun.indiFrequency * (learnRun.imuRate.A[axis] - imuPrevRate.A[axis]);
        imuPrevRate.A[axis] = learnRun.imuRate.A[axis];
    }

    // IMU correction
    float wx, wy, wz;
    float rx, ry, rz;
    float dwx, dwy, dwz;
    float ax, ay, az;

    rx = accelerometerConfig()->acc_offset[0] * 1e-3f;
    ry = accelerometerConfig()->acc_offset[1] * 1e-3f;
    rz = accelerometerConfig()->acc_offset[2] * 1e-3f;

    dwx = indiRun.rateDotIMU.A[0];
    dwy = indiRun.rateDotIMU.A[1];
    dwz = indiRun.rateDotIMU.A[2];

    wx = indiRun.rateIMU.A[0];
    wy = indiRun.rateIMU.A[1];
    wz = indiRun.rateIMU.A[2];

    ax = indiRun.spfIMU.A[0];
    ay = indiRun.spfIMU.A[1];
    az = indiRun.spfIMU.A[2];

    float fxSpfCorrected[3];
    fxSpfCorrected[0] = ax - ( rx * (-sq(wy)-sq(wz)) + ry * (wx*wy - dwz)    + rz * (wx*wz + dwy)    );
    fxSpfCorrected[1] = ay - ( rx * (wx*wy + dwz)    + ry * (-sq(wx)-sq(wz)) + rz * (wy*wz - dwx)    );
    fxSpfCorrected[2] = az - ( rx * (wx*wz - dwy)    + ry * (wy*wz + dwx)    + rz * (-sq(wx)-sq(wy)) );

    static fp_vector_t fxPrevRateDot = {0};
    static fp_vector_t fxPrevSpf = {0};
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        learnRun.fxRateDot.A[axis] = biquadFilterApply(&fxRateFilter[axis], indiRun.rateDotIMU.A[axis]);
        learnRun.fxSpf.A[axis] = biquadFilterApply(&fxSpfFilter[axis], fxSpfCorrected[axis]);

        learnRun.fxRateDotDiff.A[axis] = learnRun.fxRateDot.A[axis] - fxPrevRateDot.A[axis];
        fxPrevRateDot.A[axis] = learnRun.fxRateDot.A[axis];
        learnRun.fxSpfDiff.A[axis] = learnRun.fxSpf.A[axis] - fxPrevSpf.A[axis];
        fxPrevSpf.A[axis] = learnRun.fxSpf.A[axis];
    }

    static float fxPrevOmega[MAX_SUPPORTED_MOTORS] = {0};
    static float fxPrevOmegaDot[MAX_SUPPORTED_MOTORS] = {0};
    //static float fxPrevAngle[MAX_SUPPORTED_SERVOS] = {0};
    static float motorPrevOmega[MAX_SUPPORTED_MOTORS] = {0};

    UNUSED(actAngleFilter);

    int s = 0;
    for (int act = 0; act < indiRun.actNum; act++) {
        if (!(config->actMask & (1 << act))) {
            continue; // skip unselected actuators
        }

        switch(indiRun.actType[act]) {
            case INDI_ACT_TYPE_MOTOR:
                learnRun.fxOmega[act] = biquadFilterApply(&fxOmegaFilter[act], indiRun.omega[act]);
                learnRun.fxOmegaDiff[act] = learnRun.fxOmega[act] - fxPrevOmega[act];
                fxPrevOmega[act] = learnRun.fxOmega[act];

                float fxOmegaDot = indiRun.indiFrequency * learnRun.fxOmegaDiff[act];
                learnRun.fxOmegaDot[act] = fxOmegaDot;
                learnRun.fxOmegaDotDiff[act] = fxOmegaDot - fxPrevOmegaDot[act];
                fxPrevOmegaDot[act] = fxOmegaDot;

                learnRun.motorOmega[act] = biquadFilterApply(&actOmegaFilter[act], indiRun.omega[act]);
                learnRun.motorOmegaDot[act] = indiRun.indiFrequency * (learnRun.motorOmega[act] - motorPrevOmega[act]);
                motorPrevOmega[act] = learnRun.motorOmega[act];

                learnRun.motorD[act] = biquadFilterApply(&actDFilter[act], indiRun.d[act]);
                // second order filters could the signal to exceed bounds of the input
                // but we need to ensure [0, 1] for the square root
                float dConstr = constrainf(indiRun.d[act], 0.f, 1.f);
                learnRun.motorSqrtD[act] = biquadFilterApply(&actSqrtDFilter[act], sqrtf(dConstr));

                break;
            case INDI_ACT_TYPE_SERVO: {
                float servo_angle_rad = DEGREES_TO_RADIANS(((float)servo_feedback[s])*0.01f);
                learnRun.fxOmega[act] = biquadFilterApply(&fxOmegaFilter[act], servo_angle_rad);

                learnRun.fxOmegaDiff[act] = learnRun.fxOmega[act] - fxPrevOmega[act];
                learnRun.fxOmegaDot[act] = indiRun.indiFrequency * learnRun.fxOmegaDiff[act];

                float fxOmegaDotDiff = learnRun.fxOmegaDot[act] - fxPrevOmegaDot[act];
                learnRun.fxOmegaDotDot[act] = indiRun.indiFrequency * fxOmegaDotDiff;

                fxPrevOmega[act] = learnRun.fxOmega[act];
                fxPrevOmegaDot[act] = learnRun.fxOmegaDot[act];

                learnRun.motorOmega[act] = biquadFilterApply(&actOmegaFilter[act], servo_angle_rad);
                learnRun.motorOmegaDot[act] = indiRun.indiFrequency * (servo_angle_rad - motorPrevOmega[act]);
                motorPrevOmega[act] = servo_angle_rad;

                learnRun.motorD[act] = biquadFilterApply(&actDFilter[act], indiRun.d[act]);
                s++;

                break;
            } case INDI_ACT_TYPE_OFF:
            default:
                break; // skip unsupported actuator types
        }
    }
}

static float rotNull[LEARNING_MAX_ACT*(LEARNING_MAX_ACT - 3)];
static int sizeRotNull = 0;

static bool calculateHoverAttitude(void) {
    // for QR: call sgegr2 and sorgr2 instead of sgeqrf and sorgrf.
    // this avoids code bloat and likely the blocking will not help
    // us anyway for our sizes of matrices
    // for the unblocked cholesky and cholesky-solve needed for W != I
    // use the dependency-less chol routines from maths.h

    // 1. Get Nullspace of Br = indiRun.G1[3:][:]
    //      - Qh, R = QR(Br.T)
    //      - Qh.T, L = LQ(Br) // (sgelq2)
    //      --> Nr = last n - 3 columns of Qh
    //    Potentially more efficient hover solutions can be found if we use
    //    the last n - rk(Br) columns (to take into account the extra freedom)
    //    from not being able to satisfy Br u = 0)? This would require using 
    //    a rank-revealing factorization, such as sgeqp3
    // 2. form  H = Nr.T W Nr. if W == I, then H == I
    // 3. form  A = Nr.T Bf.T Bf Nr.  Maybe this is faster without sorgr2?
    // 4. find the only eigenvector of  Av = sigma Hv
    //      - probably best with power iteration
    //      - v <-- H-1 A v / sqrt(vT AT HT-1 H-1 A v)
    //      - H-1 A v best done with cholesky solve
    //      -- 1 / sqrt could be fast-inverse-square-root
    // 5. scale v to satisfy  vT A v == GRAVITYf*GRAVITYf
    // 6. find uHover = Nr v and if we need v or -v by ensuring  sum(Nr v) > 0
    // 7. find hover thrust direction as  Bf uHover
    // 8. verify that  Br uHover == 0

    // column major
    const uint8_t numAct = learnRun.numMotors; // actG1*IMU hold the selected motors, packed
    float BfT[LEARNING_MAX_ACT * 3];
    float BrT[LEARNING_MAX_ACT * LEARNING_MAX_ACT] = {0}; // waste of stack, reduce because M < N?
    for (int motor = 0; motor < numAct; motor++) {
        // note! BfT and BrT are the transpose of Bf/Br
        BfT[motor + 0*numAct] = actG1linIMU[motor].V.X;
        BfT[motor + 1*numAct] = actG1linIMU[motor].V.Y;
        BfT[motor + 2*numAct] = actG1linIMU[motor].V.Z;
        BrT[motor + 0*numAct] = actG1rotIMU[motor].V.X;
        BrT[motor + 1*numAct] = actG1rotIMU[motor].V.Y;
        BrT[motor + 2*numAct] = actG1rotIMU[motor].V.Z;
    }

    integer M = numAct;
    integer N = XYZ_AXIS_COUNT;
    if (M < N) return false; // not implemented, would have to adjust sorg2r inputs?

    // A is BrT
    integer LDA = numAct;
    integer JPVT[LEARNING_MAX_ACT] = {0}; // all free columns on entry
    real TAU[XYZ_AXIS_COUNT];
    real WORK[3*XYZ_AXIS_COUNT + 1]; 
    integer LWORK = 3*XYZ_AXIS_COUNT + 1;  // see sgepq3 manual
    integer INFO;
    sgeqp3_(&M, &N, BrT, &LDA, JPVT, TAU, WORK, &LWORK, &INFO);
    if (INFO < 0) return false; // panic

    // since we used sgeqp3_, the columns of the R factor are sorted so
    // that the diagonals are non-increasing. To find if we have rank-
    // deficiency (and this a larger nullspace), we can just find the
    // last non-zero diagonal element of R
    // For our context, we can be a bit stricter. The rank indicates rotational
    // controllability: if full rank, then we can generate moments around all
    // axes. if not full rank, we cannot. So we don't just check for 0, but 
    // for at least 1% of the other (more controllable) axes.
    float avgControllability = fabsf(BrT[0]);
    if (avgControllability < LEARNER_NULLSPACE_ABS_THRESH) return false;
    for (int dim = 1; dim < XYZ_AXIS_COUNT; dim++) {
        float nextDim = fabsf(BrT[dim + dim*numAct]);
        if (nextDim < LEARNER_NULLSPACE_REL_THRESH*avgControllability) return false;
        avgControllability = ( avgControllability * dim + nextDim ) / (dim + 1);
    }

    // todo interpret INFO
    sizeRotNull = numAct - XYZ_AXIS_COUNT; // Br is now assumed full rank
    integer K = XYZ_AXIS_COUNT; // Br is now assumed full rank
    sorg2r_(&M, &M, &K, BrT, &LDA, TAU, WORK, &INFO); // K == N true? or M - sizeRotNull?

    float *Nr = &BrT[K*numAct];

    // save nullspace
    for (int col = 0; col < sizeRotNull; col++)
        for (int row = 0; row < numAct; row++ ) {
            if (Nr[col*numAct + row] != Nr[col*numAct + row])
                return false;
            rotNull[col*numAct + row] = Nr[col*numAct + row];
        }

    // 2. Generate H. allow only W = I for now.
    //float H[LEARNING_MAX_ACT*LEARNING_MAX_ACT]; // todo could be smaller since we disallow M < N?
    //SGEMMt(sizeRotNull, sizeRotNull, numAct, Nr, Nr, H, 0.f, 1.f);
    // jokes, this is always I, if W = I

    // 3. Generate A
    float BfNr[3*LEARNING_MAX_ACT]; // todo could be smaller since we disallow M < N?
    float A[LEARNING_MAX_ACT*LEARNING_MAX_ACT]; // could also be smaller! will be a waste of stack.. allocate on the RAM?
    SGEMMt(3, sizeRotNull, numAct, BfT, Nr, BfNr, 0.f, 1.f);
    SGEMMt(sizeRotNull, sizeRotNull, 3, BfNr, BfNr, A, 0.f, 1.f);

    // 4. find eigenvector. Remember H == I
    static float v[LEARNING_MAX_ACT] = {-0.5051f,  0.3486f, -0.9154f,  0.5560f}; // can also be smaller since M < N
    //static float v[LEARNING_MAX_ACT] = {0}; // for testing robustness, 0 makes no sense
    float HinvAv[LEARNING_MAX_ACT]; // can be smaller
    float HinvAvNorm2;
    for (int i = LEARNER_NUM_POWER_ITERATIONS; i > 0; i--) {
        SGEMVt(sizeRotNull, sizeRotNull, A, v, HinvAv); // A is symmetric, SGEMVf is faster
        SGEVV(sizeRotNull, HinvAv, HinvAv, HinvAvNorm2); // guaranteed >= 0.f
        if (HinvAvNorm2 < 1e-8f) {
            // we picked a starting vector near orthogonal to the eigenvector
            // we want to find. reset to a pseudorandom vector
            for (int row = 0; row < sizeRotNull; row++)
                v[row] = rngFloat();
            continue;
        }
        SGEVS(sizeRotNull, HinvAv, 1.f / sqrtf(HinvAvNorm2), v); // fast inverse sqrt anyone?
    }

    // 5. find length for v to cancel gravity
    // this seems not strictly necessary if you only want the direction,
    // now that I think about it.. but maybe it's still good to cross check
    // the (linearized) hover thrust.
    float Av[LEARNING_MAX_ACT];
    float vTAv;
    SGEMVt(sizeRotNull, sizeRotNull, A, v, Av);
    SGEVV(sizeRotNull, v, Av, vTAv);
    if (vTAv < 1e-10f) return false; // panic

    float scale = GRAVITYf * 1.f / sqrtf( vTAv ); // fast inverse sqrt?
    SGEVS(sizeRotNull, v, scale, v);

    // 6. find if up or down
    float uHover[LEARNING_MAX_ACT];
    SGEMV(numAct, sizeRotNull, Nr, v, uHover);
    float uHoverSum = 0.f;
    for (int row = 0; row < numAct; row++)
        uHoverSum += uHover[row];

    if (uHoverSum < 0.f) {
        for (int row = 0; row < sizeRotNull; row++)
            v[row] = -v[row];
        for (int row = 0; row < numAct; row++)
            uHover[row] = -uHover[row];
    }

    for (int row = 0; row < numAct; row++) {
        if ((uHover[row] < -2.f) || (uHover[row] > 2.f))
            return false; // very unlikely to hover because actuator limits
    }

    // 7. hover thrust direction
    // Bf * u
    SGEMVt(numAct, 3, BfT, uHover, hoverThrust.A);
    VEC3_NORMALIZE(hoverThrust);

    // 8. verify that Br uHover == 0
    // SKIP

    // 9. compute tilt quaternion
    fp_vector_t up   = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -1.f };
    fp_vector_t orth = { .V.X = 1.f, .V.Y = 0.f, .V.Z = 0.f };
    if (learnerConfig()->applyHoverRotation)
        quaternion_of_two_vectors(&hoverAttitude, &up, &hoverThrust, &orth);
    //if (hoverAttitude.w != hoverAttitude.w)
    //    __asm("BKPT #1\n");

    return true;
}

// learned INDI parameters were applied in flight, but the INDI filters still
// use the old ones (see the end of updateLearner)
static bool learnerIndiFiltersStale = false;

static NOINLINE void learnerRetuneIndiFilters(void) {
    if (learnerIndiFiltersStale) {
        if (systemConfig()->indiProfileIndex == INDI_PROFILE_COUNT-1) {
            initIndiRuntime(); // keeps the filter states while armed
        }
        learnerIndiFiltersStale = false;
    }
}

static NOINLINE void learnerApplyProfiles(bool learning) { // not FAST_CODE: ITCM is full
    const learnerConfig_t *config = learnerConfig();

    if (learning && config->applyIndi) {
        // not changeIndiProfile: that re-tunes the filters
        systemConfigMutable()->indiProfileIndex = INDI_PROFILE_COUNT-1;
        initIndiRuntimeParameters();
        learnerIndiFiltersStale = true;
    }

    static bool appliedAfterQuery = false;
    if (!appliedAfterQuery && (learningQueryState == LEARNING_QUERY_DONE)) {
        if (config->applyIndi) {
            changeIndiProfile(INDI_PROFILE_COUNT-1); // re-tunes filters, keeps their state while armed
            learnerIndiFiltersStale = false;
        }

#ifdef USE_LOCAL_POSITION
        if (config->applyPosition) {
            changePositionProfile(POSITION_PROFILE_COUNT-1); 
        }
#endif
        appliedAfterQuery = true;
    }

    appliedAfterQuery &= !(learningQueryState == LEARNING_QUERY_IDLE);

    if (!learning) {
        learnerRetuneIndiFilters();
    }
}

#ifdef STM32H7
FAST_CODE
#endif
void updateLearner(timeUs_t current) {
    UNUSED(current);

    const learnerConfig_t *config = learnerConfig();

    bool airborne = ARMING_FLAG(ARMED)
            && FLIGHT_MODE(LEARNER_MODE)
            && !isTouchingGround()
            && (learningQueryState != LEARNING_QUERY_WAITING_FOR_LAUNCH);

    // modular_drone: in-flight learning without in-flight probing (re)starts
    // from the current profile every time we get airborne
    static bool inflightInitDone = false;
    if (!airborne) {
        inflightInitDone = false;
    } else if (!inflightInitDone
            && learnerFxIncremental()
            && (config->modeFx & LEARN_DURING_FLIGHT)
            && !(config->modeProbing & LEARN_PROBING_DURING_FLIGHT)) {
        initLearnerRls(true);
        inflightInitDone = true;
    }

    if (!learnRun.initialized) {
        return; // no can do
    }

    learnerTimings.start = micros();

    // update learning sync filters
    updateLearningFilters();
    learnerTimings.filters = cmpTimeUs(micros(), learnerTimings.start);

    // if we are not armed, we do not learn
    if (!airborne) {
        learnerRetuneIndiFilters(); // in-flight learning ended
        return;
    }

    bool probing = proberRuntime.isGenRunning;

    bool learnFx = (config->modeFx & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeFx & LEARN_DURING_PROBING) && probing);

    bool learnAct = (config->modeAct & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeAct & LEARN_DURING_PROBING) && probing);

    bool learnHover = (config->modeHover & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeHover & LEARN_DURING_PROBING) && probing);

    // between launch and the start of the excitation (prober_pre_delay_ms):
    // wait 100ms for the motors to spool down, then learn the imu position
    bool imuLearningConditions = (learningQueryState == LEARNING_QUERY_DELAY)
            && (cmpTimeUs(current, learningQueryEnabledAt) > 100000)
            && (cmpTimeUs(current, proberRuntime.genStartTimeUs) < 0);

    if (imuLearningConditions) {
        // accIMU  =  accB  +  rateDot x R  +  rate x (rate x R)
        // assume accB = 0
        fp_vector_def* w = &learnRun.imuRate.V;
        fp_vector_def* dw = &learnRun.imuRateDot.V;
        fp_vector_def* a = &learnRun.imuSpf.V;

        // regressors

        // remember: column major formulation!
        float AT[3*3] = {
            -(w->Y * w->Y + w->Z * w->Z),   w->X * w->Y + dw->Z,           w->X * w->Z - dw->Y,
             w->X * w->Y - dw->Z,          -(w->X * w->X + w->Z * w->Z),   w->Y * w->Z + dw->X,
             w->X * w->Z + dw->Y,           w->Y * w->Z - dw->X,          -(w->X * w->X + w->Y * w->Y)
        };
        float y[3] = { a->X, a->Y, a->Z }; // in the 1 - 10 m/s/s range id say

        for (int i = 0; i < 9; i++) {
            AT[i] *= 1e-2f; // parameters are in the cm range, so make sure they will be around 1 to avoid numerical issues
            // it if true parameter is 30cm is logged at in 0.3*100*1000 = 30000, which is max for logging. 1mm is logged as 0.001*100*1000 = 100
        }

        // perform rls step
        rlsNewSample(&imuRls, AT, y, learnerFixedLambda());
    }
    learnerTimings.imu = cmpTimeUs(micros(), learnerTimings.start);

    if (learnFx) {
        // setup regressors
        float A[RLS_MAX_N] = {0};
        float ySpf[3];
        float yRateDot[3];

        const bool incremental = learnerFxIncremental();

        int m = 1, s = 1; 
//#ifndef LEARNER_IS_TAILSITTER
        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act))) {
                continue; // skip unselected actuators
            }

            switch (indiRun.actType[act]) {
                case INDI_ACT_TYPE_MOTOR:
                    if (incremental) {
                        // d(omega^2) = 2 omega d(omega), and d(omegaDot)
                        A[m] = 1e-5f * 2.f * learnRun.fxOmega[act] * learnRun.fxOmegaDiff[act];
                        A[m + learnRun.numMotors] = 1e-3f * learnRun.fxOmegaDotDiff[act];
                    } else {
                        A[m] = 1e-5f * learnRun.fxOmega[act] * learnRun.fxOmega[act];
                        A[m + learnRun.numMotors] = 1e-3f * learnRun.fxOmegaDot[act];
                    }
                    m++;
                    break;
                case INDI_ACT_TYPE_SERVO:
                case INDI_ACT_TYPE_OFF:
                default:
                    UNUSED(s);
                    break;
            }
        }
//#endif

        for (int ax = 0; ax < 3; ax++) {
            // first regressor is rate cross terms for inertia ratios
            // A[0] = -learnRun.imuRate.A[ (ax+1)%3 ] * learnRun.imuRate.A[ (ax+2)%3 ];
            A[0] = 0.f;

// #define LEARNER_IS_TAILSITTER_SYMMETRIC

#ifdef LEARNER_IS_TAILSITTER
            float w20 = learnRun.fxOmega[0] * learnRun.fxOmega[0];
            float w21 = learnRun.fxOmega[1] * learnRun.fxOmega[1];
            float w2d0 = w20 * (learnRun.fxOmega[2]);
            float w2d1 = w21 * (learnRun.fxOmega[3]);
            float wdot0 = learnRun.fxOmegaDot[0];
            float wdot1 = learnRun.fxOmegaDot[1];
            float d0Dot = learnRun.fxOmegaDot[2];
            float d1Dot = learnRun.fxOmegaDot[3];
            float d0DotDot = learnRun.fxOmegaDotDot[2];
            float d1DotDot = learnRun.fxOmegaDotDot[3];
            float u = 0.f;
            float v = 0.f;
            float w = 0.f;
            float p = learnRun.imuRate.A[0];
            float q = learnRun.imuRate.A[1];
            float r = learnRun.imuRate.A[2];
            float eta = sqrtf( (u*u+v*v+w*w) + 1.f * (p*p+q*q+r*r));
#ifdef LEARNER_IS_TAILSITTER_SYMMETRIC
            switch(ax){
                case 0: // roll
                    A[1] = 1e-5f * (w20 - w21); // motor 0 is left, motor 1 is right
                    A[2] = 0.f;
                    A[3] = 0.f;
                    A[4] = 1e-1f * eta * p;
                    break;
                case 1: // pitch
                    A[1] = 1e-5 * (w20 + w21); // motor 0 is left, motor 1 is right
                    A[2] = 1e-5 * (w2d0 + w2d1);
                    A[3] = 0.f;
                    A[4] = 1e-1f * eta * q;
                    break;
                case 2: // yaw
                    A[1] = 1e-5f * (w20 - w21); // motor 0 is left, motor 1 is right
                    A[2] = 1e-5f * (w2d0 - w2d1);
                    A[3] = 1e-3f * (wdot0 - wdot1);
                    A[4] = 1e-1f * eta * r;
                    break;
            }
#else
            switch(ax){
                case 0: // roll
                    A[1] = 1e-5f * (w20); // motor 0 is left
                    A[2] = 1e-5f * (w21);
                    A[3] = 0.f * 1e-5f * (w2d0);
                    A[4] = 0.f * 1e-5f * (w2d1);
                    A[5] = 0.f * 1e-0f * (d0Dot);
                    A[6] = 0.f * 1e-0f * (d1Dot);
                    A[7] = -1e-1f * eta * p;
                    A[8] = 0.f * 1e-2f * (d0DotDot);
                    A[9] = 0.f * 1e-2f * (d1DotDot);
                    break;
                case 1: // pitch
                    A[1] = 1e-5f * (w20); // motor 0 is left
                    A[2] = 1e-5f * (w21);
                    A[3] = 1e-5f * (w2d0);
                    A[4] = 1e-5f * (w2d1);
                    A[5] = 0.f * 1e-0f * (d0Dot);
                    A[6] = 0.f * 1e-0f * (d1Dot);
                    A[7] = -1e-1f * eta * q;
                    A[8] = 0.f * 1e-2f * (d0DotDot);
                    A[9] = 0.f * 1e-2f * (d1DotDot);
                    break;
                case 2: // yaw
                    A[1] = 1e-5f * (w20); // motor 0 is left
                    A[2] = 1e-5f * (w21);
                    A[3] = 1e-5f * (w2d0);
                    A[4] = 1e-5f * (w2d1);
                    A[5] = 0.f * 1e-3f * (wdot0);
                    A[6] = 0.f * 1e-3f * (wdot1);
                    A[7] = -1e-1f * eta * r;
                    break;
            }
#endif // LEARNER_IS_TAILSITTER_SYMMETRIC
#endif

            if (incremental) {
                ySpf[ax] = learnRun.fxSpfDiff.A[ax] * 10.f;
                yRateDot[ax] = learnRun.fxRateDotDiff.A[ax];
            } else {
                ySpf[ax] = learnRun.fxSpf.A[ax] * 10.f; // scaling likely depends on sample time..
                yRateDot[ax] = learnRun.fxRateDot.A[ax]; // scaling seems okay at this sample time/filtering
            }

            float lambda = learnerSampleLambda(probing);
            rlsNewSample(&fxRls[ax], A+1, &ySpf[ax], lambda); // spf (skip inertia term)
            rlsNewSample(&fxRls[ax+3], A, &yRateDot[ax], lambda); // RateDot (include inertia term)
        }

        // parallel alternative: perform rls step
        //rlsParallelNewSample(&fxSpfRls, A, ySpf);
        //rlsParallelNewSample(&fxRateDotRls, A, yRateDot);
    }

    learnerTimings.fx = cmpTimeUs(micros(), learnerTimings.start);

    if (learnAct) {
        float A[4]; // 4 regressors per actuator
        float y;
        // simple:  omega - idle = a*d                 - tau*omegaDot  (sqrt(d) and constant terms disabled)
        // full:    omega        = a*d + b*sqrt(d) + c - tau*omegaDot
        const bool simple = config->simpleMotorModel;
        const float idleOmega = learnerIdleOmega();

        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act))) {
                continue; // skip unselected actuators
            }

            switch (indiRun.actType[act]) {
                case INDI_ACT_TYPE_MOTOR:
                    A[0] = learnRun.motorD[act];
                    A[1] = simple ? 0.f : learnRun.motorSqrtD[act];
                    A[2] = simple ? 0.f : 1.f;
                    A[3] = -1e-4f * learnRun.motorOmegaDot[act];
                    y = (learnRun.motorOmega[act] - idleOmega) * 1e-3f; // get into range of 1
                    break;
                case INDI_ACT_TYPE_SERVO:
                    // now done before throw. see updateServoProber()
                    continue;
                    // A[0] = learnRun.motorD[act];
                    // A[1] = 0.f;
                    // A[2] = 0.f;
                    // A[3] = -1e-1f * learnRun.motorOmegaDot[act];
                    // y = 1e1f * learnRun.motorOmega[act]; // already in radians
                    // break;
                case INDI_ACT_TYPE_OFF:
                default:
                    continue;
            }

            rlsNewSample( &actRls[act], A, &y, learnerFixedLambda() );
            if (simple) {
                actRls[act].x[1] = 0.f;
                actRls[act].x[2] = 0.f;
            }
        }
    }

    learnerTimings.motor = cmpTimeUs(micros(), learnerTimings.start);

    bool gainTuningConditions = learnAct;

    if (gainTuningConditions) {
        // get slowest actuator
        float maxTau = 0.f;
        float maxTauMotors = 0.f;
        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act))) {
                continue; // skip unselected actuators
            }

            if (indiRun.actType[act] == INDI_ACT_TYPE_OFF) {
                continue; // skip unsupported actuator types
            }

            if (indiRun.actType[act] == INDI_ACT_TYPE_SERVO) {
                maxTau = MAX(maxTau, (actRls[act].x[3]+actRls[act].x[2]) * 0.1f);
            } else {
                maxTau = MAX(maxTau, (actRls[act].x[3]) * 0.1f);
                maxTauMotors = MAX(maxTauMotors, (actRls[act].x[3]) * 0.1f);
            }

        }

        maxTau = constrainf(maxTau, 0.01f, 0.2f);
        maxTauMotors = constrainf(maxTauMotors, 0.01f, 0.2f);

        if (indiRun.useGainScheduling) {
            // rate and attitude gains come from the schedule (updateLearnedParameters),
            // still set velocity and position gains from the cascade
            float tempGain = 0.25f / (sq(learnRun.zeta[LEARNER_LOOP_HORIZONTAL_RATE]) * maxTau);
            for (int loop = LEARNER_LOOP_HORIZONTAL_ATTITUDE; loop <= LEARNER_LOOP_HORIZONTAL_POSITION; loop++) {
                tempGain = 0.25f * tempGain / sq(learnRun.zeta[loop]);
                if (loop > LEARNER_LOOP_HORIZONTAL_ATTITUDE) {
                    learnRun.gains[loop] = tempGain;
                }
            }
        } else {
            // calculate horizontal gains
            learnRun.gains[LEARNER_LOOP_HORIZONTAL_RATE] = 
                0.25f / (sq(learnRun.zeta[LEARNER_LOOP_HORIZONTAL_RATE]) * maxTau);

            for (int loop = LEARNER_LOOP_HORIZONTAL_ATTITUDE; loop <= LEARNER_LOOP_HORIZONTAL_POSITION; loop++) {
                learnRun.gains[loop] = 0.25f * learnRun.gains[loop-1] / sq(learnRun.zeta[loop]);
            }
        }
        // modular_drone: log the feedforward filter coefficients in place of the pre-feedforward setpoint
        indiRun.attSpNedPreFeedforward.w = indiRun.feedforwardFilterCoefs[0];
        indiRun.attSpNedPreFeedforward.x = indiRun.feedforwardFilterCoefs[1];
        indiRun.attSpNedPreFeedforward.y = indiRun.feedforwardFilterCoefs[2];

        // calculate vertical gains
        learnRun.gains[LEARNER_LOOP_VERTICAL_VELOCITY] =
            0.25f / (sq(learnRun.zeta[LEARNER_LOOP_VERTICAL_VELOCITY]) * maxTauMotors);

        learnRun.gains[LEARNER_LOOP_VERTICAL_POSITION] =
            0.25f * learnRun.gains[LEARNER_LOOP_VERTICAL_VELOCITY] / sq(learnRun.zeta[LEARNER_LOOP_VERTICAL_POSITION]);
    }
    learnerTimings.gains = cmpTimeUs(micros(), learnerTimings.start);

    updateLearnedParameters(indiProfileLearned, positionProfileLearned);
    learnerTimings.updating = cmpTimeUs(micros(), learnerTimings.start);


    static bool hoverAttitudeSuccess = false;
    UNUSED(hoverAttitudeSuccess);
    if (learnHover) {
        hoverAttitudeSuccess = calculateHoverAttitude();
    }

    learnerTimings.hover = cmpTimeUs(micros(), learnerTimings.start);

    // Apply the learned profiles. While learning, the learned INDI profile is
    // made active early (learner_apply_indi_profile) and only its parameters
    // are refreshed every loop. The stateful filters (actuator and rpm models,
    // feedforward, lead-lag) are re-tuned once, when the learning stops: at the
    // end of the query, or when in-flight learning ends. They keep their state
    // then (initIndiRuntime while armed), so control does not see a transient.
    learnerApplyProfiles(learnFx || learnAct || learnHover);

#ifdef USE_CLI_DEBUG_PRINT
    static unsigned int printCounter = 0;
    if (!(++printCounter % 1000))
        cliPrintLinef("Learner Timings (us): filt %d, imu %d, fx %d, mot %d, gain %d, update %d, hover %d", 
                    learnerTimings.filters,
                    learnerTimings.imu,
                    learnerTimings.fx,
                    learnerTimings.motor,
                    learnerTimings.gains,
                    learnerTimings.updating,
                    learnerTimings.hover);
#endif
}

// modular_drone: rate and attitude gains from the gain schedule (indi_use_gain_scheduling),
// as a function of the slowest motor time constant and the least control effectiveness
static void updateScheduledGains(indiProfile_t* indi) {
    const learnerConfig_t* config = learnerConfig();

    // TODO: Put this somewhere more logical
    float kEtaMin = 1.0f;
    float kOmegaMin = 1.0f;

    float kFf = 0.0f;
    float pFf = 0.0f;
    float zFf = 0.0f;

    // get slowest motor
    float maxTau = 0.0f;
    for (int act = 0; act < indiRun.actNum; act++) {
        if ((config->actMask & (1 << act)) && (indiRun.actType[act] == INDI_ACT_TYPE_MOTOR)) {
            maxTau = MAX(maxTau, actRls[act].x[3] * 0.1f);
        }
    }
    maxTau = constrainf(maxTau, 0.01f, 0.2f);

    for (int axis = 0; axis < 3; axis++) {
        // Find minimal control coefficient for each axis
        float minC = INFINITY;
        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act)) || (indiRun.actType[act] != INDI_ACT_TYPE_MOTOR)) {
                continue;
            }
            switch (axis) {
                case FD_ROLL: minC = MIN(minC, abs(indi->actG1_roll[act])); break;
                case FD_PITCH: minC = MIN(minC, abs(indi->actG1_pitch[act])); break;
                case FD_YAW: minC = MIN(minC, abs(indi->actG1_yaw[act])); break;
            }
        }

        minC = minC * 0.1f; // Convert C unit
        switch (indiRun.gainSchedulingType) {
            case GAIN_SCHEDULE_1D_INTERP:
                indi->rateGains[axis] = (uint16_t)(10.0f * interpolate1D(kOmega1D.xAxis, kOmega1D.table, kOmega1D.xSize, maxTau));
                indi->attGains[axis] = (uint16_t)(interpolate1D(kEta1D.xAxis, kEta1D.table, kEta1D.xSize, maxTau) * indi->rateGains[axis]);

                if (indiRun.gainScheduleFf) {
                    kFf = interpolate1D(ffK1D.xAxis, ffK1D.table, ffK1D.xSize, maxTau);
                    pFf = interpolate1D(ffPole1D.xAxis, ffPole1D.table, ffPole1D.xSize, maxTau);
                    zFf = pFf / kFf; // Based on 0 dB gain at LF
                    indi->feedforwardFilterCoefs[0] = kFf * 1e6f;
                    indi->feedforwardFilterCoefs[1] = zFf * 1e6f;
                    indi->feedforwardFilterCoefs[2] = pFf * 1e6f;
                    indiRun.feedforwardFilterCoefs[0] = kFf;
                    indiRun.feedforwardFilterCoefs[1] = zFf;
                    indiRun.feedforwardFilterCoefs[2] = pFf;
                }
                break;
            case GAIN_SCHEDULE_2D_INTERP:
                indi->rateGains[axis] = (uint16_t)(10.0f * interpolate2D(kOmega2D.xAxis, kOmega2D.xSize, kOmega2D.yAxis, kOmega2D.ySize,
                                                                        (const float**)kOmega2D.table, maxTau, minC));
                indi->attGains[axis] = (uint16_t)(interpolate2D(kEta2D.xAxis, kEta2D.xSize, kEta2D.yAxis, kEta2D.ySize,
                                                                       (const float**)kEta2D.table, maxTau, minC) * indi->rateGains[axis]);
                if (indiRun.gainScheduleFf) {
                    kFf = interpolate2D(ffK2D.xAxis, ffK2D.xSize, ffK2D.yAxis, ffK2D.ySize,
                                             (const float**)ffK2D.table, maxTau, minC);
                    pFf = -interpolate2D(ffPole2D.xAxis, ffPole2D.xSize, ffPole2D.yAxis, ffPole2D.ySize,
                                             (const float**)ffPole2D.table, maxTau, minC);
                    zFf = 0.f;
                }
                break;
            case GAIN_SCHEDULE_1D_POLY:
                indi->rateGains[axis] = (uint16_t)(10.0f * evalPoly1D(&kOmegaPoly1D, maxTau));
                indi->attGains[axis] = (uint16_t)(evalPoly1D(&kEtaPoly1D, maxTau) * indi->rateGains[axis]);
                if (indiRun.gainScheduleFf) {
                    kFf = evalPoly1D(&ffKPoly1D, maxTau);
                    pFf = -evalPoly1D(&ffPolePoly1D, maxTau); // ffPolePoly is defined as the polynomial through -p
                    zFf = indiRun.useFeedforwardFilter ? zFf : 0.f; // zFf is 0 here
                }
                break;
            case GAIN_SCHEDULE_2D_POLY:
                indi->rateGains[axis] = (uint16_t)(10.0f * evalPoly2D(&kOmegaPoly2D, maxTau, minC));
                indi->attGains[axis] = (uint16_t)(evalPoly2D(&kEtaPoly2D, maxTau, minC) * indi->rateGains[axis]);
                if (indiRun.gainScheduleFf) {
                    kFf = evalPoly2D(&ffKPoly2D, maxTau, minC);
                    pFf = -evalPoly2D(&ffPolePoly2D, maxTau, minC);
                    zFf = 0.f;
                }
                break;
        }
        if (indiRun.gainScheduleFf && (axis == FD_ROLL)) {
            // the feedforward filter itself is re-built from these when the
            // learning stops (initIndiRuntime), not every loop: that reset its state.
            // One coefficient set for roll and pitch (the 2D schedules used to
            // build per-axis filters from the per-axis minC; roll's is kept)
            indi->feedforwardFilterCoefs[0] = kFf * 1e6f;
            indi->feedforwardFilterCoefs[1] = zFf * 1e6f;
            indi->feedforwardFilterCoefs[2] = pFf * 1e6f;
            indiRun.feedforwardFilterCoefs[0] = kFf;
            indiRun.feedforwardFilterCoefs[1] = zFf;
            indiRun.feedforwardFilterCoefs[2] = pFf;
        }

        // Ensure gains are above minimum
        indi->rateGains[axis] = MAX(indi->rateGains[axis], (uint16_t)(10.0f * kOmegaMin));
        indi->attGains[axis] = MAX(indi->attGains[axis], (uint16_t)(kEtaMin * indi->rateGains[axis]));
        learnRun.gains[LEARNER_LOOP_HORIZONTAL_RATE] = indi->rateGains[axis] / 10.0f;
        learnRun.gains[LEARNER_LOOP_HORIZONTAL_ATTITUDE] = (float)indi->attGains[axis] / (float)indi->rateGains[axis];
    }
}

void updateLearnedParameters(indiProfile_t* indi, positionProfile_t* pos) {
    const learnerConfig_t* config = learnerConfig(); // fix this line

    if (indiRun.useGainScheduling) {
        updateScheduledGains(indi);
    } else {
        for (int axis = 0; axis < 3; axis++) {
            indi->rateGains[axis] = (uint16_t)
                constrainf(10.f * learnRun.gains[LEARNER_LOOP_HORIZONTAL_RATE],
                    1.f, (1 << 16) - 1.f);
            // attGains are expected for parallel PD, but we have cascaded, so
            indi->attGains[axis]  = (uint16_t) 
                constrainf(10.f * learnRun.gains[LEARNER_LOOP_HORIZONTAL_ATTITUDE] * learnRun.gains[LEARNER_LOOP_HORIZONTAL_RATE],
                    1.f, (1 << 16) - 1.f);
        }

#ifdef LEARNER_IS_TAILSITTER
        // roll boost (cbe69f73b)
        indi->rateGains[FD_ROLL] = constrainu(indi->rateGains[FD_ROLL] * 2, 1, (1 << 16) - 1);
        indi->attGains[FD_ROLL] = constrainu(indi->attGains[FD_ROLL] * 2, 1, (1 << 16) - 1);
#endif
    }

    // same for position
    pos->horz_p = (uint8_t) 
        constrainf(10.f * learnRun.gains[LEARNER_LOOP_HORIZONTAL_POSITION] * learnRun.gains[LEARNER_LOOP_HORIZONTAL_VELOCITY],
            1.f, 255.f);
    pos->horz_d = (uint8_t)
        constrainf(10.f * learnRun.gains[LEARNER_LOOP_HORIZONTAL_VELOCITY],
            1.f, 255.f);
    pos->horz_i = pos->horz_d / 10; // fudge factor: by lack of better option at this point
    if ((config->zetaVelocityVert == 0) || (config->zetaPositionVert == 0)) {
        // modular_drone: vertical gains same as horizontal
        pos->vert_p = pos->horz_p;
        pos->vert_i = pos->horz_i;
        pos->vert_d = pos->horz_d;
    } else {
        pos->vert_p = (uint8_t)
            constrainf(10.f * learnRun.gains[LEARNER_LOOP_VERTICAL_POSITION] * learnRun.gains[LEARNER_LOOP_VERTICAL_VELOCITY],
                1.f, 255.f);
        pos->vert_d = (uint8_t)
            constrainf(10.f * learnRun.gains[LEARNER_LOOP_VERTICAL_VELOCITY],
                1.f, 255.f);
        pos->vert_i = pos->vert_d / 10; // fudge factor: by lack of better option at this point
    }

    // pos->horz_max_v = 250; // cm/s
    // pos->horz_max_a = 500; // cm/s/s
    // pos->horz_max_iterm = 200; // cm/s
    // pos->max_tilt = 40; // conservative, like the others
    // pos->vert_max_v_up = 100; // cm/s
    // pos->vert_max_v_down = 100; // cm/s
    // pos->vert_max_a_up = 1000; // cm/s/s
    // pos->vert_max_a_down = 500; // cm/s/s
#ifdef LEARNER_IS_TAILSITTER
    pos->vert_max_a_up = 2500; // cm/s/s
    pos->vert_max_a_down = 500; // cm/s/s
#endif
    // pos->vert_max_iterm = 100; // cm/s/s
    // fudge factor 0.5f, maybe try to see what happens with lower zeta_attitude
    pos->yaw_p = (uint8_t) 
        constrainf(10.f * .5f * learnRun.gains[LEARNER_LOOP_HORIZONTAL_ATTITUDE],
            1.f, 255.f); // degs per sec * 10
    // pos->weathervane_p = 0;
    // pos->weathervane_min_v = 200; // cm/s/s
    // pos->use_spf_attenuation = 1;

    // indi->manualUseCoordinatedYaw = 1;
    // indi->manualMaxUpwardsSpf = 20; // conservative

    // indi->attMaxTiltRate = 500; // reduce slightly
    // indi->attMaxYawRate = 300; // reduce

    fp_quaternion_t imu_to_hover;
    fp_quaternionProducts_t imu_to_hoverP;
    fp_rotationMatrix_t imu_to_hoverR;
    imu_to_hover = hoverAttitude; // copy, not pointer
    imu_to_hover.w *= -1.; // inverse
    quaternionProducts_of_quaternion(&imu_to_hoverP, &imu_to_hover);
    rotationMatrix_of_quaternionProducts(&imu_to_hoverR, &imu_to_hoverP);

#if defined(LEARNER_IS_TAILSITTER)
    // Tailsitter specific code (see indi_init)
    indi->tails_use_scheduled = true;
    indi->tails_use_sine = false;
    indi->tails_d0[0] = -2100;
    indi->tails_d0[1] = -912;
    indi->tails_cxw[0]  = 405;
    indi->tails_cxw[1]  = 405;
    indi->tails_cyw[0]  = 0;
    indi->tails_cyw[1]  = 0;
    indi->tails_czw[0]  = -2481;
    indi->tails_czw[1]  = -2481;
    indi->tails_clw[0]  = +2597;
    indi->tails_clw[1]  = -2597;
    indi->tails_cmw[0]  = 0;
    indi->tails_cmw[1]  = 0;
    indi->tails_cnw[0]  = -702;
    indi->tails_cnw[1]  = +702;
    indi->tails_cnwd[0] = -260;
    indi->tails_cnwd[1] = +260;
    indi->tails_cxd[0]  = -125;
    indi->tails_cxd[1]  = -125;
    indi->tails_cmd[0]  = -1300;
    indi->tails_cmd[1]  = -1300;
    indi->tails_cnd[0]  = -1122;
    indi->tails_cnd[1]  = +1122;

    indi->tails_cmdd[0] = 0;
    indi->tails_cmdd[1] = 0;
    indi->tails_cndd[0] = 0;
    indi->tails_cndd[1] = 0;

#if defined(LEARNER_IS_TAILSITTER_SYMMETRIC)
    indi->tails_d0[0] = 0;
    indi->tails_d0[1] = 0;
    indi->tails_clw[0]  = (int16_t) (1e8f * fxRls[0+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_clw[1]  = -indi->tails_clw[0];
    indi->tails_cmw[0]  = (int16_t) (1e8f * fxRls[1+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmw[1]  = indi->tails_cmw[0];
    indi->tails_cnw[0]  = (int16_t) (1e8f * fxRls[2+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnw[1]  = -indi->tails_cnw[0];
    indi->tails_cnwd[0] = (int16_t) (1e5f * fxRls[2+3].x[3] * 1e-3f * 1e0f); // 1e-3 for omegadot scaling, 1e0 for y-scaling
    indi->tails_cnwd[1] = -indi->tails_cnwd[0];
    indi->tails_cmd[0] = (int16_t) (1e8f * fxRls[1+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmd[1] = indi->tails_cmd[0];
    indi->tails_cnd[0] = (int16_t) (1e8f * fxRls[2+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnd[1] = -indi->tails_cnd[0];

    UNUSED(config);
    UNUSED(actG2rotIMU);
#else

    indi->tails_cxw[0] = 0;
    indi->tails_cxw[1] = 0;
    indi->tails_cyw[0] = 0;
    indi->tails_cyw[1] = 0;
    indi->tails_czw[0] = (int16_t) (1e9f * fxRls[2].x[0] * 1e-5f * 1e-1f); // 1e-5 for omega^2 scaling, 1e-1 for y-scaling
    indi->tails_czw[1] = (int16_t) (1e9f * fxRls[2].x[1] * 1e-5f * 1e-1f); // 1e-5 for omega^2 scaling, 1e-1 for y-scaling
    indi->tails_cxd[0] = 0;
    indi->tails_cxd[1] = 0;

    indi->tails_d0[0] = 0;
    indi->tails_d0[1] = 0;

    indi->tails_clw[0]  = (int16_t) (1e8f * fxRls[0+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_clw[1]  = (int16_t) (1e8f * fxRls[0+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling

    indi->tails_cmw[0]  = (int16_t) (1e8f * fxRls[1+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmw[1]  = (int16_t) (1e8f * fxRls[1+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmd[0]  = (int16_t) (1e8f * fxRls[1+3].x[3] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmd[1]  = (int16_t) (1e8f * fxRls[1+3].x[4] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmdd[0] = (int16_t) 0;
    indi->tails_cmdd[1] = (int16_t) 0;

    indi->tails_cnw[0]  = (int16_t) (1e8f * fxRls[2+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnw[1]  = (int16_t) (1e8f * fxRls[2+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnd[0]  = (int16_t) (1e8f * fxRls[2+3].x[3] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnd[1]  = (int16_t) (1e8f * fxRls[2+3].x[4] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnwd[0] = (int16_t) 0;
    indi->tails_cnwd[1] = (int16_t) 0;
    indi->tails_cndd[0] = (int16_t) 0;
    indi->tails_cndd[1] = (int16_t) 0;

    UNUSED(config);
    UNUSED(actG2rotIMU);
#endif // LEARNER_IS_TAILSITTER_SYMMETRIC
#else
    indi->actNum = indiRun.actNum;
    int m = 0;
    for (int act = 0; act < indi->actNum; act++) {
        if (!(config->actMask & (1 << act)) || (indiRun.actType[act] != INDI_ACT_TYPE_MOTOR)) {
            continue; // skip unselected actuators and servos (packed motor index m, see initLearnerRls)
        }

        //              inv y-scale 
        // simple model: idle + a*1.  full model: a*1 + b*sqrt(1) (intercept c not included)
        float maxOmega =   1e3f  *  (actRls[act].x[0] + actRls[act].x[1]) + learnerIdleOmega();
        if (config->hardcodeMaxRpm > 0) {
            // learner_hardcode_max_rpm, replaces build flag LEARNER_HARDCODE_MAXRPM
            maxOmega = 2.f * M_PIf / 60.f  *  (float) config->hardcodeMaxRpm;
        }
        indi->actMaxRpm[act] = MAX(100.f, 60.f * 0.5f / M_PIf  *  maxOmega); // convert to deg/s
        indi->actHoverRpm[act] = indi->actMaxRpm[act] >> 1; // guess, shouldnt matter since we have useRpmDotFeedback = true
        //                                            inv y-scale   a-scale     config-scale
        indi->actTimeConstMs[act] = (uint8_t) constrainf(1e3f      *  1e-4f  *    1000.f     * actRls[act].x[3], 10.f, 200.f);

        if ((actRls[act].x[0] > 0.f) && (actRls[act].x[1] > 0.f)) {
            indi->actNonlinearity[act] = (uint8_t) 100.f * constrainf(
                actRls[act].x[0] / (actRls[act].x[0] + actRls[act].x[1]),
                0.f, 1.f);
            // todo: transform to match kappa better
        } else {
            indi->actNonlinearity[act] = 50;
        }

        actG1linIMU[m].V.X = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[0].x[m];
        actG1linIMU[m].V.Y = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[1].x[m];
        actG1linIMU[m].V.Z = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[2].x[m];
        actG1rotIMU[m].V.X = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[3].x[1 + m];
        actG1rotIMU[m].V.Y = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[4].x[1 + m];
        actG1rotIMU[m].V.Z = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[5].x[1 + m];
        actG2rotIMU[m].V.X = 1.f      * 1e-3f                *     1e5f     * fxRls[3].x[1 + learnRun.numMotors + m];
        actG2rotIMU[m].V.Y = 1.f      * 1e-3f                *     1e5f     * fxRls[4].x[1 + learnRun.numMotors + m];
        actG2rotIMU[m].V.Z = 1.f      * 1e-3f                *     1e5f     * fxRls[5].x[1 + learnRun.numMotors + m];

        fp_vector_t actG1linHover, actG1rotHover, actG2rotHover;

        actG1linHover = actG1linIMU[m];
        rotate_vector_with_rotationMatrix(&actG1linHover, &imu_to_hoverR);

        actG1rotHover = actG1rotIMU[m];
        rotate_vector_with_rotationMatrix(&actG1rotHover, &imu_to_hoverR);

        actG2rotHover = actG2rotIMU[m];
        rotate_vector_with_rotationMatrix(&actG2rotHover, &imu_to_hoverR);

        if (config->hardcodeLinearForces) {
            // learner_hardcode_linear_forces, replaces build flag LEARNER_HARDCODE_LINEAR_FORCES
            indi->actG1_fx[act]    = config->hardcodeG1Fx;
            indi->actG1_fy[act]    = config->hardcodeG1Fy;
            indi->actG1_fz[act]    = config->hardcodeG1Fz;
        } else {
            indi->actG1_fx[act]    = actG1linHover.V.X;
            indi->actG1_fy[act]    = actG1linHover.V.Y;
            indi->actG1_fz[act]    = actG1linHover.V.Z;
        }
        indi->actG1_roll[act]  = actG1rotHover.V.X;
        indi->actG1_pitch[act] = actG1rotHover.V.Y;
        indi->actG1_yaw[act]   = actG1rotHover.V.Z;
        indi->actG2_roll[act]  = actG2rotHover.V.X;
        indi->actG2_pitch[act] = actG2rotHover.V.Y;
        indi->actG2_yaw[act]   = actG2rotHover.V.Z;

        indi->wlsWu[act] = 1.;
        indi->u_pref[act] = 0;

        m++;
    }
#endif // LEARNER_IS_TAILSITTER

    // indi->imuSyncLp2Hz = 15; // lord knows
    // indi->wlsWv[0] = 1;
    // indi->wlsWv[1] = 1;
    // indi->wlsWv[2] = 50;
    // indi->wlsWv[3] = 50;
    // indi->wlsWv[4] = 50;
    // indi->wlsWv[5] = 5;

    // keep same:
    // indi->useIncrement = true;
    // indi->useRpmDotFeedback = true;
    // indi->useConstantG2 = false;
    // indi->useRpmFeedback = false;
    // indi->maxRateSp = 1800.f for all
    // indi->useWls = true;
    // indi->wlsMaxIter = 1;
    // indi->wlsAlgo = AS_QR;
    // indi->wlsCondBound = ...;
    // indi->wlsNanLimit = 10;

}

void testLearner(void) {
    rlsTest();
    learnerConfigMutable()->actMask = 0x003F; // all 6 motors

    float G1Tmp[6][6] = {
        {-0.83072608, -0.83072608, -0.83072608, -0.83072608, -0.83072608, -0.83072608},
        {-0.55116244, -0.55116244, -0.55116244, -0.55116244, -0.55116244, -0.55116244},
        {0.07819308,  0.07819308,  0.07819308,  0.07819308,  0.07819308, 0.07819308},
        {1.05405066, -0.07579926, -1.5856529 ,  0.60740149,  1.21480297, 1.82220446},
        {0.03071845, -1.61020823,  0.50788336,  1.07160642,  2.14321284, 3.21481927},
        {-1.37405734,  0.63362759, -0.47724144,  1.21767118,  2.43534237, 3.65301355}
    };
    for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++)
            indiRun.actG1[row][col] = G1Tmp[row][col];

    updateLearner(0);
    updateLearner(0);
    updateLearner(0);
    // expecting thrustVector = -8.14942279, -5.4069035 ,  0.76707411
}

// query
// --- query state machine
//
//   IDLE --(LEARNER_MODE and a probing trigger, see enableConditions)--> WAITING_FOR_LAUNCH
//   WAITING_FOR_LAUNCH --(catapult done / armed after throw / in flight)--> DELAY
//   DELAY/ACTIVE --(prober finished or timed out)--> DONE
//   any --(LEARNER_MODE off)--> IDLE
//
// DONE is final: a new query needs LEARNER_MODE to be switched off and on again.
//
// Board misalignment (learner_*_misalignment, learner_randomize_misalignment,
// for testing in simulation) is applied once, after the catapult launch and
// before the query starts, and kept for the rest of the flight, so the learner
// has to resolve it. It is undone when LEARNER_MODE is switched off, or when
// the catapult sequence restarts.

#include "flight/learning_prober.h"

static bool learnerMisaligned = false;

static bool learnerMisalignmentRequested(void) {
    const learnerConfig_t *c = learnerConfig();
    return (c->rollMisalignment != 0) || (c->pitchMisalignment != 0) || (c->yawMisalignment != 0);
}

static void learnerOverrideAttitude(const fp_quaternion_t *newAttitude) {
    overrideAttitudeQuaternion((fp_quaternion_t *) newAttitude); // updates quat, eulers, rotation matrix and quat products
#ifdef USE_EKF
    float *theEkfX = ekf_get_X();
    theEkfX[6] = newAttitude->w;
    theEkfX[7] = newAttitude->x;
    theEkfX[8] = newAttitude->y;
    theEkfX[9] = newAttitude->z; // this probably messes up covariances, who cares
#endif
}

// misalign the board after catapulting with 0 0 0 board alignment. A non-zero
// configured board alignment could not be restored correctly, so then skip.
static void learnerApplyMisalignment(void) {
    if (learnerMisaligned || !learnerMisalignmentRequested()) {
        return;
    }
    if ((boardAlignment()->rollDegrees != 0) || (boardAlignment()->pitchDegrees != 0) || (boardAlignment()->yawDegrees != 0)) {
        return;
    }

    const learnerConfig_t *c = learnerConfig();
    fp_euler_t boardEulers_fp = { 0 };
    if (c->randomizeMisalignment) {
        // Forward Right Down
        boardEulers_fp.angles.roll  = DEGREES_TO_RADIANS(ABS(c->rollMisalignment)) * rngFloat();
        boardEulers_fp.angles.pitch = DEGREES_TO_RADIANS(ABS(c->pitchMisalignment)) * rngFloat();
        boardEulers_fp.angles.yaw   = DEGREES_TO_RADIANS(ABS(c->yawMisalignment)) * rngFloat();
    } else {
        boardEulers_fp.angles.roll  = DEGREES_TO_RADIANS(c->rollMisalignment);
        boardEulers_fp.angles.pitch = DEGREES_TO_RADIANS(c->pitchMisalignment);
        boardEulers_fp.angles.yaw   = DEGREES_TO_RADIANS(c->yawMisalignment);
    }

    i16_euler_t boardEulers_i16;
    i16_euler_of_fp_euler(&boardEulers_i16, &boardEulers_fp);
    boardAlignmentMutable()->rollDegrees  = boardEulers_i16.angles.roll / 10;
    boardAlignmentMutable()->pitchDegrees = boardEulers_i16.angles.pitch / 10;
    boardAlignmentMutable()->yawDegrees   = boardEulers_i16.angles.yaw / 10;
    initBoardAlignment(boardAlignment());

    fp_quaternion_t iboard_q;
    quaternion_of_rotationMatrix(&iboard_q, &boardRotation);
    iboard_q.w = -iboard_q.w;

    fp_quaternion_t attitude; // FRD
    getAttitudeQuaternion(&attitude);
    fp_quaternion_t newAttitude = chain_quaternion(&attitude, &iboard_q);
    learnerOverrideAttitude(&newAttitude);

    learnerMisaligned = true;
}

// undo the misalignment (if any) and the learned hover rotation
static void learnerResetFrames(void) {
    if (learnerMisaligned) {
        // use current board orientation to fix attitude
        fp_quaternion_t board_q;
        quaternion_of_rotationMatrix(&board_q, &boardRotation);

        fp_quaternion_t attitude; // FRD
        getAttitudeQuaternion(&attitude);
        fp_quaternion_t newAttitude = chain_quaternion(&board_q, &attitude);
        learnerOverrideAttitude(&newAttitude);

        // reset board rotation to 0
        boardAlignmentMutable()->rollDegrees  = 0;
        boardAlignmentMutable()->pitchDegrees = 0;
        boardAlignmentMutable()->yawDegrees   = 0;
        initBoardAlignment(boardAlignment());

        learnerMisaligned = false;
    }

    // reset hover rotation
    hoverAttitude.w = 1.;
    hoverAttitude.x = 0.;
    hoverAttitude.y = 0.;
    hoverAttitude.z = 0.;
}

void runLearningQueryStateMachine(timeUs_t current) {
    for (int i = 0; i < MAXU; i++) {
        outputFromLearningQuery[i] = 0.f;
    }

    const learnerConfig_t* config = learnerConfig();

    bool idle_before_throw_query_requested =
        FLIGHT_MODE(LEARNER_MODE)
        && (config->modeProbing & LEARN_PROBING_AFTER_THROW)
        && throwConfig()->idleBeforeThrow
        && ARMING_FLAG(ARMED)
        && throwState == THROW_STATE_WAITING_FOR_THROW;

    // must not accidentally drop into inflight query when idle_before_throw is requested
    bool inflight_query_requested =
        !idle_before_throw_query_requested
        && FLIGHT_MODE(LEARNER_MODE)
        && (config->modeProbing & LEARN_PROBING_DURING_FLIGHT)
        && ARMING_FLAG(ARMED)
        && !isTouchingGround();

    bool post_launch_query_requested =
        !idle_before_throw_query_requested
        && !inflight_query_requested
        && FLIGHT_MODE(LEARNER_MODE)
        && (config->modeProbing & (LEARN_PROBING_AFTER_THROW | LEARN_PROBING_AFTER_CATAPULT))
        && !ARMING_FLAG(ARMED);

    bool disableConditions = !FLIGHT_MODE(LEARNER_MODE);

    // LEARNER_MODE switched off: abort or forget the query, undo misalignment
    // and hover rotation. This is the only way back to IDLE.
    static bool wasEnabled = false;
    if (disableConditions) {
        if (wasEnabled) {
            learnerResetFrames();
            resetProber();
        }
        learningQueryState = LEARNING_QUERY_IDLE;
    }
    wasEnabled = !disableConditions;

#ifdef USE_CATAPULT
    // catapult sequence restarted (disarmed and catapult mode cycled)
    if (learnerMisaligned && (catapultState < CATAPULT_DELAY)) {
        learnerResetFrames();
    }
#endif

    bool enableConditions = !disableConditions
        && ( idle_before_throw_query_requested
                || inflight_query_requested
                || post_launch_query_requested
            );

    learnRun.mixControl = false;

doMore:
    switch (learningQueryState) {
        case LEARNING_QUERY_IDLE:
            if (enableConditions) {
                if (config->applyHoverRotation) {
                    // reset accelerometer trims
                    accelerometerConfigMutable()->accZero.raw[0] = 0;
                    accelerometerConfigMutable()->accZero.raw[1] = 0;
                    accelerometerConfigMutable()->accZero.raw[2] = 0;
                    accelerometerConfigMutable()->accZero.raw[3] = 1;
                    setAccelerationTrims(&accelerometerConfigMutable()->accZero); // probably not necessary because of the horrific pointer magic
                }

                initLearnerRls(false); // reset all RLS filters to 0 initial state, and reset lowpass filters
                resetProber();
                initServoProber(current);
                learningQueryState = LEARNING_QUERY_WAITING_FOR_LAUNCH; goto doMore;
            }
            break;
        case LEARNING_QUERY_WAITING_FOR_LAUNCH: { // catapult or throw
            bool catapultLaunched = false;
#ifdef USE_CATAPULT
            catapultLaunched = (config->modeProbing & LEARN_PROBING_AFTER_CATAPULT) && (catapultState == CATAPULT_DONE);
#endif
            if (catapultLaunched) {
                learnerApplyMisalignment();
            }

            // cue servo identification
            if (!inflight_query_requested) {
                updateServoProber(current);
                // update output from prober
                for (int act = 0; act < indiRun.actNum; act++) {
                    if ((config->actMask & (1 << act))
                        && (indiRun.actType[act] == INDI_ACT_TYPE_SERVO)) {

                        if (proberRuntime.isGenRunningServo) {
                            outputFromLearningQuery[act] = constrainf(proberRuntime.output[act], -1.f, 1.f);
                        } else {
                            outputFromLearningQuery[act] = 0.f;
                        }
                    }
                }
            }

            // considered launched if succesfully activated inflight_query, or catapult/throw states correct
            bool launched = inflight_query_requested
                || catapultLaunched
                || ((config->modeProbing & LEARN_PROBING_AFTER_THROW) && (throwState == THROW_STATE_ARMED_AFTER_THROW));

            if (launched) {
                learningQueryEnabledAt = current;
                initProber(current);
                learningQueryState = LEARNING_QUERY_DELAY; goto doMore;
            }

            break;
        }
        case LEARNING_QUERY_DELAY: // skip this now
        case LEARNING_QUERY_ACTIVE:
            // cycles through motors
            updateProber(current);

            if (cmpTimeUs(current, proberRuntime.genStartTimeUs) > 1e3 * config->mixControlAfterMs) {
                learnRun.mixControl = true;
            }

            if (proberRuntime.isFinished) {
                // all motors done, or timeout reached
                learningQueryState = LEARNING_QUERY_DONE; goto doMore;
            } else {
                // update output from prober
                for (int act = 0; act < indiRun.actNum; act++) {
                    if (!(config->actMask & (1 << act))) {
                        continue; // skip unselected actuators
                    }

                    float output = proberRuntime.output[act];

                    switch(indiRun.actType[act]) {
                        case INDI_ACT_TYPE_MOTOR:
                            outputFromLearningQuery[act] = constrainf(output, 0.f, 1.f);
                            break;
                        case INDI_ACT_TYPE_SERVO:
                            outputFromLearningQuery[act] = constrainf(output, -1.f, 1.f);
                            break;
                        case INDI_ACT_TYPE_OFF:
                        default:
                            outputFromLearningQuery[act] = 0.f;
                            break;
                    }
                }
            }

            break;
        case LEARNING_QUERY_DONE:
            // final until LEARNER_MODE is switched off (see above)
            break;
    }
}

// true while the query drives the actuators and INDI is bypassed. While the
// catapult launches, the catapult has precedence (the query waits for it).
bool learnerQueryIsActive(void) {
    if (!FLIGHT_MODE(LEARNER_MODE)
            || (learningQueryState < LEARNING_QUERY_WAITING_FOR_LAUNCH)
            || (learningQueryState >= LEARNING_QUERY_DONE)) {
        return false;
    }
#ifdef USE_CATAPULT
    if (FLIGHT_MODE(CATAPULT_MODE) && (catapultState > CATAPULT_IDLE) && (catapultState < CATAPULT_DONE)) {
        return false;
    }
#endif
    return true;
}

bool learnerQueryActuatorSelected(int i) {
    return (i < indiRun.actNum) && (proberConfig()->actMask & (1 << i));
}

#endif
