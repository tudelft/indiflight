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

PG_REGISTER_WITH_RESET_TEMPLATE(learnerConfig_t, learnerConfig, PG_LEARNER_CONFIG, 4);
PG_RESET_TEMPLATE(learnerConfig_t, learnerConfig, 
    .modeProbing = (uint8_t) (LEARN_PROBING_AFTER_CATAPULT | LEARN_PROBING_AFTER_THROW | LEARN_PROBING_DURING_FLIGHT),
    // .modeFx    = (uint8_t) (LEARN_DURING_PROBING | LEARN_DURING_FLIGHT),
    // .modeAct   = (uint8_t) (LEARN_DURING_PROBING | LEARN_DURING_FLIGHT),
    // .modeHover = (uint8_t) (LEARN_DURING_PROBING | LEARN_DURING_FLIGHT),
    .modeFx    = (uint8_t) (LEARN_DURING_PROBING | LEARN_DURING_FLIGHT),
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
    .rollMisalignment = 0,
    .pitchMisalignment = 0,
    .yawMisalignment = 0,
    .randomizeMisalignment = false,
    .applyIndi = false,
    .applyPosition = false,
    .applyHoverRotation = false
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

    learnRun.zeta[LEARNER_LOOP_RATE]     = constrainf(0.01f * learnerConfig()->zetaRate    , 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_ATTITUDE] = constrainf(0.01f * learnerConfig()->zetaAttitude, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_VELOCITY] = constrainf(0.01f * learnerConfig()->zetaVelocity, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_POSITION] = constrainf(0.01f * learnerConfig()->zetaPosition, 0.5f, 1.0f);

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

#ifdef STM32H7
FAST_CODE
#endif
static void initLearnerRls(void) {
    if (!learnRun.filtersInitialized) {
        learnRun.initialized = false;
        return; 
    }

    const indiProfile_t *p = indiProfiles(systemConfig()->indiProfileIndex);
    const learnerConfig_t *config = learnerConfig();

    float actionBandwidthHz = 0.001f * ( 1. / (2.f * M_PIf * 0.015f) ); // 5 times slower than assumed fastest actuator
    rlsInit(&imuRls, 3, 3, 1e2f, gyro.targetLooptime, actionBandwidthHz, config->useFortescue);

    // init filters and other rls
    //rlsParallelInit(&fxSpfRls, learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // forces
    //rlsParallelInit(&fxRateDotRls, 2*learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // rotations need twice the parameters

    // Spf
    for (int i = 0; i < 3; i++) {
        rlsInit(&fxRls[i], learnRun.numActuators, 1, 1e-4f, gyro.targetLooptime, actionBandwidthHz, config->useFortescue);
    }

    // RateDot
    for (int i = 3; i < 6; i++) {
        rlsInit(&fxRls[i], 1 + 2*learnRun.numActuators, 1, 1e-2f, gyro.targetLooptime, actionBandwidthHz, config->useFortescue);
    }

    // princ. inertia ratios set to zero: all princ. inertias are the same
    fxRls[3].x[0] = 0.f;
    fxRls[4].x[0] = 0.f;
    fxRls[5].x[0] = 0.f;

    for (int act = 0; act < indiRun.actNum; act++) {
        if (!(config->actMask & (1 << act))) {
            continue;
        }

        switch(indiRun.actType[act]) {
            case INDI_ACT_TYPE_MOTOR:
                rlsInit(&actRls[act], 4, 1, 1e0f, gyro.targetLooptime, actionBandwidthHz, config->useFortescue);

                // inverse of updateLearnedParameters
                float maxOmega = 2.f * M_PIf / 60.f  *  p->actMaxRpm[act];
                float isq = 1.f / sq(maxOmega);

                if (config->initFromProfileAct) {
                    float k = 0.01f * p->actNonlinearity[act];
                    actRls[act].x[0] = 1e-3f * k * maxOmega;
                    actRls[act].x[1] = 1e-3f * (1.f - k) * maxOmega;
                    actRls[act].x[2] = 0.;
                    actRls[act].x[3] = 1e-3f * 1e4f * 1e-3f * (p->actTimeConstMs[act]);
                }

                // todo: rotate with hover rotation
                if (config->initFromProfileFx) {
                    fxRls[0].x[act] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fx[act];
                    fxRls[1].x[act] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fy[act];
                    fxRls[2].x[act] = 10.f * 1e5f * isq * 1e-2f * p->actG1_fz[act];

                    fxRls[3].x[1 + act] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_roll[act];
                    fxRls[4].x[1 + act] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_pitch[act];
                    fxRls[5].x[1 + act] = 1.f  * 1e5f * isq * 1e-1f * p->actG1_yaw[act];

                    fxRls[3].x[1 + learnRun.numMotors + act] = 1.f  * 1e3f * 1e-5f * p->actG2_roll[act];
                    fxRls[4].x[1 + learnRun.numMotors + act] = 1.f  * 1e3f * 1e-5f * p->actG2_pitch[act];
                    fxRls[5].x[1 + learnRun.numMotors + act] = 1.f  * 1e3f * 1e-5f * p->actG2_yaw[act];
                }
                break;
            case INDI_ACT_TYPE_SERVO:
                rlsInit(&actRls[act], 4, 1, 1e0f, gyro.targetLooptime, actionBandwidthHz, config->useFortescue);
                break;
            case INDI_ACT_TYPE_OFF:
            default:
                break; // skip unsupported actuator types
        }
    }


    if (config->initFromProfileAct) {
        learnRun.gains[LEARNER_LOOP_RATE] = 0.1f * p->rateGains[0];
        learnRun.gains[LEARNER_LOOP_ATTITUDE] = ((float) p->attGains[0]) / ((float) p->rateGains[0]);
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

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        learnRun.fxRateDot.A[axis] = biquadFilterApply(&fxRateFilter[axis], indiRun.rateDotIMU.A[axis]);
        learnRun.fxSpf.A[axis] = biquadFilterApply(&fxSpfFilter[axis], fxSpfCorrected[axis]);
    }

    static float fxPrevOmega[MAX_SUPPORTED_MOTORS] = {0};
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
    const uint8_t numAct = learnRun.numActuators;
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

#ifdef STM32H7
FAST_CODE
#endif
void updateLearner(timeUs_t current) {
    UNUSED(current);

    if (!learnRun.initialized) {
        return; // no can do
    }

    learnerTimings.start = micros();

    // update learning sync filters
    updateLearningFilters();
    learnerTimings.filters = cmpTimeUs(micros(), learnerTimings.start);

    // if we are not armed, we do not learn
    if (!ARMING_FLAG(ARMED)
            || !FLIGHT_MODE(LEARNER_MODE)
            || isTouchingGround()
            || learningQueryState == LEARNING_QUERY_WAITING_FOR_LAUNCH) {
        return;
    }

    const learnerConfig_t *config = learnerConfig();

    bool probing = proberRuntime.isGenRunning;

    bool learnFx = (config->modeFx & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeFx & LEARN_DURING_PROBING) && probing);

    bool learnAct = (config->modeAct & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeAct & LEARN_DURING_PROBING) && probing);

    bool learnHover = (config->modeHover & LEARN_DURING_FLIGHT && !proberRuntime.isInitialized)
        || ((config->modeHover & LEARN_DURING_PROBING) && probing);

    // wait for motors to spool down before learning imu position
    bool imuLearningConditions = ((learningQueryState == LEARNING_QUERY_DELAY) 
            && (cmpTimeUs(current, learningQueryEnabledAt) > 100000));

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

        for (int i = 0; i < 9; i++)
            AT[i] *= 1e-2f; // parameters are in the cm range, so make sure they will be around 1 to avoid numerical issues
            // it if true parameter is 30cm is logged at in 0.3*100*1000 = 30000, which is max for logging. 1mm is logged as 0.001*100*1000 = 100

        // perform rls step
        rlsNewSample(&imuRls, AT, y);
    }
    learnerTimings.imu = cmpTimeUs(micros(), learnerTimings.start);

    if (learnFx) {
        // setup regressors
        float A[RLS_MAX_N] = {0};
        float ySpf[3];
        float yRateDot[3];

        int m = 1, s = 1; 
//#ifndef LEARNER_IS_TAILSITTER
        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act))) {
                continue; // skip unselected actuators
            }

            switch (indiRun.actType[act]) {
                case INDI_ACT_TYPE_MOTOR:
                    A[m] = 1e-5f * learnRun.fxOmega[act] * learnRun.fxOmega[act];
                    A[m + learnRun.numMotors] = 1e-3f * learnRun.fxOmegaDot[act];
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
            A[0] = -learnRun.imuRate.A[ (ax+1)%3 ] * learnRun.imuRate.A[ (ax+2)%3 ];

// #define LEARNER_IS_TAILSITTER_SYMMETRIC

#ifdef LEARNER_IS_TAILSITTER
            float w20 = learnRun.fxOmega[0] * learnRun.fxOmega[0];
            float w21 = learnRun.fxOmega[1] * learnRun.fxOmega[1];
            float w2d0 = w20 * (learnRun.fxOmega[2]);
            float w2d1 = w21 * (learnRun.fxOmega[3]);
            float wdot0 = learnRun.fxOmegaDot[0];
            float wdot1 = learnRun.fxOmegaDot[1];
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
                    A[3] = 0.f;
                    A[4] = 0.f;
                    A[5] = 0.f;
                    A[6] = 0.f;
                    A[7] = 1e-1f * eta * p;
                    break;
                case 1: // pitch
                    A[1] = 1e-5 * (w20); // motor 0 is left
                    A[2] = 1e-5 * (w21);
                    A[3] = 1e-5 * (w2d0);
                    A[4] = 1e-5 * (w2d1);
                    A[5] = 0.f;
                    A[6] = 0.f;
                    A[7] = 1e-1f * eta * q;
                    break;
                case 2: // yaw
                    A[1] = 1e-5 * (w20); // motor 0 is left
                    A[2] = 1e-5 * (w21);
                    A[3] = 1e-5 * (w2d0);
                    A[4] = 1e-5 * (w2d1);
                    A[5] = 1e-3f * (wdot0);
                    A[6] = 1e-3f * (wdot1);
                    A[7] = 1e-1f * eta * r;
                    break;
            }
#endif // LEARNER_IS_TAILSITTER_SYMMETRIC
#endif

            ySpf[ax] = learnRun.fxSpf.A[ax] * 10.f; // scaling likely depends on sample time..
            yRateDot[ax] = learnRun.fxRateDot.A[ax]; // scaling seems okay at this sample time/filtering
            rlsNewSample(&fxRls[ax], A+1, &ySpf[ax]); // spf (skip inertia term)
            rlsNewSample(&fxRls[ax+3], A, &yRateDot[ax]); // RateDot (include inertia term)
        }

        // parallel alternative: perform rls step
        //rlsParallelNewSample(&fxSpfRls, A, ySpf);
        //rlsParallelNewSample(&fxRateDotRls, A, yRateDot);
    }

    learnerTimings.fx = cmpTimeUs(micros(), learnerTimings.start);

    if (learnAct) {
        float A[4]; // 4 regressors per actuator
        float y;

        for (int act = 0; act < indiRun.actNum; act++) {
            if (!(config->actMask & (1 << act))) {
                continue; // skip unselected actuators
            }

            switch (indiRun.actType[act]) {
                case INDI_ACT_TYPE_MOTOR:
                    A[0] = learnRun.motorD[act];
                    A[1] = learnRun.motorSqrtD[act];
                    A[2] = 1.f;
                    A[3] = -1e-4f * learnRun.motorOmegaDot[act];
                    y = learnRun.motorOmega[act] * 1e-3f; // get into range of 1
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

            rlsNewSample( &actRls[act], A, &y );
        }
    }

    learnerTimings.motor = cmpTimeUs(micros(), learnerTimings.start);

    bool gainTuningConditions = learnAct;

    if (gainTuningConditions) {
        // get slowest actuator
        float maxTau = 0.f;
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
            }

        }

        maxTau = constrainf(maxTau, 0.01f, 0.2f);

        // calculate gains
        learnRun.gains[LEARNER_LOOP_RATE] = 
            0.25f / (sq(learnRun.zeta[LEARNER_LOOP_RATE]) * maxTau);

        for (int loop = LEARNER_LOOP_ATTITUDE; loop < LEARNER_LOOP_COUNT; loop++) {
            learnRun.gains[loop] = 0.25f * learnRun.gains[loop-1] / sq(learnRun.zeta[loop]);
        }
        learnRun.gains[LEARNER_LOOP_RATE] = 10.f;
        learnRun.gains[LEARNER_LOOP_ATTITUDE] = 5.f;
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

    static bool appliedAfterQuery = false;
    if (!appliedAfterQuery && (learningQueryState == LEARNING_QUERY_DONE)) {
        if (learnerConfig()->applyIndi) {
            changeIndiProfile(INDI_PROFILE_COUNT-1); // CAREFUL WITH THIS
        }

#ifdef USE_LOCAL_POSITION
        if (learnerConfig()->applyPosition) {
            changePositionProfile(POSITION_PROFILE_COUNT-1); 
        }
#endif
        appliedAfterQuery = true;
    }

    appliedAfterQuery &= !(learningQueryState == LEARNING_QUERY_IDLE);

    if (learnFx && learnerConfig()->applyIndi) {
        if (systemConfig()->indiProfileIndex != INDI_PROFILE_COUNT-1) {
            changeIndiProfile(INDI_PROFILE_COUNT-1); // CAREFUL WITH THIS
        }
        initIndiRuntimeParameters();
    }

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

void updateLearnedParameters(indiProfile_t* indi, positionProfile_t* pos) {
    const learnerConfig_t* config = learnerConfig(); // fix this line

    for (int axis = 0; axis < 3; axis++) {
        indi->rateGains[axis] = (uint16_t) 10.f * learnRun.gains[LEARNER_LOOP_RATE];
        // attGains are expected for parallel PD, but we have cascaded, so
        indi->attGains[axis]  = (uint16_t) 10.f
             * learnRun.gains[LEARNER_LOOP_ATTITUDE] * learnRun.gains[LEARNER_LOOP_RATE];
    }

    // same for position
    pos->horz_p = (uint8_t) 10.f 
        * learnRun.gains[LEARNER_LOOP_POSITION] * learnRun.gains[LEARNER_LOOP_VELOCITY];
    pos->horz_d = (uint8_t) 10.f * learnRun.gains[LEARNER_LOOP_VELOCITY];
    pos->horz_i = pos->horz_d / 10; // fudge factor: by lack of better option at this point
    pos->vert_p = pos->horz_p;
    pos->vert_i = pos->horz_i;
    pos->vert_d = pos->horz_d;
    // pos->horz_max_v = 250; // cm/s
    // pos->horz_max_a = 500; // cm/s/s
    // pos->horz_max_iterm = 200; // cm/s
    // pos->max_tilt = 40; // conservative, like the others
    // pos->vert_max_v_up = 100; // cm/s
    // pos->vert_max_v_down = 100; // cm/s
    // pos->vert_max_a_up = 1000; // cm/s/s
    // pos->vert_max_a_down = 500; // cm/s/s
    // pos->vert_max_iterm = 100; // cm/s/s
    // fudge factor 0.5f, maybe try to see what happens with lower zeta_attitude
    pos->yaw_p = (uint8_t) 10.f * .5f * learnRun.gains[LEARNER_LOOP_ATTITUDE]; // deg/s per deg * 10
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
    indi->tails_d0[0] = 0;
    indi->tails_d0[1] = 0;

    indi->tails_clw[0]  = (int16_t) (1e8f * fxRls[0+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_clw[1]  = (int16_t) (1e8f * fxRls[0+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling

    indi->tails_cmw[0]  = (int16_t) (1e8f * fxRls[1+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmw[1]  = (int16_t) (1e8f * fxRls[1+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmd[0]  = (int16_t) (1e8f * fxRls[1+3].x[3] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cmd[1]  = (int16_t) (1e8f * fxRls[1+3].x[4] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling

    indi->tails_cnw[0]  = (int16_t) (1e8f * fxRls[2+3].x[1] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnw[1]  = (int16_t) (1e8f * fxRls[2+3].x[2] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnd[0]  = (int16_t) (1e8f * fxRls[2+3].x[3] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnd[1]  = (int16_t) (1e8f * fxRls[2+3].x[4] * 1e-5f * 1e0f); // 1e-5 for omega^2 scaling, 1e0 for y-scaling
    indi->tails_cnwd[0] = (int16_t) (1e5f * fxRls[2+3].x[5] * 1e-3f * 1e0f); // 1e-3 for omegadot scaling, 1e0 for y-scaling
    indi->tails_cnwd[1] = (int16_t) (1e5f * fxRls[2+3].x[6] * 1e-3f * 1e0f); // 1e-3 for omegadot scaling, 1e0 for y-scaling

    UNUSED(config);
    UNUSED(actG2rotIMU);
#endif // LEARNER_IS_TAILSITTER_SYMMETRIC
#else
    indi->actNum = indiRun.actNum;
    int m = 0;
    for (int act = 0; act < indi->actNum; act++) {
        if (!(config->actMask & (1 << act))) {
            continue; // skip unselected actuators
        }

        //              inv y-scale 
        float maxOmega =   1e3f  *  (actRls[act].x[0] + actRls[act].x[1]);
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

        actG1linIMU[act].V.X = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[0].x[m];
        actG1linIMU[act].V.Y = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[1].x[m];
        actG1linIMU[act].V.Z = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxRls[2].x[m];
        actG1rotIMU[act].V.X = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[3].x[1 + m];
        actG1rotIMU[act].V.Y = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[4].x[1 + m];
        actG1rotIMU[act].V.Z = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRls[5].x[1 + m];
        actG2rotIMU[act].V.X = 1.f      * 1e-3f                *     1e5f     * fxRls[3].x[1 + learnRun.numMotors + m];
        actG2rotIMU[act].V.Y = 1.f      * 1e-3f                *     1e5f     * fxRls[4].x[1 + learnRun.numMotors + m];
        actG2rotIMU[act].V.Z = 1.f      * 1e-3f                *     1e5f     * fxRls[5].x[1 + learnRun.numMotors + m];

        fp_vector_t actG1linHover, actG1rotHover, actG2rotHover;

        actG1linHover = actG1linIMU[act];
        rotate_vector_with_rotationMatrix(&actG1linHover, &imu_to_hoverR);

        actG1rotHover = actG1rotIMU[act];
        rotate_vector_with_rotationMatrix(&actG1rotHover, &imu_to_hoverR);

        actG2rotHover = actG2rotIMU[act];
        rotate_vector_with_rotationMatrix(&actG2rotHover, &imu_to_hoverR);

        indi->actG1_fx[act]    = actG1linHover.V.X;
        indi->actG1_fy[act]    = actG1linHover.V.Y;
        indi->actG1_fz[act]    = actG1linHover.V.Z;
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
    // indi->attRateDenom = 4; 
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
#define LEARNING_SAFETY_TIME_MAX ((timeUs_t) 1000000) // 1 sec

#include "flight/learning_prober.h"

void runLearningQueryStateMachine(timeUs_t current) {
    for (int i=0; i < MAX_SUPPORTED_MOTORS; i++) {
        outputFromLearningQuery[i] = 0.f;
    }

    const learnerConfig_t* config = learnerConfig();
//    int numMotors = config->numMotors;
//
//    if ((numMotors > MAX_SUPPORTED_MOTORS) || (numMotors == 0)) {
//        learningQueryState = LEARNING_QUERY_IDLE;
//        return;
//    }

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

    if (disableConditions) {
        learningQueryState = LEARNING_QUERY_IDLE;
    }

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
                // DEBUG BEUN
                //bool success = calculateHoverAttitude(indiProfileLearned);
                //if (learnRun.applyHoverRotation && success)
                //    updateBodyFrameToHover(indiProfileLearned); // rotate G1, G2, IMU rotation matrix and current attitude state

                if (learnerConfig()->applyHoverRotation) {
                    // reset accelerometer trims
                    accelerometerConfigMutable()->accZero.raw[0] = 0;
                    accelerometerConfigMutable()->accZero.raw[1] = 0;
                    accelerometerConfigMutable()->accZero.raw[2] = 0;
                    accelerometerConfigMutable()->accZero.raw[3] = 1;
                    setAccelerationTrims(&accelerometerConfigMutable()->accZero); // probably not necessary because of the horrific pointer magic
                }

                initLearnerRls(); // reset all RLS filters to 0 initial state, and reset lowpass filters
                initServoProber(current); // reset prober state
                learningQueryState = LEARNING_QUERY_WAITING_FOR_LAUNCH; goto doMore;
            }
            break;
        case LEARNING_QUERY_WAITING_FOR_LAUNCH: // catapult or throw
            if ((config->modeHover & LEARN_DURING_PROBING)
                    && (config->modeProbing & LEARN_PROBING_AFTER_CATAPULT)
                    && (catapultState == CATAPULT_DONE)) {

                // randomize board rotation after catapulting with 0 0 0 board rotation
                fp_euler_t boardEulers_fp = { 0 };
                if (learnerConfig()->randomizeMisalignment) {
                    // Forward Right Down
                    boardEulers_fp.angles.roll  = DEGREES_TO_RADIANS(ABS(learnerConfig()->rollMisalignment)) * rngFloat();
                    boardEulers_fp.angles.pitch = DEGREES_TO_RADIANS(ABS(learnerConfig()->pitchMisalignment)) * rngFloat();
                    boardEulers_fp.angles.yaw   = DEGREES_TO_RADIANS(ABS(learnerConfig()->yawMisalignment)) * rngFloat();
                } else {
                    boardEulers_fp.angles.roll  = DEGREES_TO_RADIANS(learnerConfig()->rollMisalignment);
                    boardEulers_fp.angles.pitch = DEGREES_TO_RADIANS(learnerConfig()->pitchMisalignment);
                    boardEulers_fp.angles.yaw   = DEGREES_TO_RADIANS(learnerConfig()->yawMisalignment);
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
                fp_quaternion_t newAttitude; // FRD
                getAttitudeQuaternion(&attitude);
                newAttitude = chain_quaternion(&attitude, &iboard_q);
                overrideAttitudeQuaternion(&newAttitude); // updates quat, eulers, rotation matrix and quat products
#ifdef USE_EKF
                float *theEkfX = ekf_get_X();
                theEkfX[6] = newAttitude.w;
                theEkfX[7] = newAttitude.x;
                theEkfX[8] = newAttitude.y;
                theEkfX[9] = newAttitude.z; // this probably messes up covariances, who cares
#endif
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
                || ((config->modeProbing & LEARN_PROBING_AFTER_CATAPULT) && (catapultState == CATAPULT_DONE))
                || ((config->modeProbing & LEARN_PROBING_AFTER_THROW)    && (throwState == THROW_STATE_ARMED_AFTER_THROW));

            if (launched) {
                initProber(current);
                learningQueryState = LEARNING_QUERY_DELAY; goto doMore;
            }

            break;
        case LEARNING_QUERY_DELAY: // skip this now
        case LEARNING_QUERY_ACTIVE:
            // cycles through motors
            updateProber(current);

            if (cmpTimeUs(current, proberRuntime.genStartTimeUs) > 1e3 * config->mixControlAfterMs) {
                learnRun.mixControl = true;
            }

            if (proberRuntime.isFinished) {
                // all motors done, or timeout reached
                learningQueryState = LEARNING_QUERY_DONE;
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
            if ((config->modeHover & LEARN_DURING_PROBING)
                    && ( ((config->modeProbing & LEARN_PROBING_AFTER_CATAPULT) && (catapultState == CATAPULT_DONE)) 
                        || ((config->modeProbing & LEARN_PROBING_AFTER_THROW) && (throwState == THROW_STATE_WAITING_FOR_THROW)) )
                ) {

                // use current board orientation to fix attitude
                fp_quaternion_t board_q;
                quaternion_of_rotationMatrix(&board_q, &boardRotation);

                fp_quaternion_t attitude; // FRD
                fp_quaternion_t newAttitude; // FRD
                getAttitudeQuaternion(&attitude);
                newAttitude = chain_quaternion(&board_q, &attitude);
                overrideAttitudeQuaternion(&newAttitude); // updates quat, eulers, rotation matrix and quat products
#ifdef USE_EKF
                float *theEkfX = ekf_get_X();
                theEkfX[6] = newAttitude.w;
                theEkfX[7] = newAttitude.x;
                theEkfX[8] = newAttitude.y;
                theEkfX[9] = newAttitude.z; // this probably messes up covariances, who cares
#endif

                // reset hover rotation
                hoverAttitude.w = 1.;
                hoverAttitude.x = 0.;
                hoverAttitude.y = 0.;
                hoverAttitude.z = 0.;

                // reset board rotation to 0
                boardAlignmentMutable()->rollDegrees  = 0;
                boardAlignmentMutable()->pitchDegrees = 0;
                boardAlignmentMutable()->yawDegrees   = 0;
                initBoardAlignment(boardAlignment());

                // reset learning query
                learningQueryState = LEARNING_QUERY_IDLE;
            }
            break;
    }
}

#endif
