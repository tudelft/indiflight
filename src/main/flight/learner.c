
#include "common/maths.h"
#include "common/rls.h"
#include "common/axis.h"
#include "common/filter.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "flight/catapult.h"
#include "flight/pos_ctl.h"
#include "flight/throw.h"

#include "f2c.h"
#include "clapack.h"

#include <stdbool.h>

#include "learner.h"

learning_query_state_t learningQueryState = LEARNING_QUERY_IDLE;

#ifdef USE_LEARNER

#ifndef USE_INDI
#error "must use learner with USE_INDI"
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(learnerConfig_t, learnerConfig, PG_LEARNER_CONFIG, 1);
PG_RESET_TEMPLATE(learnerConfig_t, learnerConfig, 
    .mode = (uint8_t) LEARN_AFTER_CATAPULT,
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
    .motorFiltHz = 40,
    .zetaRate = 80,
    .zetaAttitude = 80,
    .zetaVelocity = 60,
    .zetaPosition = 80,
    .applyIndiProfileAfterQuery = false,
    .applyPositionProfileAfterQuery = false
);

// extern
learnerRuntime_t learnRun = {0};

void initLearnerRuntime(void) {
    learnRun.zeta[LEARNER_LOOP_RATE]     = constrainf(0.01f * learnerConfig()->zetaRate    , 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_ATTITUDE] = constrainf(0.01f * learnerConfig()->zetaAttitude, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_VELOCITY] = constrainf(0.01f * learnerConfig()->zetaVelocity, 0.5f, 1.0f);
    learnRun.zeta[LEARNER_LOOP_POSITION] = constrainf(0.01f * learnerConfig()->zetaPosition, 0.5f, 1.0f);
    learnRun.applyIndiProfileAfterQuery = (bool) learnerConfig()->applyIndiProfileAfterQuery;
    learnRun.applyPositionProfileAfterQuery = (bool) learnerConfig()->applyPositionProfileAfterQuery;
}

// externs
float outputFromLearningQuery[MAX_SUPPORTED_MOTORS];
static timeUs_t learningQueryEnabledAt = 0;
rls_parallel_t motorRls[MAXU];
rls_t imuRls;
rls_parallel_t fxSpfRls;
rls_parallel_t fxRateDotRls;

static biquadFilter_t imuRateFilter[3];
static biquadFilter_t imuSpfFilter[3];

static biquadFilter_t motorOmegaFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t motorDFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t motorSqrtDFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxOmegaFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxUFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxRateFilter[MAX_SUPPORTED_MOTORS];
static biquadFilter_t fxSpfFilter[MAX_SUPPORTED_MOTORS];

static t_fp_vector hoverThrust;

#define LEARNING_MAX_ACT ((int) RLS_MAX_N / 2)
#define LEARNER_OMEGADOT_SCALER 1e-5f // for numerical stability
#define LEARNER_OMEGADOTDIFF_SCALER 10.f // for numerical stability
#define LEARNER_NULLSPACE_THRESH 1e-3f // todo, do we need somethign relative here?
#define LEARNER_NUM_POWER_ITERATIONS 1

// really dumb rng number to reset power iterations in case a numerically 
// orthogonal vector is encountered
#define LEARNER_RNG_MAX 101 // should be prime to avoid wrapping dumbRng onto itself
static uint8_t randomSequence[LEARNER_RNG_MAX] = {
    // from random import randint; ",".join([hex(randint(0,255)) for i in range(101)])
    0x7a,0x11,0xaa,0x88,0xc,0x38,0x1,0xfc,0xd3,0x73,0xfc,0xd2,0x47,0xc1,0x39,0x6a,0x1b,0xe8,0x82,0x6,0x1c,0xd,0x11,0xa7,0x3b,0x2d,0x69,0xf7,0xad,0xef,0x75,0x6c,0x0,0x15,0x3d,0x18,0x1e,0x87,0xdc,0xda,0xa8,0x7c,0x6b,0xc5,0x18,0xe3,0xd8,0x86,0xf,0x2f,0x65,0x1,0x96,0xf0,0x11,0x43,0xd6,0x46,0x20,0x3d,0xc7,0x20,0xaa,0x60,0x6,0x5c,0x61,0x4f,0x83,0xf7,0x90,0x6,0xa7,0x4a,0x96,0x80,0x68,0x4b,0xa5,0x8b,0xdd,0xb0,0xce,0xa1,0xce,0x7b,0x2d,0xa1,0x5b,0x1,0x7e,0xfa,0xa1,0x8a,0x7,0xca,0x63,0xfe,0x69,0x9e,0xc5
};

float dumbRng(void) {
    // return "random number" between -1 and 1
    static int rngSeed = 0; // index in randomSequence
    if (++rngSeed >= LEARNER_RNG_MAX)
        // wrap sequence
        rngSeed = 0;

    return ( ((float)(randomSequence[rngSeed] - 127)) / 128.f );
}

static indiProfile_t* indiProfileLearned;
static positionProfile_t* positionProfileLearned;

// learning
void initLearner(void) {
    // limited to 4 for now
    learnerConfigMutable()->numAct = MIN(LEARNING_MAX_ACT, learnerConfigMutable()->numAct);

    indiProfileLearned = indiProfilesMutable(INDI_PROFILE_COUNT-1);
    positionProfileLearned = positionProfilesMutable(POSITION_PROFILE_COUNT-1);
    initLearnerRuntime();

    rlsInit(&imuRls, 3, 3, 1e2f, 0.995f);

    // init filters and other rls
    float dT = indiRun.dT;
    float Tchar = 10.f*0.025f; // 10 times act constant
    rlsParallelInit(&fxSpfRls, learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // forces
    rlsParallelInit(&fxRateDotRls, 2.f*learnerConfig()->numAct, 3, 1e2f, dT, Tchar); // rotations need twice the parameters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        //rlsParallelInit(&fxRls[axis], learnerConfig()->numAct, 1, 1e0f, 0.997f); // forces
        //rlsParallelInit(&fxRls[axis+3], 2*learnerConfig()->numAct, 1, 1e0f, 0.997f); // rotational
        biquadFilterInitLPF(&imuRateFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&imuSpfFilter[axis], learnerConfig()->imuFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxRateFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxSpfFilter[axis], learnerConfig()->fxFiltHz, gyro.targetLooptime);
    }

    for (int act = 0; act < learnerConfig()->numAct; act++) {
        rlsParallelInit(&motorRls[act], 4, 1, 1e2f, dT, Tchar);
        biquadFilterInitLPF(&fxOmegaFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&fxUFilter[act], learnerConfig()->fxFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&motorOmegaFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&motorDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
        biquadFilterInitLPF(&motorSqrtDFilter[act], learnerConfig()->motorFiltHz, gyro.targetLooptime);
    }
}

#ifdef STM32H7
FAST_CODE
#endif
static void updateLearningFilters(void) {
    static t_fp_vector imuPrevRate = {0};
    static t_fp_vector fxPrevRateDot = {0};
    static t_fp_vector fxPrevSpf = {0};

    // IMU rls filters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        learnRun.imuRate.A[axis] = biquadFilterApply(&imuRateFilter[axis], indiRun.rate.A[axis]);
        learnRun.imuSpf.A[axis] = biquadFilterApply(&imuSpfFilter[axis], indiRun.spf.A[axis]);

        learnRun.imuRateDot.A[axis] = indiRun.indiFrequency * (learnRun.imuRate.A[axis] - imuPrevRate.A[axis]);
        imuPrevRate.A[axis] = learnRun.imuRate.A[axis];
    }

//#pragma message "TODO: implement imu location correction"
    // IMU correction
    float wx, wy, wz;
    float rx, ry, rz;
    float dwx, dwy, dwz;
    float ax, ay, az;

#ifdef HIL_BUILD
    rx = 0.f; ry = 0.f; rz = 0.f;
#else
    rx = -0.010f; ry = -0.010f; rz = 0.015f; // hardcoded for now
#endif

    dwx = indiRun.rateDot.A[0];
    dwy = indiRun.rateDot.A[1];
    dwz = indiRun.rateDot.A[2];

    wx = indiRun.rate.A[0];
    wy = indiRun.rate.A[1];
    wz = indiRun.rate.A[2];

    ax = indiRun.spf.A[0];
    ay = indiRun.spf.A[1];
    az = indiRun.spf.A[2];

    float fxSpfCorrected[3];
    fxSpfCorrected[0] = ax - ( rx * (-sq(wy)-sq(wz)) + ry * (wx*wy - dwz)    + rz * (wx*wz + dwy)    );
    fxSpfCorrected[1] = ay - ( rx * (wx*wy + dwz)    + ry * (-sq(wx)-sq(wz)) + rz * (wy*wz - dwx)    );
    fxSpfCorrected[2] = az - ( rx * (wx*wz - dwy)    + ry * (wy*wz + dwx)    + rz * (-sq(wx)-sq(wy)) );

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float fxRateDot = biquadFilterApply(&fxRateFilter[axis], indiRun.rateDot.A[axis]);
        learnRun.fxRateDotDiff.A[axis] = fxRateDot - fxPrevRateDot.A[axis];
        fxPrevRateDot.A[axis] = fxRateDot;

        float fxSpf = biquadFilterApply(&fxSpfFilter[axis], fxSpfCorrected[axis]);
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
        // but we need to ensure [0, 1] for the square root
        float dConstr = constrainf(indiRun.d[act], 0.f, 1.f);
        learnRun.motorSqrtD[act] = biquadFilterApply(&motorSqrtDFilter[act], sqrtf(dConstr));
    }
}

static struct learnerTimings_s {
    timeUs_t start;
    timeDelta_t filters;
    timeDelta_t imu;
    timeDelta_t fx;
    timeDelta_t motor;
    timeDelta_t hover;
} learnerTimings = {0};

#ifdef STM32H7
FAST_CODE
#endif
void updateLearner(timeUs_t current) {
    UNUSED(current);

    learnerTimings.start = micros();

    // update learning sync filters
    updateLearningFilters();
    learnerTimings.filters = cmpTimeUs(micros(), learnerTimings.start);

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

        // remember: column major formulation!
        float AT[3*3] = {
            -(w->Y * w->Y + w->Z * w->Z),   w->X * w->Y - dw->Z,           w->X * w->Z + dw->Y,
             w->X * w->Y + dw->Z,          -(w->X * w->X + w->Z * w->Z),   w->Y * w->Z - dw->X,
             w->X * w->Z - dw->Y,           w->Y * w->Z + dw->X,          -(w->X * w->X + w->Y * w->Y)
        };
        float y[3] = { a->X, a->Y, a->Z }; // in the 1 - 10 m/s/s range id say

        for (int i = 0; i < 9; i++)
            AT[i] *= 1e-2f; // parameters are in the cm range, so make sure they will be around 1 to avoid numerical issues
            // it if true parameter is 30cm is logged at in 0.3*100*1000 = 30000, which is max for logging. 1mm is logged as 0.001*100*1000 = 100

        // perform rls step
        rlsNewSample(&imuRls, AT, y);
    }
    learnerTimings.imu = cmpTimeUs(micros(), learnerTimings.start);

    bool fxLearningConditions = FLIGHT_MODE(LEARNER_MODE) && ARMING_FLAG(ARMED) && !isTouchingGround()
        && (
            (learnerConfig()->mode & LEARN_DURING_FLIGHT)
            || ((learningQueryState > LEARNING_QUERY_DELAY) && (learningQueryState != LEARNING_QUERY_DONE))
           );

    if (fxLearningConditions) {
        //setup regressors
        float A[RLS_MAX_N];
        for (int act = 0; act < learnerConfig()->numAct; act++) {
            A[act] = 1e-5f * 2.f * learnRun.fxOmega[act] * learnRun.fxOmegaDiff[act];
            A[act + learnerConfig()->numAct] = 1e-3f * learnRun.fxOmegaDotDiff[act];
        }
        float ySpf[3];
        float yRateDot[3];
        for (int i = 0; i < 3; i++) {
            ySpf[i] = learnRun.fxSpfDiff.A[i] * 10.f; // scaling likely depends on sample time..
            yRateDot[i] = learnRun.fxRateDotDiff.A[i]; // scaling seems okay at this sample time/filtering
        }

        // perform rls step
        rlsParallelNewSample(&fxSpfRls, A, ySpf);
        rlsParallelNewSample(&fxRateDotRls, A, yRateDot);
    }
    learnerTimings.fx = cmpTimeUs(micros(), learnerTimings.start);

    // same for now
    bool motorLearningConditions = fxLearningConditions;

    if (motorLearningConditions) {
        for (int act = 0; act < learnerConfig()->numAct; act++) {
            float A[4] = {
                learnRun.motorD[act],
                learnRun.motorSqrtD[act],
                1.f,
                -1e-4f * learnRun.motorOmegaDot[act]
            };
            float y = learnRun.motorOmega[act] * 1e-3f; // get into range of 1
            rlsParallelNewSample(&motorRls[act], A, &y);
        }
    }
    learnerTimings.motor = cmpTimeUs(micros(), learnerTimings.start);

    bool gainTuningConditions = fxLearningConditions;

    if (gainTuningConditions) {
        // get slowest actuator
        float maxTau = 0.f;
        for (int act = 0; act < learnerConfig()->numAct; act++)
            maxTau = MAX(maxTau, motorRls[act].X[3] * 0.1f);
        maxTau = constrainf(maxTau, 0.01f, 0.2f);

        // calculate gains
        learnRun.gains[LEARNER_LOOP_RATE] = 
            0.25f / (sq(learnRun.zeta[LEARNER_LOOP_RATE]) * maxTau);

        for (int loop = LEARNER_LOOP_ATTITUDE; loop < LEARNER_LOOP_COUNT; loop++)
            learnRun.gains[loop] = 0.25f * learnRun.gains[loop-1] / sq(learnRun.zeta[loop]);
    }

    bool hoverAttitudeLearningConditions = false; // for debugging

    if (hoverAttitudeLearningConditions) {
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
        const uint8_t numAct = learnerConfig()->numAct;
        float BfT[LEARNING_MAX_ACT * 3];
        float BrT[LEARNING_MAX_ACT * LEARNING_MAX_ACT] = {0}; // waste of stack, reduce because M < N?
        for (int col = 0; col < 3; col++) {
            for (int row = 0; row < numAct; row++) {
                BfT[row + col*numAct] = indiRun.actG1[col][row];
                BrT[row + col*numAct] = indiRun.actG1[3+col][row];
            }
        }

#define LEARNING_HOVER_D 3
        integer M = numAct;
        integer N = LEARNING_HOVER_D;
        if (M < N)
            goto panic; // not implemented, would have to adjust sorg2r inputs?

        // A is BrT
        integer LDA = numAct;
        integer JPVT[LEARNING_MAX_ACT] = {0}; // all free columns on entry
        real TAU[MIN(LEARNING_MAX_ACT, LEARNING_HOVER_D)];
        real WORK[3*LEARNING_HOVER_D + 1]; 
        integer LWORK = 3*LEARNING_HOVER_D + 1;  // see sgepq3 manual
        integer INFO;
        sgeqp3_(&M, &N, BrT, &LDA, JPVT, TAU, WORK, &LWORK, &INFO);
        if (INFO < 0)
            goto panic; // panic

        int sizeNr = numAct - LEARNING_HOVER_D; // if Br full rank

        // since we used sgeqp3_, the columns of the R factor are sorted so
        // that the diagonals are non-increasing. To find if we have rank-
        // deficiency (and this a larger nullspace), we can just find the
        // last non-zero diagonal element of R
        for (int row = LEARNING_HOVER_D-1; row >= 0; row--)
            // could do bisection search, not going to
            if (fabsf(BrT[row + row*numAct]) < LEARNER_NULLSPACE_THRESH)
                // diagonal entry in R factor is small, Br is rank deficient
                // TODO: do we need to check BrT[row + (row+1:numAct)*numAct]? DONE: chatGPT says no
                sizeNr++;
            else
                // cannot have any more zero-diagonal entries, since R is sorted
                break;

        if (sizeNr == 0)
            // can happen, when N = M and Br full rank
            goto panic; // panic

        // todo interpret INFO
        integer K = numAct - sizeNr;
        sorg2r_(&M, &M, &K, BrT, &LDA, TAU, WORK, &INFO); // K == N true? or M - sizeNr?
        // todo interpret INFO

        float *Nr = &BrT[(numAct-sizeNr)*numAct];

        // 2. Generate H. allow only W = I for now.
        //float H[LEARNING_MAX_ACT*LEARNING_MAX_ACT]; // todo could be smaller since we disallow M < N?
        //SGEMMt(sizeNr, sizeNr, numAct, Nr, Nr, H, 0.f, 1.f);
        // jokes, this is always I, if W = I

        // 3. Generate A
        float BfNr[3*LEARNING_MAX_ACT]; // todo could be smaller since we disallow M < N?
        float A[LEARNING_MAX_ACT*LEARNING_MAX_ACT]; // will be a waste of stack.. allocate on the RAM!
        SGEMMt(3, sizeNr, numAct, BfT, Nr, BfNr, 0.f, 1.f);
        SGEMMt(sizeNr, sizeNr, 3, BfNr, BfNr, A, 0.f, 1.f);

        // 4. find eigenvector. Remember H == I
        static float v[LEARNING_MAX_ACT] = {-0.5051f,  0.3486f, -0.9154f,  0.5560f}; // can also be smaller since M < N
        //static float v[LEARNING_MAX_ACT] = {0}; // for testing robustness, 0 makes no sense
        float HinvAv[LEARNING_MAX_ACT]; // can be smaller
        float HinvAvNorm2;
        for (int i = LEARNER_NUM_POWER_ITERATIONS; i > 0; i--) {
            SGEMVt(sizeNr, sizeNr, A, v, HinvAv); // A is symmetric, SGEMVf is faster
            SGEVV(sizeNr, HinvAv, HinvAv, HinvAvNorm2); // guaranteed >= 0.f
            if (HinvAvNorm2 < 1e-8f) {
                // we picked a starting vector near orthogonal to the eigenvector
                // we want to find. reset to a pseudorandom vector
                for (int row = 0; row < sizeNr; row++)
                    v[row] = dumbRng();
                continue;
            }
            SGEVS(sizeNr, HinvAv, 1.f / sqrtf(HinvAvNorm2), v); // fast inverse sqrt anyone?
        }

        // 5. find length for v to cancel gravity
        // this seems not strictly necessary if you only want the direction,
        // now that I think about it.. but maybe it's still good to cross check
        // the (linearized) hover thrust.
        float Av[LEARNING_MAX_ACT];
        float vTAv;
        SGEMVt(sizeNr, sizeNr, A, v, Av);
        SGEVV(sizeNr, v, Av, vTAv);
        if (vTAv < 1e-10f)
            goto panic; // panic

        float scale = GRAVITYf * 1.f / sqrtf( vTAv ); // fast inverse sqrt?
        SGEVS(sizeNr, v, scale, v);

        // 6. find if up or down
        float uHover[LEARNING_MAX_ACT];
        SGEMV(numAct, sizeNr, Nr, v, uHover);
        float uHoverSum = 0.f;
        for (int row = 0; row < numAct; row++)
            uHoverSum += uHover[row];

        if (uHoverSum < 0.f) {
            for (int row = 0; row < sizeNr; row++)
                v[row] = -v[row];
            for (int row = 0; row < numAct; row++)
                uHover[row] = -uHover[row];
        }

        // 7. hover thrust direction
        // Bf * u
        SGEMVt(numAct, 3, BfT, uHover, hoverThrust.A);

        // 8. verify that Br uHover == 0
    }
panic:
    learnerTimings.hover = cmpTimeUs(micros(), learnerTimings.start);

#ifdef USE_CLI_DEBUG_PRINT
    static unsigned int printCounter = 0;
    if (!(++printCounter % 1000))
        cliPrintLinef("Learner Timings (us): filt %d, imu %d, fx %d, mot %d, hover %d", learnerTimings.filters, learnerTimings.imu, learnerTimings.fx, learnerTimings.motor, learnerTimings.hover);
#endif
}

void updateLearnedParameters(indiProfile_t* indi, positionProfile_t* pos) {
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
    pos->horz_max_v = 250; // cm/s
    pos->horz_max_a = 500; // cm/s/s
    pos->horz_max_iterm = 200; // cm/s
    pos->max_tilt = 40; // conservative, like the others
    pos->vert_p = pos->horz_p;
    pos->vert_i = pos->horz_i;
    pos->vert_d = pos->horz_d;
    pos->vert_max_v_up = 100; // cm/s
    pos->vert_max_v_down = 100; // cm/s
    pos->vert_max_a_up = 1000; // cm/s/s
    pos->vert_max_a_down = 500; // cm/s/s
    pos->vert_max_iterm = 100; // cm/s/s
    // fudge factor 0.5f, maybe try to see what happens with lower zeta_attitude
    pos->yaw_p = (uint8_t) 10.f * .5f * learnRun.gains[LEARNER_LOOP_ATTITUDE]; // deg/s per deg * 10
    pos->weathervane_p = 0;
    pos->weathervane_min_v = 200; // cm/s/s
    pos->use_spf_attenuation = 1;

    indi->manualUseCoordinatedYaw = 1;
    indi->manualMaxUpwardsSpf = 20; // conservative

    indi->attMaxTiltRate = 500; // reduce slightly
    indi->attMaxYawRate = 300; // reduce
    for (int act = 0; act < learnerConfig()->numAct ; act++) {
        //              inv y-scale 
        float maxOmega =   1e3f  *  (motorRls[act].X[0] + motorRls[act].X[1]);
        indi->actMaxRpm[act] = MAX(100.f, 60.f * 0.5f / M_PIf  *  maxOmega); // convert to deg/s
        indi->actHoverRpm[act] = indi->actMaxRpm[act] >> 1; // guess, shouldnt matter since we have useRpmDotFeedback = true
        //                                            inv y-scale   a-scale     config-scale
        indi->actTimeConstMs[act] = (uint8_t) constrainf(1e3f      *  1e-4f  *    1000.f     * motorRls[act].X[3], 10.f, 200.f);

        if ((motorRls[act].X[0] > 0.f) && (motorRls[act].X[1] > 0.f))
            indi->actNonlinearity[act] = (uint8_t) 100.f * constrainf(
                motorRls[act].X[0] / (motorRls[act].X[0] + motorRls[act].X[1]),
                0.f, 1.f);
            // todo: transform to match kappa better
        else
            indi->actNonlinearity[act] = 50;

        indi->actLimit[act] = 60; // conservative for now
        //                    inv y-scale        a-scale           config scale
        indi->actG1_fx[act]    = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxSpfRls.X[0 + act];
        indi->actG1_fy[act]    = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxSpfRls.X[4 + act];
        indi->actG1_fz[act]    = 0.1f     * 1e-5f * sq(maxOmega) *     1e2f     * fxSpfRls.X[8 + act];
        indi->actG1_roll[act]  = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRateDotRls.X[0  + act];
        indi->actG1_pitch[act] = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRateDotRls.X[8  + act];
        indi->actG1_yaw[act]   = 1.f      * 1e-5f * sq(maxOmega) *     1e1f     * fxRateDotRls.X[16 + act];
        indi->actG2_roll[act]  = 1.f      * 1e-3f                *     1e5f     * fxRateDotRls.X[0  + 4 + act];
        indi->actG2_pitch[act] = 1.f      * 1e-3f                *     1e5f     * fxRateDotRls.X[8  + 4 + act];
        indi->actG2_yaw[act]   = 1.f      * 1e-3f                *     1e5f     * fxRateDotRls.X[16 + 4 + act];

        indi->wlsWu[act] = 1;
        indi->u_pref[act] = 0;
    }

    indi->imuSyncLp2Hz = 15; // lord knows
    indi->wlsWv[0] = 1;
    indi->wlsWv[1] = 1;
    indi->wlsWv[2] = 50;
    indi->wlsWv[3] = 50;
    indi->wlsWv[4] = 50;
    indi->wlsWv[5] = 5;

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
    learnerConfigMutable()->numAct = 6;

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

    bool disableConditions = !FLIGHT_MODE(LEARNER_MODE)
            || !(learnerConfig()->mode & (LEARN_AFTER_CATAPULT | LEARN_AFTER_THROW));

    bool enableConditions = !ARMING_FLAG(ARMED) && !disableConditions;

doMore:
    switch (learningQueryState) {
        case LEARNING_QUERY_IDLE:
            if (enableConditions) {
                initLearner(); // reset all RLS filters to 0 initial state, and reset lowpass filters
                learningQueryState = LEARNING_QUERY_WAITING_FOR_LAUNCH; goto doMore;
            }
            break;
        case LEARNING_QUERY_WAITING_FOR_LAUNCH: // catapult or throw
            if (disableConditions) {
                learningQueryState = LEARNING_QUERY_IDLE;
                break;
            }

            if ( ((learnerConfig()->mode & LEARN_AFTER_CATAPULT) && (catapultState == CATAPULT_DONE))
                    || ((learnerConfig()->mode & LEARN_AFTER_THROW) && (throwState == THROW_STATE_ARMED_AFTER_THROW)) ) {
                learningQueryEnabledAt = current;
                startAt = current + c->delayMs*1e3;
                // 10ms grace period, then cutoff, even if learning is not done, because that means error in the state machine
                safetyTimeoutUs = 10000 + c->delayMs*1e3 +
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
                        && ( (time_in_query + 1e3 * c->overlapMs) > 1e3 * (c->stepMs + c->rampMs) ) ) {
                    resetMotorQueryState(motorStates, motor + 1, true);
                    // FIXME
                    //uint8_t* p = NULL;
                    //*p = 0; // generate crash
                    //__asm("b ."); // generate hang
                    //while (1) __asm("nop"); // another hang 
                }

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
                        outputFromLearningQuery[motor] = c->rampAmp * 0.01f * ( 1.f - ( ((float) time_in_ramp) / ((float) c->rampMs * 1e3f) ) );
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
                learningQueryState = LEARNING_QUERY_APPLYING; goto doMore;
            }

            break;
        case LEARNING_QUERY_APPLYING: 
            updateLearnedParameters(indiProfileLearned, positionProfileLearned);

            if (learnRun.applyIndiProfileAfterQuery)
                changeIndiProfile(INDI_PROFILE_COUNT-1); // CAREFUL WITH THIS

            if (learnRun.applyPositionProfileAfterQuery)
                changePositionProfile(POSITION_PROFILE_COUNT-1); 

            learningQueryState = LEARNING_QUERY_DONE; goto doMore;
        case LEARNING_QUERY_DONE:
            if ( ((learnerConfig()->mode & LEARN_AFTER_CATAPULT) && (catapultState == CATAPULT_WAITING_FOR_ARM))
                    || ((learnerConfig()->mode & LEARN_AFTER_THROW) && (throwState == THROW_STATE_WAITING_FOR_THROW))
                )
                learningQueryState = LEARNING_QUERY_IDLE;
            break;
    }

    for (int motor = 0; motor < c->numAct; motor++) {
        outputFromLearningQuery[motor] = constrainf(outputFromLearningQuery[motor], 0.f, 1.f);
    }
}

#endif
