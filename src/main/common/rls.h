
#pragma once

#include "platform.h"
#include "common/filter.h"

typedef enum rls_exit_code_e {
    RLS_SUCCESS = 0,
    RLS_FAIL, // TODO fine grained error handliing
} rls_exit_code_t;

#define RLS_MAX_N 8
#define RLS_MAX_D 3
#define RLS_MAX_P 3
#define RLS_COV_MAX 1e+10f
#define RLS_COV_MIN 1e-10f
#define RLS_MAX_P_ORDER_DECREMENT 0.1f // one order
#define RLS_FORGET_PER_VAR_PER_DT 4.
#define RLS_FORGET_MAX_VAR (4*4)
// todo: finally give in and use VLA or some malloc in order for this not to
// blow up when RLS_MAX_N rises?

// https://fanf2.user.srcf.net/hermes/doc/antiforgery/stats.pdf
typedef struct movvar_s {
    float lambda;
    float mean;
    float variance;
} emwv_t;

typedef struct fortescue_tuning_s {
    biquadFilter_t errorLP;
    emwv_t errorMV;
    float sampleFreqHz;
} fortescue_tuning_t;

void fortescueTuningInit(fortescue_tuning_t* fortescue, float cutoff, uint32_t sampleTime);
float fortescueApply(fortescue_tuning_t* fortescue, float error, float regressTimesGains);
void emwvInit(emwv_t* m, float halflifeTime, uint32_t sampleTimeUs);
float emwvApply(emwv_t* m, float sample);

// supports matrix valued regressors (ie multiple-output targets)
// y = A x
//
// where 
// y in Rd x 1, each weighed the samed
// x in Rn x 1
typedef struct rls_s {
    long n; // parameters
    long d; // outputs
    float x[RLS_MAX_N];
    float P[RLS_MAX_N*RLS_MAX_N]; // column major!!
    float lambdaBase;
    float lambda;
    fortescue_tuning_t fortescue;
    uint32_t samples;
} rls_t; // 4 * ( N + N^2 + 1 ) + 2 bytes

// compute single-output rls's with shared regressor row-vectors in parallel
// yT = A X
//
// where
// yT in 1 x Rp
// X in Rn x Rp
typedef struct rls_parallel_s {
    long n; // parameters per RLS
    long p; // number of RLSs to be run in parallel
    float X[RLS_MAX_N*RLS_MAX_P];
    float P[RLS_MAX_N*RLS_MAX_N]; // column major!!
    //float baseLambda;
    float lambda;
    uint32_t samples;
} rls_parallel_t;

// we likely need to allocate 6 + 1 + 4 RLS's for a quad, thats 3.2KB

rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, uint32_t sampleTimeUs, float actionBandwidthHz);
rls_exit_code_t rlsNewSample(rls_t* rls, float* AT, float* y);
rls_exit_code_t rlsParallelInit(rls_parallel_t* rls, int n, int p, float gamma, float Ts, float Tchar);

// a is row vector, to stay aligned with the definitions from rlsNewSample!
// So  a X  is a defined operation resulting in a row vector
//
// likewise, y is a column vector. Not that any of this matters, because row
// and column vectors are treated the same way by the linalg routines, but 
// to stay mathematically consistent
rls_exit_code_t rlsParallelNewSample(rls_parallel_t* rls, float* aT, float* yT);
rls_exit_code_t rlsTest(void);
