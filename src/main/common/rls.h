
#pragma once

#include "platform.h"

typedef enum rls_exit_code_e {
    RLS_SUCCESS = 0,
    RLS_FAIL, // TODO fine grained error handliing
} rls_exit_code_t;

#define RLS_MAX_N 8
#define RLS_MAX_D 3
#define RLS_MAX_P 3
#define RLS_COV_LIM 1e12f
// todo: finally give in and use VLA or some malloc in order for this not to
// blow up when RLS_MAX_N rises?

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
    float lambda;
    uint32_t samples;
} rls_t; // 4 * ( N + N^2 + 1 ) + 2 bytes

// compute single-output rls's with shared regressor row-vectors in parallel
// Y = A X
//
// where
// Y in 1 x Rp
// X in Rn x Rp
typedef struct rls_parallel_s {
    long n; // parameters per RLS
    long p; // number of RLSs to be run in parallel
    float x[RLS_MAX_N*RLS_MAX_P];
    float P[RLS_MAX_N*RLS_MAX_N]; // column major!!
    float lambda;
    uint32_t samples;
} rls_parallel_t;

// we likely need to allocate 6 + 1 + 4 RLS's for a quad, thats 3.2KB

rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, float lambda);
rls_exit_code_t rlsUpdateLambda(rls_t* rls, float lambda);
rls_exit_code_t rlsNewSample(rls_t* rls, float A[RLS_MAX_N*RLS_MAX_D], float y[RLS_MAX_D]);
rls_exit_code_t rlsParallelInit(rls_parallel_t* rls, int n, int p, float gamma, float lambda);
rls_exit_code_t rlsParallelNewSample(rls_parallel_t* rls, float* A, float* y);
rls_exit_code_t rlsTest(void);
