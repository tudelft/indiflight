
#pragma once

typedef enum rls_exit_code_e {
    RLS_SUCCESS = 0,
    RLS_FAIL, // TODO fine grained error handliing
} rls_exit_code_t;

#define RLS_MAX_N 8
#define RLS_MAX_D 3
#define RLS_P_LIM 1e12f
// todo: finally give in and use VLA or some malloc in order for this not to
// blow up when RLS_MAX_N rises?

typedef struct rls_s {
    long n;
    long d;
    float x[RLS_MAX_N];
    float P[RLS_MAX_N*RLS_MAX_N]; // column major!!
    float lambda;
} rls_t; // 4 * ( N + N^2 + 1 ) + 2 bytes

// we likely need to allocate 6 + 1 + 4 RLS's for a quad, thats 3.2KB

rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, float lambda);
rls_exit_code_t rlsUpdateLambda(rls_t* rls, float lambda);
rls_exit_code_t rlsNewSample(rls_t* rls, float A[RLS_MAX_N*RLS_MAX_D], float y[RLS_MAX_D]);
rls_exit_code_t rlsTest(void);
