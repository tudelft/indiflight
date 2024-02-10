
//#include "platform.h"


#include "common/maths.h"

#include "rls.h"

// --- helpers
rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, float lambda) {
    if (rls == NULL)
        return RLS_FAIL;

    // init to all zero
    *rls = (rls_t){0};

    // check bounds on n, d and assign
    if ((n > 0) && (n <= RLS_MAX_N) && (d > 0) && (d <= RLS_MAX_D)) {
        rls->n = n;
        rls->d = d;
    } else {
        return RLS_FAIL;
    }

    // check bounds on gamma and assign P
    if (gamma <= 0.f)
        return RLS_FAIL;

    for (int i = 0; i < n; i++)
        rls->P[i*n + i] = gamma;

    // initalize forgetting factor
    if ((lambda <= 0.f) || (lambda > 1.0f)) {
        return RLS_FAIL;
    }
    rls->lambda = lambda;

    return RLS_SUCCESS;
}

rls_exit_code_t rlsParallelInit(rls_parallel_t* rls, int n, int p, float gamma, float lambda) {
    if (rls == NULL)
        return RLS_FAIL;

    // init to all zero
    *rls = (rls_parallel_t){0};

    // check bounds on n, p and assign
    if ((n > 0) && (n <= RLS_MAX_N) && (p > 0) && (p <= RLS_MAX_P)) {
        rls->n = n;
        rls->p = p;
    } else {
        return RLS_FAIL;
    }

    // check bounds on gamma and assign P
    if (gamma <= 0.f)
        return RLS_FAIL;

    for (int i = 0; i < n; i++)
        rls->P[i*n + i] = gamma;

    // initalize forgetting factor
    if ((lambda <= 0.f) || (lambda > 1.0f)) {
        return RLS_FAIL;
    }
    rls->lambda = lambda;

    return RLS_SUCCESS;
}

#ifdef STM32H7
FAST_CODE
#endif
rls_exit_code_t rlsNewSample(rls_t* rls, float* AT, float* y) {
    float lam = rls->lambda;
    for (int row = 0; row < rls->n; row++) {
        if (rls->P[row + row*rls->n] > RLS_COV_LIM) {
            lam = 1.f + 0.1f * (1.f - rls->lambda); // attempt return to lower P
            break;
        }
    }

    // M = lambda I  +  A P A**T
    // K = P A**T * inv(M)

    // P A**T  =  P**T A**T  which is likely faster (however SSYMM would be even faster of course)
    float PAT[RLS_MAX_D*RLS_MAX_N];
    SGEMMt(rls->n, rls->d, rls->n, rls->P, AT, PAT, 0.f, 1.f);

    // M = lambda I  +  A (P A**T)  = lambda I  +  (P A**T)**T AT
    float M[RLS_MAX_D*RLS_MAX_D] = {0};
    for (int row = 0; row < rls->d; row++)
        M[row + row*rls->d] = lam;

    SGEMMt(rls->d, rls->d, rls->n, PAT, AT, M, 1.f, 1.f);

    // solve K = P A**T inv(M)
    // solve K**T = inv(M) A P
    float AP[RLS_MAX_D*RLS_MAX_N];
    for (int row = 0; row < rls->d; row++) {
        for (int col = 0; col < rls->n; col++) {
            AP[row + col*rls->n] = PAT[col + row*rls->d];
        }
    }

    float KT[RLS_MAX_D*RLS_MAX_N];
    float U[RLS_MAX_D*RLS_MAX_D] = {0};
    float iDiag[RLS_MAX_D];
    chol(U, M, iDiag, rls->d);
    for (int col = 0; col < rls->n; col++)
        chol_solve(U, iDiag, rls->d, &AP[col*rls->d], &KT[col*rls->d]);

    // K A P  =  (A P)**T K**T  =  (P A**T) K**T
    // lambda**(-1) (P  -  K A P)  =  ilam (P  -  (P A**T) K**T)
    float ilam = 1.f / lam;
    SGEMM(rls->n, rls->n, rls->d, PAT, KT, rls->P, -1., -ilam);

    // e = y - A x
    // e**T = y**T - x**T A**T
    float e[RLS_MAX_D];
    SGEMV(rls->d, rls->n, AT, rls->x, e);
    for (int row = 0; row < rls->d; row++)
        e[row] = y[row] - e[row];

    // x = x + K * e
    // x**T = x**T  +  e**T K**T
    float dx[RLS_MAX_N];
    SGEMV(rls->d, rls->n, KT, e, dx);
    for (int row = 0; row < rls->n; row++)
        rls->x[row] += dx[row];

    return RLS_SUCCESS;
}

#ifdef STM32H7
FAST_CODE
#endif
rls_exit_code_t rlsParallelNewSample(rls_parallel_t* rls, float* A, float* y) {
    float lam = rls->lambda;
    for (int row = 0; row < rls->n; row++) {
        if (rls->P[row + row*rls->n] > RLS_COV_LIM) {
            lam = 1.f + 0.1f * (1.f - rls->lambda); // attempt return to lower P
            break;
        }
    }

    // P AT, but more efficient to compute A PT = A P
    float PAT[RLS_MAX_N];
    SGEMV(rls->n, rls->p, rls->P, A, PAT);

    // A (P AT)  as dot product
    float APAT;
    SGEVV(rls->n, A, PAT, APAT);

    // (lambda + A P AT)**(-1)
    float isig = 1.f / (lam + APAT);

    // K = P AT * (lambda + A P AT)**(-1)
    float K[RLS_MAX_N];
    SGEVS(rls->n, PAT, isig, K);

    // P = lambda**(-1) (P - K A P) = lambda**(-1) (P - K P AT)
    float ilam = 1.f / lam;
    SGEMM(rls->n, rls->n, 1, K, PAT, rls->P, -1.f, -ilam);

    // e = y - A * x = 
    float e[RLS_MAX_P];
    SGEMV(rls->n, rls->p, rls->x, A, e);
    for (int row = 0; row < rls->p; row++)
        e[row] = y[row] - e[row];

    // x = x + K * e
    SGEMM(rls->n, rls->p, 1, K, e, rls->x, 1.f, 1.f);

    return RLS_SUCCESS;
}

/*
rls_exit_code_t rlsTest(void) {
    rls_t rls;
    float gamma = 1e3f;
    float lambda = 1.f;
    rlsInit(&rls, 3, 2, gamma, lambda);

    float A[RLS_MAX_D*RLS_MAX_N] = {
        0.03428682, 0.13472362,
        -0.16932012, 0.22687657,
        -0.43582882, -0.05850954
    };
    float y[RLS_MAX_D] = { 0.33754053, -0.16460538 };
    //float newP[RLS_MAX_N*RLS_MAX_N] = {
    //     743.78821764, -380.46820593,  205.81121973,
    //    -380.46820593,  210.67176737, -110.27608938,
    //     205.81121973, -110.27608938,   63.65037199
    //};
    //float newx[RLS_MAX_N] = {
    //    -0.22782148,
    //    -0.71320062,
    //    -0.51199282
    //};

    rlsNewSample(&rls, A, y);

    return RLS_SUCCESS;
}
*/
