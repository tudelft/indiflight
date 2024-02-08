
#include "rls.h"

#include "f2c.h"
#include "clapack.h"
#include <string.h>

rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, float lambda) {
    if (rls == NULL)
        return RLS_FAIL;

    // init to all zero
    *rls = (rls_t){0};

    // check bounds on n, d and assign
    if ((n > 0) && (n < RLS_MAX_N) && (d > 0) && (d < RLS_MAX_N)) {
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
    if (rlsUpdateLambda(rls, lambda) == RLS_FAIL)
        return RLS_FAIL;

    return RLS_SUCCESS;
}

rls_exit_code_t rlsUpdateLambda(rls_t* rls, float lambda) {
    if (rls == NULL)
        return RLS_FAIL;

    if ((lambda < 0.01f) || (lambda > 1.f)) {
        return RLS_FAIL;
    }

    rls->lambda = lambda;

    return RLS_SUCCESS;
}

rls_exit_code_t rlsNewSample(rls_t* rls, float* A, float* y) {
    // A is column major!!

    // equations
    // 
    // x: parameters, A: regressors, y: observations, K: gain, P: covariance
    // K ( lambda I  +  A P A^T ) = P A^T
    // P  <--  1 / lambda  *  (P - K A P)
    // x  <--  x + K (y - A x)
    //
    // A few observations:
    // P A^T  ==  (A P)^T  because of the symmetry of P. We only have to compute that once
    // ( lambda I  +  A P A^T ) is symmetric positive definite. Cholesky is fastest to solve the system, if well conditioned. TODO: how do we check this? gershgorin disks?

    // AP   =   1.  *  A * P  +    0. * AP
    // C    = alpha *  B * A  +  beta * C    // A symmetric
    float AP [RLS_MAX_D*RLS_MAX_N];
    char SIDE = 'R';
    char UPLO = 'L';
    float alpha = 1.f;
    float beta = 0.f;
    ssymm_(&SIDE, &UPLO, &rls->d, &rls->n,
            &alpha,
            rls->P, &rls->n,
            A, &rls->d,
            &beta,
            AP, &rls->d);

    float M[RLS_MAX_D*RLS_MAX_D] = {0};
    for (int i = 0; i < rls->d; i++)
        M[i + i*rls->d] = rls->lambda;

    // M  =  alpha (AP) *  (A)**T   +   beta M
    //         1.   A   *   B**T    +    1.
    char TRANSA = 'N'; // no transpose for AP
    char TRANSB = 'T'; // transpose for A
    alpha = 1.f;
    beta = 1.f;
    sgemm_(&TRANSA, &TRANSB,
            &rls->d, &rls->d, &rls->n,
            &alpha, AP, &rls->d, // alpha * AP
            A, &rls->d,          // * A**T
            &beta, M, &rls->d);  // + beta M

#pragma message "implement routines for K and x separetely, for speed in fx calc"
    float KT[RLS_MAX_D*RLS_MAX_N];
    memcpy(KT, AP, (rls->d)*(rls->n)*sizeof(float));
    // solve AP = M K^T
    //       B  = A * X
    long INFO=-1;
    sposv_(&UPLO, &rls->d, &rls->n,
            M, &rls->d,
            KT, &rls->d,
            &INFO);
    if (INFO != 0)
        return RLS_FAIL;

    // P  =  1/lam * (P - K * A * P)
    // P  =  -1/lam * K * AP  +  1/lam P
    // C  =  alpha * A * B    +  beta P
    beta = 1. / rls->lambda;
    alpha = -beta;
    TRANSA = 'T';
    TRANSB = 'N';
    sgemm_(&TRANSA, &TRANSB, &rls->n, &rls->n, &rls->d,
            &alpha,
            KT, &rls->d,
            AP, &rls->d,
            &beta,
            rls->P, &rls->n);

    // e = y - A * x
    // e =       - A * x  +   y
    // y = alpha * A * x  +  beta*y
    TRANSA = 'N';
    alpha = -1.;
    beta = 1.;
    long int INCXY = 1;
    sgemv_(&TRANSA, &rls->d, &rls->n,
            &alpha,
            A, &rls->d,
            rls->x, &INCXY,
            &beta,
            y, &INCXY);

    // x = x + K * e
    // x  =  alpha * K * y   +   beta * x
    TRANSA = 'T'; // need to transpose KT to K
    alpha = 1.;
    beta = 1.;
    sgemv_(&TRANSA, &rls->d, &rls->n,
            &alpha,
            KT, &rls->d,
            y, &INCXY,
            &beta,
            rls->x, &INCXY);

    return RLS_SUCCESS;
}

rls_exit_code_t rlsTest(void) {
    rls_t rls;
    float gamma = 1e3f;
    float lambda = 0.99f;
    rlsInit(&rls, 3, 2, gamma, lambda);
    rlsUpdateLambda(&rls, 1.f);

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
