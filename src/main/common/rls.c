
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
        if (rls->P[row + row*rls->n] > RLS_COV_MAX) {
            lam = 1.f + 0.1f * (1.f - rls->lambda); // attempt return to lower P
            break;
        }
    }

    // M = lambda I  +  A P A**T
    // K = P A**T * inv(M)

    // P A**T  =  P**T A**T  which is likely faster
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
    for (int col = 0; col < rls->n; col++) {
        for (int row = 0; row < rls->d; row++) {
            AP[row + col*rls->d] = PAT[col + row*rls->n];
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
    SGEMM(rls->n, rls->n, rls->d, PAT, KT, rls->P, -1.f, -ilam);

    // ensure positive definiteness (of at least the upper/lower factor)
    // we potentially need to re-symmetrize every couple of iterations in order
    // to not accumulate errors. Or actually write SSYMM/SSYMV routines that only
    // look at the upper triangular of P
    for (int row = 0; row < rls->n; row++)
        rls->P[row + row*rls->n] = MAX(RLS_COV_MIN, rls->P[row + row*rls->n]);

    // e = y - A x
    // e**T = y**T - x**T A**T
    float e[RLS_MAX_D];
    SGEMVt(rls->n, rls->d, AT, rls->x, e);
    for (int row = 0; row < rls->d; row++)
        e[row] = y[row] - e[row];

    // x = x + K * e
    // x**T = x**T  +  e**T K**T
    float dx[RLS_MAX_N];
    SGEMVt(rls->d, rls->n, KT, e, dx);
    for (int row = 0; row < rls->n; row++)
        rls->x[row] += dx[row];

    return RLS_SUCCESS;
}

#ifdef STM32H7
FAST_CODE
#endif
rls_exit_code_t rlsParallelNewSample(rls_parallel_t* rls, float* aT, float* yT) {
    float lam = rls->lambda;
    for (int row = 0; row < rls->n; row++) {
        if (rls->P[row + row*rls->n] > RLS_COV_MAX) {
            lam = 1.f + 0.1f * (1.f - rls->lambda); // attempt return to lower P
            break;
        }
    }

    // P aT, but more efficient to compute a PT = a P for memory access
    float PaT[RLS_MAX_N];
    SGEMVt(rls->n, rls->n, rls->P, aT, PaT);

    // a (P aT)  as dot product
    float aPaT;
    SGEVV(rls->n, aT, PaT, aPaT);

    // (lambda + a P aT)**(-1)
    float isig = 1.f / (lam + aPaT);

    // k = P aT * (lambda + a P aT)**(-1)   column vector
    float k[RLS_MAX_N];
    SGEVS(rls->n, PaT, isig, k);

    // P = lambda**(-1) (P - k a P) = lambda**(-1) (P - k P aT)
    float ilam = 1.f / lam;
    SGEMM(rls->n, rls->n, 1, k, PaT, rls->P, -1.f, -ilam);

    // eT = yT - a * X = 
    float eT[RLS_MAX_P];
    SGEMVt(rls->n, rls->p, rls->X, aT, eT);
    for (int row = 0; row < rls->p; row++)
        eT[row] = yT[row] - eT[row];

    // X = X + k * eT
    SGEMM(rls->n, rls->p, 1, k, eT, rls->X, 1.f, 1.f);

    return RLS_SUCCESS;
}

rls_exit_code_t rlsTest(void) {
    rls_t rls;
    float gamma = 1e3f;
    float lambda = 0.9f;
    rlsInit(&rls, 3, 2, gamma, lambda);
    rls.x[0] = 1.;
    rls.x[1] = 2.;
    rls.x[2] = 3.;

    // column major
    float AT[RLS_MAX_N*RLS_MAX_D] = {
        0.03428682, 0.13472362, -0.16932012,
        0.22687657, -0.43582882, -0.05850954
    };
    float y[RLS_MAX_D] = { 0.33754053, -0.16460538 };
    //float newP[RLS_MAX_N*RLS_MAX_N] = {
    //    740.73667109, 328.91447157, 401.56846372,
    //    328.91447157, 150.83991394, 177.92090952,
    //    401.56846372, 177.92090952, 248.03972859
    //};
    //float newx[RLS_MAX_N] = {
    //     2.72319665, 1.79674801, 0.06989191
    //};

    rlsNewSample(&rls, AT, y);

    //----
    rls_parallel_t rlsP;
    rlsParallelInit(&rlsP, 3, 2, gamma, lambda);
    rlsP.X[0] = 1.f;
    rlsP.X[1] = 1.f;
    rlsP.X[2] = 1.f;
    rlsP.X[3] = 2.f;
    rlsP.X[4] = 2.f;
    rlsP.X[5] = 2.f;

    float aT[RLS_MAX_N] = {
        0.03428682, 0.13472362, -0.16932012
    };
    float yT[RLS_MAX_P] = { 0.33754053, -0.16460538 };

    //float newPP[RLS_MAX_N*RLS_MAX_N] = {
    //    1084.39677353, -104.96897249,  131.9245951,
    //    -104.96897249,  698.65524725,  518.37292053,
    //     131.9245951 ,  518.37292053,  459.62204699
    //};
    // column major
    //float newX[RLS_MAX_N*RLS_MAX_P] = {
    //    1.23691028,  1.93089445, -0.16994451,
    //    1.88500829,  1.54816167,  2.56786866
    //};
    rlsParallelNewSample(&rlsP, aT, yT);

    return RLS_SUCCESS;
}
