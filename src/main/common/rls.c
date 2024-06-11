/*
 * Implement statically allocated recursive least squares
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


//#include "platform.h"


#include "common/maths.h"
#include <math.h>

#include "rls.h"
#include "common/filter.h"

void fortescueTuningInit(fortescue_tuning_t* fortescue, float cutoffFreqHz, uint32_t sampleTimeUs) {
    biquadFilterInitLPF( &(fortescue->errorLP), cutoffFreqHz, sampleTimeUs );
    emwvInit( &(fortescue->errorMV), 1. / ( 2.f * M_PIf * cutoffFreqHz ), sampleTimeUs );
    fortescue->errorMV.variance = 0.01f;
    fortescue->sampleFreqHz = 1e6f / ((float) sampleTimeUs);
}

float fortescueApply(fortescue_tuning_t* fortescue, float error, float regressTimesGains) {
    float errorLP = biquadFilterApply( &(fortescue->errorLP), error );
    float errorHP = error - errorLP;
    float errorVar = MAX(1e-4, emwvApply( &(fortescue->errorMV), errorHP ));

    // lam = 1 - ( 1 - AT K ) * (e**2) / Sigma0
    float e2 = error * error;
    float lambda = 1.f - ( 1.f - regressTimesGains ) * e2 / ( 5. * fortescue->sampleFreqHz * errorVar );

    if (lambda != lambda)
        __asm("BKPT #0\n") ; // Break into the debugger

    return lambda;
}

void emwvInit(emwv_t* m, float cutoffFreqHz, uint32_t sampleTimeUs) {
    m->lambda = powf( 1.f - (float) _M_LN2, (2.f * M_PIf * cutoffFreqHz * 1e-6f * ((float) sampleTimeUs)) );
    m->mean = 0.f;
    m->variance = 0.f;
}

float emwvApply(emwv_t* m, float sample) {
    float diff = sample - m->mean;
    float incr = ( 1 - m->lambda ) * diff;
    m->mean += incr;
    m->variance = m->lambda * ( m->variance  +  diff * incr );

    return m->variance;
}

// --- helpers
rls_exit_code_t rlsInit(rls_t* rls, int n, int d, float gamma, uint32_t sampleTimeUs, float actionBandwidthHz) {
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
    float sampleFreqHz = 1e6f / ((float) sampleTimeUs);
    if ((sampleTimeUs <= 0) || (actionBandwidthHz >= 0.45 * sampleFreqHz))
        return RLS_FAIL;

    rls->lambdaBase = rls->lambda = powf(1.f - (float) _M_LN2, (2. * M_PIf * actionBandwidthHz) / (sampleFreqHz));

    // initialize fortescue tuner
    fortescueTuningInit( &(rls->fortescue), actionBandwidthHz, sampleTimeUs );

    return RLS_SUCCESS;
}

rls_exit_code_t rlsParallelInit(rls_parallel_t* rls, int n, int p, float gamma, float Ts, float actMinBandwidthHz) {
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
    if ((Ts <= 0.f) || (actMinBandwidthHz <= 2.f*Ts)) {
        return RLS_FAIL;
    }

    rls->lambda = powf(1.f - (float) _M_LN2, Ts / actMinBandwidthHz);

    return RLS_SUCCESS;
}

#ifdef STM32H7
FAST_CODE
#endif
rls_exit_code_t rlsNewSample(rls_t* rls, float* AT, float* y) {
    rls->samples++;

    // re-symmetrize P by copying upper tri to lower tri
    for (int col = 0; col < rls->n; col++)
        for (int row = col+1; row < rls->n; row++)
            rls->P[col*rls->n + row] = rls->P[row*rls->n + col];

    // M = lambda I  +  A P A**T
    // K = P A**T * inv(M)

    // P A**T  =  P**T A**T  which is likely faster
    float PAT[RLS_MAX_D*RLS_MAX_N];
    SGEMMt(rls->n, rls->d, rls->n, rls->P, AT, PAT, 0.f, 1.f);

    // M = lambda I  +  A (P A**T)  = lambda I  +  (P A**T)**T AT
    float M[RLS_MAX_D*RLS_MAX_D] = {0};
    for (int row = 0; row < rls->d; row++)
        M[row + row*rls->d] = rls->lambda;

    SGEMMt(rls->d, rls->d, rls->n, PAT, AT, M, 1.f, 1.f);
    if (M[0] <= 0.f)
        __asm("BKPT #0\n") ; // Break into the debugger

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

    // e = y - A x
    // e**T = y**T - x**T A**T
    float e[RLS_MAX_D];
    SGEMVt(rls->n, rls->d, AT, rls->x, e);
    for (int row = 0; row < rls->d; row++)
        e[row] = y[row] - e[row];

    if (rls->d == 1) {
        // adaptive forgetting only implemented for MISO systems
        float ATK;
        SGEVV(rls->n, AT, KT, ATK);
        float lamFortescue = fortescueApply( &(rls->fortescue), e[0], ATK );
        rls->lambda = constrainf(lamFortescue, rls->lambdaBase, 1.);
    }

    float traceP = 0.f;
    for (int row = 0; row < rls->n; row++) {
        traceP += rls->P[row + row*rls->n];
        if (rls->P[row + row*rls->n] > RLS_COV_MAX) {
            rls->lambda = 1.f + 0.1f * (1.f - rls->lambdaBase); // attempt return to lower P
            // break; // still need to compute trace
        }
    }

    // in the subtraction below numerical inaccuracies can creep in.
    // limit the order reduction
    float traceKAP = 0.f;
    // get trace(PAT KT) = trace(K A P)
    for (int i = 0; i < rls->n; i++) {
        // n dot products of dimension d
        float tmp = 0;
        SGEVV(rls->d, (&AP[i*rls->d]), (&KT[i*rls->d]), tmp);
        traceKAP += tmp;
    }

    // K A P  =  (A P)**T K**T  =  (P A**T) K**T
    // lambda**(-1) (P  -  K A P)  =  ilam (P  - KAPmult * (P A**T) K**T)
    // =  ilam*KAPmult (iKAPmult * P  - (P A**T) K**T)
    // P  <--  -ilam*KAPmult * (-iKAPmult * P  +  (P A**T) K**T)

    // select KAPmult to ensure that traceP - trace(KAP * KAPmult) > 0.1 traceP
    // to maintain numerical accuracy
    float KAPmult = 1.f;
    if (traceKAP > RLS_COV_MIN)
        KAPmult = constrainf((1.f - RLS_MAX_P_ORDER_DECREMENT) * (traceP / traceKAP), 1e-6f, 1.f);

    float ilam = 1.f / rls->lambda;
    float ilamKAPmult = ilam * KAPmult;
    float iKAPmult = 1.f / KAPmult;

    SGEMM(rls->n, rls->n, rls->d, PAT, KT, rls->P, -iKAPmult, -ilamKAPmult);
    if (rls->P[0] < 0.)
        __asm("BKPT #0\n") ; // Break into the debugger

    // ensure positive definiteness (of at least the upper/lower factor)
    // we potentially need to re-symmetrize every couple of iterations in order
    // to not accumulate errors. Or actually write SSYMM/SSYMV routines that only
    // look at the upper triangular of P
    //for (int row = 0; row < rls->n; row++)
    //    rls->P[row + row*rls->n] = MAX(RLS_COV_MIN, rls->P[row + row*rls->n]);
    // can never happen now, since we have KAPmult. Re-symmetrizing is a good idea though

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
    rls->samples++;

    float lam = rls->lambda;
    float diagP[RLS_MAX_N] = {0};
    for (int row = 0; row < rls->n; row++) {
        diagP[row] = rls->P[row + row*rls->n];
        if (diagP[row] > RLS_COV_MAX) {
            lam = 1.f + 0.1f * (1.f - rls->lambda); // attempt return to lower P
            // break; // still need to compute trace
        }
    }

    // P aT, but more efficient to compute a PT = a P for memory access
    float PaT[RLS_MAX_N];
    SGEMVt(rls->n, rls->n, rls->P, aT, PaT);

    // a (P aT)  as dot product
    float aPaT;
    SGEVV(rls->n, aT, PaT, aPaT);

    // output noise estimate
    // if output/regressor noise known, could add those terms here. then it's almost a Kalman filter
    //static float movingVar = 0.f;
    //static float varLambda = 0.995f;
    //float eVarEst = aPaT + movingVar;
    //float ieVarEst = (eVarEst > 1e-6f)  ?  1.f / eVarEst  :  1e6f;

    // eT = yT - a * X
    float eT[RLS_MAX_P];
    //float varRatioMax = 0.f;
    SGEMVt(rls->n, rls->p, rls->X, aT, eT); // not really eT yet
    for (int row = 0; row < rls->p; row++) {
        eT[row] = yT[row] - eT[row];
        //float eTe = (eT[row] * eT[row]);
        // // exponentially weighed moving variance estimator assuming 0 mean
        //movingVar = varLambda*movingVar + (1.f-varLambda) * eTe;
        //varRatioMax = MAX(eTe * ieVarEst, varRatioMax);
    }

    //varRatioMax = MIN(varRatioMax, RLS_FORGET_MAX_VAR);
    //lam = powf(rls->baseLambda, varRatioMax);
    //varLambda = lam; // is this backwards?

    // (lambda + a P aT)**(-1)
    float isig = 1.f / (lam + aPaT);

    // k = P aT * (lambda + a P aT)**(-1)   column vector
    float k[RLS_MAX_N];
    SGEVS(rls->n, PaT, isig, k);

    // in the subtraction below numerical inaccuracies can creep in.
    // limit the order reduction
    float diagKAP[RLS_MAX_N] = {0};
    float maxDiagRatio = 0.f;
    // get trace(k P aT)
    for (int i = 0; i < rls->n; i++) {
        // n dot products of dimension d
        diagKAP[i] += k[i] * PaT[i]; // diagonal of the rank 1 matrix. Always > 0
        if (diagKAP[i] > 1e-6f) {
            diagP[i] /= diagKAP[i];
            maxDiagRatio = (maxDiagRatio < diagP[i]) ? diagP[i] : maxDiagRatio;
        }
    }

    // P = lambda**(-1) (P - k a P) = lambda**(-1) (P - k (P aT)**T)

    // select KAPmult to ensure that traceP - trace(KAP * KAPmult) > 0.1 traceP
    // to maintain numerical accuracy
    // TODO: don't do this on the trace, but on every diagonal element separetely!
    float KAPmult = 1.f;
    if (maxDiagRatio > RLS_COV_MIN)
        KAPmult = MIN((1.f - RLS_MAX_P_ORDER_DECREMENT) * maxDiagRatio, 1.f);

    float ilam = 1.f / lam;
    SGEMM(rls->n, rls->n, 1, k, PaT, rls->P, -KAPmult, -ilam);

    // X = X + k * eT
    SGEMM(rls->n, rls->p, 1, k, eT, rls->X, 1.f, 1.f);

    return RLS_SUCCESS;
}

rls_exit_code_t rlsTest(void) {
    rls_t rls;
    float gamma = 1e3f;
    rlsInit(&rls, 3, 2, gamma, 5000, 1.f / (2.f * M_PIf * 0.011212807f));
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
    rlsParallelInit(&rlsP, 3, 2, gamma, 0.005f, 0.011212807f);
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
