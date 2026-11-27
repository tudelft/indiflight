/*
 * Wind direction estimate from the mean tilt while slowly yawing on station
 *
 * Copyright 2026 Till Blaha (Delft University of Technology)
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

// See wind_estimator.h for the method. RAM: about 0.4 KB.

#include <math.h>
#include <string.h>

#include "wind_estimator.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

#define WIND_EST_PIVOT_MIN 1e-9f

static float wrapPi(float a)
{
    while (a >  M_PIf) a -= 2.f * M_PIf;
    while (a < -M_PIf) a += 2.f * M_PIf;
    return a;
}

void windEstCoreReset(windEstCore_t *c)
{
    memset(c, 0, sizeof(*c));
}

void windEstCoreAddSample(windEstCore_t *c, float psi, float tN, float tE, float alphaMin)
{
    psi = wrapPi(psi);

    // heading range covered
    if (c->samples == 0) {
        c->yawUnwrapped = c->yawMin = c->yawMax = psi;
    } else {
        c->yawUnwrapped += wrapPi(psi - wrapPi(c->yawUnwrapped));
        if (c->yawUnwrapped < c->yawMin) c->yawMin = c->yawUnwrapped;
        if (c->yawUnwrapped > c->yawMax) c->yawMax = c->yawUnwrapped;
    }

    // regressor [1, cos psi, sin psi, cos 2psi, sin 2psi, ...]
    float phi[WIND_EST_N];
    phi[0] = 1.f;
#if WIND_EST_HARMONICS > 0
    const float c1 = cosf(psi);
    const float s1 = sinf(psi);
    float ck = c1;
    float sk = s1;
    for (int k = 1; k <= WIND_EST_HARMONICS; k++) {
        phi[2*k - 1] = ck;
        phi[2*k]     = sk;
        const float ckNext = ck * c1 - sk * s1;
        sk = sk * c1 + ck * s1;
        ck = ckNext;
    }
#endif

    // running (weighted) means: weight 1/k gives the plain mean over all
    // samples, alphaMin > 0 caps it into exponential forgetting
    c->samples++;
    float alpha = 1.f / (float) c->samples;
    if (alpha < alphaMin) {
        alpha = alphaMin;
    }

    const float y[2] = { tN, tE };
    for (int col = 0; col < WIND_EST_N; col++) {
        for (int row = col; row < WIND_EST_N; row++) {
            float m = c->M[row + col*WIND_EST_N];
            m += alpha * (phi[row] * phi[col] - m);
            c->M[row + col*WIND_EST_N] = m;
            c->M[col + row*WIND_EST_N] = m;
        }
        for (int j = 0; j < 2; j++) {
            c->B[col + j*WIND_EST_N] += alpha * (phi[col] * y[j] - c->B[col + j*WIND_EST_N]);
        }
    }
    for (int j = 0; j < 2; j++) {
        c->yy[j] += alpha * (y[j] * y[j] - c->yy[j]);
    }

    c->s2 = (1.f - alpha) * (1.f - alpha) * c->s2 + alpha * alpha;
    c->nEff = 1.f / c->s2;
}

// Cholesky A = L L^T (column major, lower), false if not positive definite
static bool cholesky(const float *A, float *L, int n)
{
    memset(L, 0, sizeof(float) * n * n);
    for (int j = 0; j < n; j++) {
        float d = A[j + j*n];
        for (int k = 0; k < j; k++) {
            d -= L[j + k*n] * L[j + k*n];
        }
        if (!(d > WIND_EST_PIVOT_MIN)) { // also catches NaN
            return false;
        }
        const float ljj = sqrtf(d);
        L[j + j*n] = ljj;
        for (int i = j + 1; i < n; i++) {
            float s = A[i + j*n];
            for (int k = 0; k < j; k++) {
                s -= L[i + k*n] * L[j + k*n];
            }
            L[i + j*n] = s / ljj;
        }
    }
    return true;
}

static void choleskySolve(const float *L, int n, const float *b, float *x)
{
    for (int i = 0; i < n; i++) { // L z = b
        float s = b[i];
        for (int k = 0; k < i; k++) {
            s -= L[i + k*n] * x[k];
        }
        x[i] = s / L[i + i*n];
    }
    for (int i = n - 1; i >= 0; i--) { // L^T x = z
        float s = x[i];
        for (int k = i + 1; k < n; k++) {
            s -= L[k + i*n] * x[k];
        }
        x[i] = s / L[i + i*n];
    }
}

bool windEstCoreSolve(windEstCore_t *c, windEstimate_t *est)
{
    est->samples = c->samples;
    est->sweptYawDeg = (c->yawMax - c->yawMin) * (180.f / M_PIf);

    if (c->samples == 0) {
        c->solved = false;
        return false;
    }

    float A[WIND_EST_N * WIND_EST_N];
    memcpy(A, c->M, sizeof(A));
    for (int i = 1; i < WIND_EST_N; i++) {
        A[i + i*WIND_EST_N] += WIND_EST_RIDGE;
    }

    float L[WIND_EST_N * WIND_EST_N];
    if (!cholesky(A, L, WIND_EST_N)) {
        c->solved = false;
        return false;
    }

    for (int j = 0; j < 2; j++) {
        choleskySolve(L, WIND_EST_N, &c->B[j*WIND_EST_N], &c->coef[j*WIND_EST_N]);
    }

    // (A^-1)_00 for the variance of the mean term
    float e0[WIND_EST_N] = { 1.f };
    float z[WIND_EST_N];
    choleskySolve(L, WIND_EST_N, e0, z);

    // mean squared residual: mean(t^2) - coef^T mean(phi t), pooled over N and E
    float mse = 0.f;
    for (int j = 0; j < 2; j++) {
        float r = c->yy[j];
        for (int i = 0; i < WIND_EST_N; i++) {
            r -= c->coef[i + j*WIND_EST_N] * c->B[i + j*WIND_EST_N];
        }
        mse += 0.5f * fmaxf(r, 0.f);
    }

    const float tN = c->coef[0];
    const float tE = c->coef[WIND_EST_N];
    const float tiltNorm = sqrtf(tN*tN + tE*tE);
    est->tilt[0] = tN;
    est->tilt[1] = tE;
    est->tiltDeg = asinf(fminf(tiltNorm, 1.f)) * (180.f / M_PIf);
    float dir = atan2f(tE, tN) * (180.f / M_PIf);
    if (dir < 0.f) {
        dir += 360.f;
    }
    est->dirDeg = dir;

    // formal standard error of c0 (per component), assuming white residuals.
    // Consecutive samples are correlated (turbulence, controller), so this is
    // optimistic; use it as a relative quality measure.
    float dirStd = 180.f;
    const float dof = c->nEff - (float) WIND_EST_N;
    if (dof > 1.f) {
        const float sigma2 = mse * c->nEff / dof;
        const float stdC0 = sqrtf(fmaxf(sigma2 * z[0] / c->nEff, 0.f));
        if (tiltNorm > stdC0) {
            dirStd = fminf(asinf(stdC0 / tiltNorm) * (180.f / M_PIf), 180.f);
        }
    }
    est->dirStdDeg = dirStd;

    // body-fixed tilt: co-rotating part of the 1st harmonic, R(psi) b
#if WIND_EST_HARMONICS > 0
    const float c1N = c->coef[1], s1N = c->coef[2];
    const float c1E = c->coef[1 + WIND_EST_N], s1E = c->coef[2 + WIND_EST_N];
    est->biasBody[0] = 0.5f * (c1N + s1E);
    est->biasBody[1] = 0.5f * (c1E - s1N);
#else
    est->biasBody[0] = est->biasBody[1] = 0.f;
#endif

    c->solved = true;
    return true;
}

// --- flight controller glue

#ifdef USE_TRAJECTORY_TRACKER

#include "platform.h"
#include "common/maths.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "flight/indi.h"

static windEstCore_t windCore;
static windEstimate_t windEst;

// box average down to WIND_EST_SAMPLE_HZ
static struct {
    float cosPsi;
    float sinPsi;
    float tN;
    float tE;
    float time;
    timeUs_t lastUs;
    bool haveLast;
} windWin;

static void windWindowClear(void)
{
    windWin.cosPsi = windWin.sinPsi = windWin.tN = windWin.tE = windWin.time = 0.f;
}

void windEstimatorReset(void)
{
    windEstCoreReset(&windCore);
    memset(&windEst, 0, sizeof(windEst));
    windEst.state = WIND_EST_STATE_IDLE;
    windEst.dirStdDeg = 180.f;
    windWindowClear();
}

const windEstimate_t *getWindEstimate(void)
{
    return &windEst;
}

void updateWindEstimator(timeUs_t current, bool sampling)
{
    float dt = 0.f;
    if (windWin.haveLast) {
        dt = cmpTimeUs(current, windWin.lastUs) * 1e-6f;
    }
    windWin.lastUs = current;
    windWin.haveLast = true;

    if (!sampling) {
        windEst.state = (windCore.samples > 0) ? WIND_EST_STATE_HOLD : WIND_EST_STATE_IDLE;
        windWindowClear();
        return;
    }

    if ((windEst.state == WIND_EST_STATE_IDLE) || (windEst.state == WIND_EST_STATE_HOLD)) {
        windEst.state = (windCore.solved && (windEst.sweptYawDeg >= WIND_EST_MIN_SWEEP_DEG))
            ? WIND_EST_STATE_VALID : WIND_EST_STATE_COLLECTING;
    }

    if ((dt <= 0.f) || (dt > 0.1f)) {
        return; // first call or a gap: no time weight for this one
    }

    // only on station
    if (VEC3_XY_LENGTH(velEstNed) > WIND_EST_MAX_HORZ_SPEED) {
        return;
    }

    // thrust axis = -z of the hover frame, in NED. Level hover: bodyZNed = (0, 0, 1)
    fp_quaternion_t qHover;
    getHoverAttitudeQuaternion(&qHover);
    const fp_vector_t bodyZNed = quatRotMatCol(&qHover, 2);
    if (bodyZNed.V.Z < cosf(DEGREES_TO_RADIANS(WIND_EST_MAX_TILT_DEG))) { // cos(tilt)
        return;
    }

    const float psi = getYawWithoutSingularity();
    windWin.cosPsi += dt * cosf(psi);
    windWin.sinPsi += dt * sinf(psi);
    windWin.tN     += dt * (-bodyZNed.V.X);
    windWin.tE     += dt * (-bodyZNed.V.Y);
    windWin.time   += dt;

    if (windWin.time < (1.f / WIND_EST_SAMPLE_HZ)) {
        return;
    }

    const float iT = 1.f / windWin.time;
    const float psiMean = atan2f(windWin.sinPsi, windWin.cosPsi);
    const float alphaMin = (WIND_EST_FORGET_TIME_S > 0)
        ? 1.f / ((float) WIND_EST_FORGET_TIME_S * WIND_EST_SAMPLE_HZ) : 0.f;
    windEstCoreAddSample(&windCore, psiMean, windWin.tN * iT, windWin.tE * iT, alphaMin);
    windWindowClear();

    if (windEstCoreSolve(&windCore, &windEst) && (windEst.sweptYawDeg >= WIND_EST_MIN_SWEEP_DEG)) {
        windEst.state = WIND_EST_STATE_VALID;
    } else {
        windEst.state = WIND_EST_STATE_COLLECTING;
    }
}

#endif // USE_TRAJECTORY_TRACKER
