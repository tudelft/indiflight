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

// See wind_estimator.h for the method. RAM: about 0.8 KB.

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

bool windEstCoreSolve(windEstCore_t *c, windEstFit_t *fit)
{
    fit->samples = c->samples;
    fit->sweptYawDeg = (c->yawMax - c->yawMin) * (180.f / M_PIf);
    fit->solved = c->solved = false;

    if (c->samples == 0) {
        return false;
    }

    float A[WIND_EST_N * WIND_EST_N];
    memcpy(A, c->M, sizeof(A));
    for (int i = 1; i < WIND_EST_N; i++) {
        A[i + i*WIND_EST_N] += WIND_EST_RIDGE;
    }

    float L[WIND_EST_N * WIND_EST_N];
    if (!cholesky(A, L, WIND_EST_N)) {
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

    fit->tilt[0] = c->coef[0];
    fit->tilt[1] = c->coef[WIND_EST_N];

    // formal standard error of c0 (per component), assuming white residuals.
    // Consecutive samples are correlated (turbulence, controller), so this is
    // optimistic; use it as a relative quality measure.
    fit->tiltStd = 1.f;
    const float dof = c->nEff - (float) WIND_EST_N;
    if (dof > 1.f) {
        const float sigma2 = mse * c->nEff / dof;
        fit->tiltStd = sqrtf(fmaxf(sigma2 * z[0] / c->nEff, 0.f));
    }

    // body-fixed tilt: co-rotating part of the 1st harmonic, R(psi) b
#if WIND_EST_HARMONICS > 0
    const float c1N = c->coef[1], s1N = c->coef[2];
    const float c1E = c->coef[1 + WIND_EST_N], s1E = c->coef[2 + WIND_EST_N];
    fit->biasBody[0] = 0.5f * (c1N + s1E);
    fit->biasBody[1] = 0.5f * (c1E - s1N);
#else
    fit->biasBody[0] = fit->biasBody[1] = 0.f;
#endif

    fit->solved = c->solved = true;
    return true;
}

static bool fitValid(const windEstFit_t *f)
{
    return f->solved && (f->sweptYawDeg >= WIND_EST_MIN_SWEEP_DEG);
}

void windEstCombine(const windEstFit_t fits[WIND_EST_SPIN_COUNT], windEstimate_t *est)
{
    const windEstFit_t *p = &fits[WIND_EST_SPIN_POS];
    const windEstFit_t *n = &fits[WIND_EST_SPIN_NEG];
    const bool vp = fitValid(p);
    const bool vn = fitValid(n);

    est->state &= WIND_EST_SAMPLING;
    est->state |= (vp ? WIND_EST_VALID_POS : 0) | (vn ? WIND_EST_VALID_NEG : 0);
    for (int d = 0; d < WIND_EST_SPIN_COUNT; d++) {
        est->sweptYawDeg[d] = fits[d].sweptYawDeg;
        est->samples[d] = fits[d].samples;
    }

    float tilt[2], bias[2], std;
    if (vp && vn) {
        // average: the spin offset has opposite signs in the two directions
        for (int j = 0; j < 2; j++) {
            tilt[j] = 0.5f * (p->tilt[j] + n->tilt[j]);
            bias[j] = 0.5f * (p->biasBody[j] + n->biasBody[j]);
        }
        std = 0.5f * sqrtf(p->tiltStd * p->tiltStd + n->tiltStd * n->tiltStd);
    } else {
        // one direction (or none valid yet): the valid one, else the one with
        // the larger heading range, so the log shows the estimate converging
        const windEstFit_t *f;
        if (vp != vn) {
            f = vp ? p : n;
        } else if (p->solved != n->solved) {
            f = p->solved ? p : n;
        } else {
            f = (p->sweptYawDeg >= n->sweptYawDeg) ? p : n;
        }
        if (!f->solved) {
            est->tilt[0] = est->tilt[1] = est->biasBody[0] = est->biasBody[1] = 0.f;
            est->tiltDeg = est->dirDeg = 0.f;
            est->dirStdDeg = 180.f;
            return;
        }
        for (int j = 0; j < 2; j++) {
            tilt[j] = f->tilt[j];
            bias[j] = f->biasBody[j];
        }
        std = f->tiltStd;
    }

    const float tiltNorm = sqrtf(tilt[0]*tilt[0] + tilt[1]*tilt[1]);
    est->tilt[0] = tilt[0];
    est->tilt[1] = tilt[1];
    est->biasBody[0] = bias[0];
    est->biasBody[1] = bias[1];
    est->tiltDeg = asinf(fminf(tiltNorm, 1.f)) * (180.f / M_PIf);
    float dir = atan2f(tilt[1], tilt[0]) * (180.f / M_PIf);
    if (dir < 0.f) {
        dir += 360.f;
    }
    est->dirDeg = dir;
    est->dirStdDeg = (tiltNorm > std) ? fminf(asinf(std / tiltNorm) * (180.f / M_PIf), 180.f) : 180.f;
}

// --- flight controller glue

#ifdef USE_TRAJECTORY_TRACKER

#include "platform.h"
#include "common/maths.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "flight/indi.h"

static windEstCore_t windCore[WIND_EST_SPIN_COUNT];
static windEstFit_t windFit[WIND_EST_SPIN_COUNT];
static windEstimate_t windEst = { .dirStdDeg = 180.f };

// box average down to WIND_EST_SAMPLE_HZ
static struct {
    float cosPsi;
    float sinPsi;
    float tN;
    float tE;
    float time;
    int dir;          // windEstSpinDir_e of the samples in the window
    timeUs_t lastUs;
    bool haveLast;
} windWin;

static void windWindowClear(void)
{
    windWin.cosPsi = windWin.sinPsi = windWin.tN = windWin.tE = windWin.time = 0.f;
}

void windEstimatorReset(void)
{
    for (int d = 0; d < WIND_EST_SPIN_COUNT; d++) {
        windEstCoreReset(&windCore[d]);
        memset(&windFit[d], 0, sizeof(windFit[d]));
    }
    memset(&windEst, 0, sizeof(windEst));
    windEst.dirStdDeg = 180.f;
    windWindowClear();
}

const windEstimate_t *getWindEstimate(void)
{
    return &windEst;
}

void updateWindEstimator(timeUs_t current, bool sampling, float yawRateSp)
{
    float dt = 0.f;
    if (windWin.haveLast) {
        dt = cmpTimeUs(current, windWin.lastUs) * 1e-6f;
    }
    windWin.lastUs = current;
    windWin.haveLast = true;

    if (!sampling || (fabsf(yawRateSp) < WIND_EST_MIN_YAW_RATE)) {
        // hold the last estimate
        windEst.state &= ~WIND_EST_SAMPLING;
        windWindowClear();
        return;
    }
    windEst.state |= WIND_EST_SAMPLING;

    const int dir = (yawRateSp > 0.f) ? WIND_EST_SPIN_POS : WIND_EST_SPIN_NEG;
    if (dir != windWin.dir) {
        windWindowClear(); // don't mix spin directions in one sample
        windWin.dir = dir;
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
    windEstCoreAddSample(&windCore[dir], psiMean, windWin.tN * iT, windWin.tE * iT, alphaMin);
    windWindowClear();

    windEstCoreSolve(&windCore[dir], &windFit[dir]);
    windEstCombine(windFit, &windEst);
}

#endif // USE_TRAJECTORY_TRACKER
