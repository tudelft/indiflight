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

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Idea: while the trajectory tracker's spin mode yaws the craft slowly on
// station, the thrust axis leans into the wind so that its horizontal
// component cancels the wind force. Everything fixed to the body (CG offset,
// hover attitude trim, thrust misalignment, a constant moment that must be
// countered by tilting) rotates with the heading, the wind does not. So the
// part of the tilt that does not depend on the heading points upwind.
//
// Model, per NED horizontal axis j in {N, E}, with the thrust axis (-z of the
// hover frame) t expressed in NED and the heading psi:
//
//   t_j(psi) = c0_j + sum_{k=1..H} ( ck_j cos(k psi) + sk_j sin(k psi) )
//
// fitted by least squares over all samples of the spin session (exponential
// forgetting optional). The two axes share the regressor, so one 5x5 normal
// matrix serves both. Over whole turns at constant rate c0 is just the mean
// tilt vector; the harmonics make it robust against partial turns and an
// uneven yaw rate. They also absorb that the wind force depends on the
// heading (wing broadside vs edge-on: 2nd harmonic) and the lift-like
// cross force at oblique incidence, which averages out over a turn only for a
// laterally symmetric craft.
//
// c0 points into the wind: wind direction (where it comes FROM, 0 deg = N,
// 90 deg = E) is atan2(c0_E, c0_N), |c0| = sin(mean wind tilt). Wind speed is
// not observable without knowing the drag. The co-rotating 1st harmonic is the
// body-fixed tilt (bias), reported in the heading frame (forward, right).

// --- tuning (compile time, can be overridden with -D) ---
#ifndef WIND_EST_HARMONICS
#define WIND_EST_HARMONICS 2        // highest harmonic of the heading in the fit (0: plain mean)
#endif
#ifndef WIND_EST_SAMPLE_HZ
#define WIND_EST_SAMPLE_HZ 50       // inputs are box-averaged down to this rate before the fit
#endif
#ifndef WIND_EST_FORGET_TIME_S
#define WIND_EST_FORGET_TIME_S 0    // exponential forgetting time constant, s. 0: no forgetting (mean over the whole spin session)
#endif
#ifndef WIND_EST_MIN_SWEEP_DEG
#define WIND_EST_MIN_SWEEP_DEG 360  // yaw range covered before the estimate is flagged valid
#endif
#ifndef WIND_EST_MAX_HORZ_SPEED
#define WIND_EST_MAX_HORZ_SPEED 1.0f // m/s, samples are skipped above this ground speed (not on station)
#endif
#ifndef WIND_EST_MAX_TILT_DEG
#define WIND_EST_MAX_TILT_DEG 60    // samples are skipped above this tilt (not a hover)
#endif
#ifndef WIND_EST_RIDGE
#define WIND_EST_RIDGE 1e-4f        // ridge on the harmonic terms (normalised): unexcited harmonics go to 0 instead of making the fit singular
#endif

#define WIND_EST_N (1 + 2 * WIND_EST_HARMONICS)

typedef enum {
    WIND_EST_STATE_IDLE = 0,    // no samples since reset
    WIND_EST_STATE_COLLECTING,  // sampling, yaw range below WIND_EST_MIN_SWEEP_DEG
    WIND_EST_STATE_VALID,       // sampling, estimate valid
    WIND_EST_STATE_HOLD,        // not sampling (spin mode off/paused), holds the last estimate (valid or not, see sweptYaw)
} windEstState_t;

typedef struct {
    // least squares in averaged form: M = mean(phi phi^T), B = mean(phi t^T), yy = mean(t_j^2)
    float M[WIND_EST_N * WIND_EST_N];  // symmetric, both triangles kept
    float B[WIND_EST_N * 2];           // column j: axis j (N, E)
    float yy[2];
    float s2;                          // sum of squared normalised sample weights
    float nEff;                        // effective number of samples, 1 / s2
    uint32_t samples;
    float coef[WIND_EST_N * 2];        // fitted coefficients, column j: axis j
    float yawUnwrapped;
    float yawMin;
    float yawMax;
    bool solved;
} windEstCore_t;

typedef struct {
    windEstState_t state;
    float tilt[2];        // c0: horizontal thrust axis component, NED (N, E), = sin(tilt) * upwind unit vector
    float tiltDeg;        // asin(|c0|)
    float dirDeg;         // direction the wind comes FROM, NED, [0, 360)
    float dirStdDeg;      // formal 1-sigma of dirDeg (white residuals assumed, so optimistic)
    float biasBody[2];    // body-fixed tilt in the heading frame (forward, right), same unit as tilt
    float sweptYawDeg;    // range of heading covered since reset
    uint32_t samples;
} windEstimate_t;

// pure math, no flight controller state (host testable)
void windEstCoreReset(windEstCore_t *c);
// add one sample: heading psi (rad), thrust axis horizontal components tN, tE.
// alphaMin: smallest weight of the new sample (exponential forgetting), 0 for none
void windEstCoreAddSample(windEstCore_t *c, float psi, float tN, float tE, float alphaMin);
// solve the fit and fill est (except state). Returns false if not solvable.
bool windEstCoreSolve(windEstCore_t *c, windEstimate_t *est);

#ifdef USE_TRAJECTORY_TRACKER
#include "common/time.h"

// flight controller glue
void windEstimatorReset(void);           // start a new session (trajectory tracker spin mode start)
void updateWindEstimator(timeUs_t current, bool sampling); // call at the position control rate
const windEstimate_t *getWindEstimate(void);
#endif
