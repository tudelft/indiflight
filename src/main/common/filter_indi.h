/*
 * This file is part of Indiflight.
 *
 * Filters INDI needs on top of upstream common/filter.h:
 * - coefficient helpers that set up an upstream phaseComp_t (first-order
 *   section b0,b1,a1) as a zero-pole, motor lead-lag or static gain filter
 * - svfLowpassFilterSetState() to put an upstream SVF lowpass into steady state
 * - a generic second-order section with user-set coefficients (INDI
 *   rate/attitude lead-lag and feedforward), which upstream does not have
 *
 * 2nd order Butterworth lowpass filters use upstream svfLowpassFilter_t.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#pragma once

#include "common/filter.h"

// H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2), direct form II transposed
typedef struct sos2Filter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2;
} sos2Filter_t;

void sos2FilterInit(sos2Filter_t *filter, float b0, float b1, float b2, float a1, float a2);
float sos2FilterApply(sos2Filter_t *filter, float input);

// first-order sections on upstream phaseComp_t; dt in seconds, bilinear transform
void phaseCompInitGain(phaseComp_t *filter, float k);
void phaseCompInitZeroPole(phaseComp_t *filter, float k, float z, float p, float dt);       // k (s - z) / (s - p)
void phaseCompInitMotorLeadLag(phaseComp_t *filter, float tauEst, float tauDes, float dt); // (tauEst s + 1) / (tauDes s + 1)

// steady state of the SVF lowpass at value: output == value for input == value
void svfLowpassFilterSetState(svfLowpassFilter_t *filter, float value);
