/*
 * This file is part of Indiflight.
 *
 * Biquad filters (removed from upstream common/filter.{c,h}), kept as an
 * indiflight-owned file. Taken from Betaflight 4.4 + indiflight additions.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#pragma once

#include <stdint.h>

#include "common/filter.h"

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInitMotorLeadLag(biquadFilter_t *filter, float tauEst, float tauDes, uint32_t refreshRate);
void biquadFilterInitZeroPole(biquadFilter_t *filter, float k, float z, float p, uint32_t refreshRate);
void biquadFilterInitLeadLag(biquadFilter_t *filter, float b0, float b1, float b2, float a1, float a2);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);

float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApplyDF1Weighted(biquadFilter_t *filter, float input);
float biquadFilterApply(biquadFilter_t *filter, float input);
