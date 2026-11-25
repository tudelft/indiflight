/*
 * This file is part of Indiflight.
 *
 * See filter_indi.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include "platform.h"

#include "common/filter_indi.h"

void sos2FilterInit(sos2Filter_t *filter, float b0, float b1, float b2, float a1, float a2)
{
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
}

FAST_CODE float sos2FilterApply(sos2Filter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

static void phaseCompSet(phaseComp_t *filter, float b0, float b1, float a1)
{
    filter->b0 = b0;
    filter->b1 = b1;
    filter->a1 = a1;
    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
}

void phaseCompInitGain(phaseComp_t *filter, float k)
{
    phaseCompSet(filter, k, 0.0f, 0.0f);
}

void phaseCompInitZeroPole(phaseComp_t *filter, float k, float z, float p, float dt)
{
    // G(s) = k (s - z) / (s - p), s -> c (1 - z^-1) / (1 + z^-1), c = 2 / dt
    const float c = 2.0f / dt;
    const float a0 = c - p;
    phaseCompSet(filter, k * (c - z) / a0, k * (-c - z) / a0, (-c - p) / a0);
}

void phaseCompInitMotorLeadLag(phaseComp_t *filter, float tauEst, float tauDes, float dt)
{
    // A_comp(s) = (tauEst s + 1) / (tauDes s + 1): motor time constant tauEst -> tauDes
    const float a0 = 2.0f * tauDes / dt + 1.0f;
    phaseCompSet(filter, (2.0f * tauEst / dt + 1.0f) / a0, (1.0f - 2.0f * tauEst / dt) / a0, (1.0f - 2.0f * tauDes / dt) / a0);
}

void svfLowpassFilterSetState(svfLowpassFilter_t *filter, float value)
{
    // input = value: v3 = 0, v1 = 0, v2 = ic2 = value, and the states stay put
    filter->ic1 = 0.0f;
    filter->ic2 = value;
}
