/*
 * This file is part of Indiflight.
 *
 * Biquad filters, see filter_biquad.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/filter_biquad.h"

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

/* sets up a biquad filter as a 2nd order butterworth LPF */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}

// Initialize a biquad filter based on chosen gain, zero and pole location
void biquadFilterInitZeroPole(biquadFilter_t *filter, float k, float z, float p, uint32_t refreshRate)
{
    /* Continuous-time filter:
        G(s) = K * (s - z) / (s - p)

    Discretization using bilinear transform (s -> (2/Ts) * (1 - z^-1) / (1 + z^-1)):
        Numerator: (s - z) ->
            (2/Ts * (1 - z^-1) / (1 + z^-1) - z)
        Denominator: (s - p) ->
            (2/Ts * (1 - z^-1) / (1 + z^-1) - p)
        Multiply top and bottom by (1 + z^-1):
        G(z) = K *   ((c - z) * z + (-c - z))
                    --------------------------
                     ((c - p) * z + (-c - p)) 
        where c = 2 / Ts, Ts = refreshRate * 1e-6
    */

    float Ts = refreshRate * 0.000001f;
    float c = 2.0f / Ts;
    float b0 = k * (c - z);
    float b1 = k * (-c - z);
    float a0 = (c - p);
    float a1 = (-c - p);

    float b0n = b0 / a0;
    float b1n = b1 / a0;
    float a1n = a1 / a0;
    biquadFilterInitLeadLag(filter, b0n, b1n, 0.0f, a1n, 0.0f);
}


// Initialize lead or lag filter A_comp(z) to modify time constant of motor from tauEst [s] to tauDes [s]
// Here the desired actuator dynamics should be A_des(s) = A_est(s) * A_comp(s) where A_est(s) is the estimated
// actuator dynamics modeled as a first order lowpass filter.
void biquadFilterInitMotorLeadLag(biquadFilter_t *filter, float tauEst, float tauDes, uint32_t refreshRate)
{
    /* Compute coefficients based on motor being a first order lowpass filter A(s) with time constant tauEst:
    A(s) = 1 / (tauEst * s + 1)
    A_des(s) = A(s) * A_comp(s) = A(s) * (tauEst * s + 1) / (tauDes * s + 1)
    Discretization of A_comp(s) using bilinear transform yields:
    A_comp(z) =  ((2 * tau_est / Ts) + 1) * z + (1 - (2 * tau_est / Ts)) 
                ---------------------------------------------------------
                 ((2 * tau_des / Ts) + 1) * z + (1 - (2 * tau_des / Ts))
    */
    float Ts = refreshRate * 0.000001f;
    float a0 = 2 * tauDes / Ts + 1;
    float a1 = 1 - 2 * tauDes / Ts;
    float b0 = 2 * tauEst / Ts + 1;
    float b1 = 1 - 2 * tauEst / Ts;

    float a1n = a1 / a0;
    float b0n = b0 / a0;
    float b1n = b1 / a0;
    biquadFilterInitLeadLag(filter, b0n, b1n, 0.0f, a1n, 0.0f);
}

/* Setups up a biquad filter as a lead-lag filter in the form H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)*/
void biquadFilterInitLeadLag(biquadFilter_t *filter, float b0, float b1, float b2, float a1, float a2)
{
    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y2 = 0.0f;
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;
    filter->weight = 1.0f; 
}

void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        filter->b1 = 1 - cs;
        filter->b0 = filter->b1 * 0.5f;
        filter->b2 = filter->b0;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        filter->b0 = 1;
        filter->b1 = -2 * cs;
        filter->b2 = 1;
        filter->a1 = filter->b1;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_BPF:
        filter->b0 = alpha;
        filter->b1 = 0;
        filter->b2 = -alpha;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    filter->weight = weight;
}

FAST_CODE void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
FAST_CODE float biquadFilterApplyDF1Weighted(biquadFilter_t* filter, float input)
{
    // compute result
    const float result = biquadFilterApplyDF1(filter, input);

    // crossfading of input and output to turn filter on/off gradually
    return filter->weight * result + (1 - filter->weight) * input;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}
