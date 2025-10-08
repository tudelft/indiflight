
#include "ortho_signals.h"
#include "common/time.h"
#include "common/maths.h"

#include <math.h>
#include <stdlib.h>

ortho_signal_t orthoSignal = {
    .K = 4,
    .tf = 0.5f, // time shrink factor: 0.5f means the signal is compressed to half the time
    .base_type = ORTHO_BASE_CHIRP,
    .base_param = 12.566371f,
    .transform_types = {
        ORTHO_TRANS_NONE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
    },
    .transform_params = {
        1.f,
        0.85f,
        0.85f*0.85f,
        0.85f*0.85f*0.85f,
    },
    .alpha = {0.418543f, 0.245190f, 0.118029f, 0.236798f},
    .beta = {0.400000f, 0.450956f, -0.005313f, 0.015259f},
    .dependency = {-1, -1, 0, 1},  // -1 for independent, otherwise index of dependency
    .mixing_matrix = {
        0.477849f, -0.502383f, 0.744380f, 1.573077f, -2.297061f, 1.723984f, -1.744169f, 4.450476f, -4.672701f, 2.316842f
    }
};


static float monomialSignal(ortho_base_e type, float param, float t) {
    switch (type) {
        case ORTHO_BASE_POLYNOMIAL:
            return t * param; // Linear polynomial
        case ORTHO_BASE_SINE:
            return cosf(param * t); // Sine wave
        case ORTHO_BASE_NOISE:
            return (2.0f * ((float)rand() / RAND_MAX) - 1.0f); // Random noise
        case ORTHO_BASE_CHIRP:
            return cosf(param * (1.f-t) * (1.f-t)); // Chirp signal
        default:
            return 0.f; // Default case
    }
}

static float transformSignal(ortho_base_e type, float base_param, ortho_trans_e transform, float param, float t) {

    float mono;
    switch (transform) {
        case ORTHO_TRANS_POWER:
            mono = monomialSignal(type, base_param, t);
            return powf(mono, param);
        case ORTHO_TRANS_SCALE:
            mono = monomialSignal(type, base_param, t * param);
            return mono;
        case ORTHO_TRANS_TIMESHIFT:
            mono = monomialSignal(type, base_param, t + param);
            return mono;
        case ORTHO_TRANS_AMPSHIFT:
            mono = monomialSignal(type, base_param, t);
            return mono + param;
        default:
            return monomialSignal(type, base_param, t); // Default case
    }
}

bool orthoSignalGenerate(float t, float* out, int n) {
    float v[ORTHO_SIGNAL_MAX]; // basis functions and transforms
    float b[ORTHO_SIGNAL_MAX]; // orthonormalized basis functions

    if (n > orthoSignal.K) {
        n = orthoSignal.K;
    }

    // apply time shrink
    t /= orthoSignal.tf;
    if ((t < 0.f) || (t > 1.f)) {
        for (int i = 0; i < n; i++) {
            out[i] = 0.f;
        }
        return false;
    }

    // Generate the orthogonal signal basis functions
    for (int i = 0; i < n; i++) {
        v[i] = transformSignal(
            orthoSignal.base_type,
            orthoSignal.base_param,
            orthoSignal.transform_types[i],
            orthoSignal.transform_params[i],
            t);

        // diagonal
        int diagIdx = ((i+1)*(i+2) >> 1) - 1;
        b[i] = orthoSignal.mixing_matrix[diagIdx] * v[i];

        for (int j = 0; j < i; j++) {
            b[i] += orthoSignal.mixing_matrix[diagIdx - (i-j)] * v[j];
        }

        // apply scaling and offset
        int ii = orthoSignal.dependency[i];
        if (ii == -1) {
            out[i] = orthoSignal.alpha[i] * b[i] + orthoSignal.beta[i];
        } else if (ii < i) {
            // legal, but still check divisor
            float ioutii = fabsf(out[ii]) > 1e-3f ? 1.f / out[ii] : 1e3f;
            out[i] = orthoSignal.alpha[i] * b[i]*ioutii + orthoSignal.beta[i];
        } else {
            out[i] = 0.f; // illegal dependency
        }
    }
    return true;
}
