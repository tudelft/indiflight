
#include "ortho_signals.h"
#include "common/time.h"
#include "common/maths.h"

#include <math.h>
#include <stdlib.h>

ortho_signal_t orthoSignal = {
    .K = 4,
    .base_param = 20.0f,
    .base_type = ORTHO_BASE_CHIRP,
    .transform_type = {
        ORTHO_TRANS_NONE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
//         ORTHO_TRANS_SCALE,
//         ORTHO_TRANS_SCALE,
    },
    .trans_param = {
        1.f,
        0.9f,
        0.9f*0.9f,
        0.9f*0.9f*0.9f,
//         0.9f*0.9f*0.9f*0.9f,
//         0.9f*0.9f*0.9f*0.9f*0.9f,
    },
    // .mixing_matrix = {0.236303, -0.013044, 0.237292, 0.029661, -0.044778, 0.243902, -0.050836, 0.072173, -0.095275, 0.262043}
    .mixing_matrix = {0.542061, -0.563899, 0.818948, 1.583174, -2.335288, 1.775406, -1.846006, 4.809987, -4.979564, 2.563008}
};


static float orthoSignalMonomial(ortho_base_e type, float param, float t) {
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

static float orthoTransform(ortho_base_e type, float base_param, ortho_trans_e transform, float param, float t) {

    float mono;
    switch (transform) {
        case ORTHO_TRANS_POWER:
            mono = orthoSignalMonomial(type, base_param, t);
            return powf(mono, param);
        case ORTHO_TRANS_SCALE:
            mono = orthoSignalMonomial(type, base_param, t * param);
            return mono;
        case ORTHO_TRANS_TIMESHIFT:
            mono = orthoSignalMonomial(type, base_param, t + param);
            return mono;
        case ORTHO_TRANS_AMPSHIFT:
            mono = orthoSignalMonomial(type, base_param, t);
            return mono + param;
        default:
            return orthoSignalMonomial(type, base_param, t); // Default case
    }
}

void orthoSignalGenerate(float t, float* out, int n) {
    float v[ORTHO_SIGNAL_MAX];

    if (n > orthoSignal.K) {
        n = orthoSignal.K;
    }

    // Generate the orthogonal signal basis functions
    for (int i = 0; i < n; i++) {
        v[i] = orthoTransform(
            orthoSignal.base_type,
            orthoSignal.base_param,
            orthoSignal.transform_type[i],
            orthoSignal.trans_param[i],
            t);

        // diagonal
        int diagIdx = ((i+1)*(i+2) >> 1) - 1;
        out[i] = 1.f*orthoSignal.mixing_matrix[diagIdx] * v[i];

        for (int j = 0; j < i; j++) {
            out[i] += 1.f*orthoSignal.mixing_matrix[diagIdx - (i-j)] * v[j];
        }
    }
}
