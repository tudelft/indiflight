
#include "ortho_signals.h"
#include "common/time.h"
#include "common/maths.h"

#include <math.h>
#include <stdlib.h>

ortho_signal_t orthoSignal = {
    .K = 6,
    .base_param = 20.0f,
    .base_type = ORTHO_BASE_SINE,
    .transform_type = {
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE,
        ORTHO_TRANS_SCALE
    },
    .trans_param = {
        0.9f,
        0.9f,
        0.9f,
        0.9f,
        0.9f,
        0.9f,
    },
    .mixing_matrix = {1.401225, -0.735259, 1.799495, 1.178102, -1.937457, 2.577425, -1.491344, 3.059466, -4.132516, 3.753533, 1.573364, -4.493330, 8.461420, -10.251562, 6.940148, -3.152923, 11.829003, -27.703880, 42.777754, -40.674826, 18.913716} // row major order
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
            return cosf(param * t * t); // Chirp signal
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
            mono = param * orthoSignalMonomial(type, base_param, t*param);
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

void orthoSignalGenerate(float t, float* out) {
    float v[ORTHO_SIGNAL_MAX];

    // Generate the orthogonal signal basis functions
    for (int i = 0; i < orthoSignal.K; i++) {
        v[i] = orthoTransform(
            orthoSignal.base_type,
            orthoSignal.base_param,
            orthoSignal.transform_type[i],
            orthoSignal.trans_param[i],
            t);

        // diagonal
        int diagIdx = ((i+1)*(i+2) >> 1) - 1;
        out[i] = orthoSignal.mixing_matrix[diagIdx] * v[i];

        for (int j = 0; j < i; j++) {
            out[i] += orthoSignal.mixing_matrix[diagIdx - (i-j)] * v[j];
        }
    }
}
