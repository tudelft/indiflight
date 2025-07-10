
#pragma once

typedef enum {
    ORTHO_BASE_POLYNOMIAL,
    ORTHO_BASE_SINE,
    ORTHO_BASE_NOISE,
    ORTHO_BASE_CHIRP,
    ORTHO_BASE_SINC,
} ortho_base_e;

typedef enum {
    ORTHO_TRANS_NONE,
    ORTHO_TRANS_POWER,
    ORTHO_TRANS_SCALE,
    ORTHO_TRANS_TIMESHIFT,
    ORTHO_TRANS_AMPSHIFT,
} ortho_trans_e;

#define ORTHO_SIGNAL_MAX 6

typedef struct ortho_signal_s {
    int K;
    float base_param;  // parameter for the orthogonal signal (e.g., frequency, amplitude)
    ortho_base_e base_type;  // type of the orthogonal signal base
    ortho_trans_e transform_type[ORTHO_SIGNAL_MAX];  // type of transformation applied to the base signal
    float trans_param[ORTHO_SIGNAL_MAX];  // type of transformation applied to the base signal
    float mixing_matrix[ORTHO_SIGNAL_MAX*(ORTHO_SIGNAL_MAX+1) >> 1];  // mixing matrix obtained from Gram-Schmidt process
} ortho_signal_t;

extern ortho_signal_t orthoSignal;

void orthoSignalGenerate(float t, float* out, int n);