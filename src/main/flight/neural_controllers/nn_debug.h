#pragma once

#include <stdint.h>

// Blackbox DEBUG_NN mode (`set debug_mode = NN`), for checking whether the
// FC is actually receiving fresh feature vectors from the companion.
//
// Kept separate from nn_controller.c/.h on purpose: those are generated and
// get swapped wholesale, so nothing in here may depend on more than their
// public API (features_freespace, nn_features_age(), NN_STATUS_*,
// NN_FEATURE_DIM).
//
//   debug[0]  feature age, control steps since the last vector arrived.
//             Should sawtooth at the camera rate; a ramp means nothing is
//             arriving.
//   debug[1]  NN_STATUS_* returned by nn_control()
//             (0 = OK, 1 = STALE, 2 = NO_FEATURES).
//   debug[2]  L2 distance of the vector in use from features_freespace,
//             x1000. Near-zero means still flying on the baked fallback.
//   debug[3]  first element of the vector in use, x10000.
//
// Controllers without NN_FEATURE_DIM (no companion input) only log
// debug[1] and leave the rest at 0.

// Call right after nn_reset(): the controller is back on features_freespace.
void nn_debug_on_reset(void);

// Call right after nn_set_features(feat) with the same vector.
void nn_debug_on_features(const float *feat);

// Call once per control step with the return value of nn_control().
void nn_debug_update(int status);
