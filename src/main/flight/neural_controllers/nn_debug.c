#include "nn_debug.h"

#include <math.h>

#include "build/debug.h"
#include "nn_controller.h"

// nn_features is static inside nn_controller.c, so instead of reading it we
// note what was handed over and what a reset put back.
static int16_t nn_debug_dist_freespace = 0;   // x1000
static int16_t nn_debug_feature0       = 0;   // x10000

static int16_t nn_debug_clamp(float v)
{
    if (v >  32767.0f) { return  32767; }
    if (v < -32768.0f) { return -32768; }
    return (int16_t)lrintf(v);
}

void nn_debug_on_reset(void)
{
#ifdef NN_FEATURE_DIM
    nn_debug_dist_freespace = 0;
    nn_debug_feature0       = nn_debug_clamp(features_freespace[0] * 10000.0f);
#endif
}

void nn_debug_on_features(const float *feat)
{
#ifdef NN_FEATURE_DIM
    float d2 = 0.0f;
    for (int i = 0; i < NN_FEATURE_DIM; ++i) {
        float d = feat[i] - features_freespace[i];
        d2 += d * d;
    }
    nn_debug_dist_freespace = nn_debug_clamp(sqrtf(d2) * 1000.0f);
    nn_debug_feature0       = nn_debug_clamp(feat[0] * 10000.0f);
#else
    (void)feat;
#endif
}

void nn_debug_update(int status)
{
    if (debugMode != DEBUG_NN) { return; }

#ifdef NN_FEATURE_DIM
    uint32_t age = nn_features_age();
    DEBUG_SET(DEBUG_NN, 0, age > 32767u ? 32767 : (int16_t)age);
#endif
    DEBUG_SET(DEBUG_NN, 1, (int16_t)status);
    DEBUG_SET(DEBUG_NN, 2, nn_debug_dist_freespace);
    DEBUG_SET(DEBUG_NN, 3, nn_debug_feature0);
}
