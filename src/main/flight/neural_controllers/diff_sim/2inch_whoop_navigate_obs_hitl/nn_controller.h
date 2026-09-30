#ifndef NN_CONTROLLER_H
#define NN_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include "neural_network.h"

#define NUM_TARGETS   1
#define TARGET_RADIUS 0.3f   // advance to the next waypoint inside this radius (m)
#define TARGET_LOOP   0    // 1: wrap to the first waypoint, 0: hold the last

// Control steps the FC will fly on a held feature vector before saying so.
#define NN_FEATURE_MAX_AGE 15   // 3 camera frames at 20 Hz

#define NN_STATUS_OK           0
#define NN_STATUS_STALE        1   // flying on a held vector, older than the cap
#define NN_STATUS_NO_FEATURES  2   // nothing ever arrived: using free space

// The encoder output for an empty scene, baked in so the FC has an
// in-distribution feature vector before the first one arrives.
extern const float features_freespace[NN_FEATURE_DIM];

// Mutable, so a groundstation can retarget in flight. Initialised to the
// waypoints baked in at generation time.
extern float target_pos[NUM_TARGETS][3];
extern const float start_pos[3];
extern const float start_yaw;
extern uint8_t target_index;

void nn_set_target(uint8_t index, const float p[3]);

// Reset ALL controller memory: waypoint index, GRU hidden state, features.
// Call once on the rising edge of the NN flight mode. The hidden state is
// only advanced inside nn_control(), so as long as nn_control() is not
// called while the mode is inactive, no history accumulates.
void nn_reset(void);

// Hand over the latest CNN features from the companion. This is the ONLY
// interface to that half: call it from wherever your receive path lands,
// at whatever rate it lands, and nn_control() will read whatever is
// current. Framing, checksums and ordering belong above this call --
// the controller trusts the vector it is given.
void nn_set_features(const float feat[NN_FEATURE_DIM]);

// Control steps since the last nn_set_features(). Also drives the
// NN_STATUS_STALE return below.
uint32_t nn_features_age(void);

// One control step, at the training rate (100 Hz). Returns
// NN_STATUS_*: the motor commands are always written, the status says how
// fresh the depth information behind them is.
int nn_control(const float world_state[16], float motor_cmds[4]);

#endif
