#ifndef NN_CONTROLLER_H
#define NN_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// Include the neural network code
#include "neural_network.h"

// Motor count of the trained airframe, and the world_state length it
// implies (12 rigid-body entries + one motor speed per motor).
#define NN_NUM_MOTORS       6
#define NN_WORLD_STATE_DIM  18

#define NUM_TARGETS   1
#define TARGET_RADIUS 0.3f   // advance to the next waypoint inside this radius (m)
#define TARGET_LOOP   0    // 1: wrap to the first waypoint, 0: hold the last

extern const float target_pos[NUM_TARGETS][3];
extern const float start_pos[3];
extern const float start_yaw;
extern uint8_t target_index;

// Reset ALL controller memory: waypoint index and GRU hidden state.
// Call once on the rising edge of the NN flight mode. The hidden state is
// only advanced inside nn_control(), so as long as nn_control() is not
// called while the mode is inactive, no history accumulates.
void nn_reset(void);

// One control step, at the training rate (100 Hz).
void nn_control(const float world_state[NN_WORLD_STATE_DIM], float motor_cmds[NN_NUM_MOTORS]);

#endif
