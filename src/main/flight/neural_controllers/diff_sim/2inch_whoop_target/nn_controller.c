#include "nn_controller.h"
#include <math.h>

// ---------------------------------------------------------------------
// Conventions inherited from training (JADS/tasks/hover_real.py)
//   world_state[0:3]   position      NED, metres
//   world_state[3:6]   velocity      NED, m/s, world frame
//   world_state[6:9]   roll, pitch, yaw   rad
//   world_state[9:12]  body rates    rad/s
//   world_state[12:16] motor speeds  rad/s (see MOTOR_OMEGA_SCALE)
// ---------------------------------------------------------------------

#define W_MAX_N     5000.0f   // motor speed mapping to obs = +1 (rad/s)
// Set to (2*M_PI/60) if world_state[12:16] arrives in RPM instead of rad/s.
#define MOTOR_OMEGA_SCALE 1.0f
// Yaw of the training world frame +x axis expressed in the estimator frame.
// Nonzero if the EKF heading origin is not the training origin.
#define WORLD_YAW_OFFSET 0.0f

// motor_order[i] is the indiflight motor carrying training motor i.
// The identified coefficients are per-motor and asymmetric, so a wrong
// order flies badly rather than obviously. Check against the mixer.
static const uint8_t motor_order[4] = {0, 1, 2, 3};

const float target_pos[NUM_TARGETS][3] = {
    {3.0f, 0.0f, -1.5f},
};

// Expected arming pose in the training world frame. Not used by
// nn_control(); exported for the indiflight side to position/check against.
const float start_pos[3] = {
    -3.0f, 0.0f, -1.5f
};

const float start_yaw = 0.0f;

uint8_t target_index = 0;
static float nn_hidden[NN_HIDDEN_DIM];

void nn_reset(void) {
    target_index = 0;
    nn_reset_hidden(nn_hidden);
}

void nn_control(const float world_state[16], float motor_cmds[4]) {
    // Advance the waypoint once we are inside the capture radius.
    float dx = world_state[0] - target_pos[target_index][0];
    float dy = world_state[1] - target_pos[target_index][1];
    float dz = world_state[2] - target_pos[target_index][2];
    if (dx*dx + dy*dy + dz*dz < TARGET_RADIUS * TARGET_RADIUS) {
#if TARGET_LOOP
        target_index = (uint8_t)((target_index + 1) % NUM_TARGETS);
#else
        if (target_index + 1 < NUM_TARGETS) { target_index++; }
#endif
        dx = world_state[0] - target_pos[target_index][0];
        dy = world_state[1] - target_pos[target_index][1];
        dz = world_state[2] - target_pos[target_index][2];
    }

    float obs[NN_OBS_DIM];
    // position error, world frame (no target-frame rotation, unlike the gates)
    obs[0] = dx;
    obs[1] = dy;
    obs[2] = dz;
    // velocity, world frame
    obs[3] = world_state[3];
    obs[4] = world_state[4];
    obs[5] = world_state[5];
    // attitude
    obs[6] = world_state[6];
    obs[7] = world_state[7];
    float yaw = world_state[8] - WORLD_YAW_OFFSET;
    while (yaw >  (float)M_PI) { yaw -= 2.0f * (float)M_PI; }
    while (yaw < -(float)M_PI) { yaw += 2.0f * (float)M_PI; }
    obs[8] = yaw;
    // body rates
    obs[9]  = world_state[9];
    obs[10] = world_state[10];
    obs[11] = world_state[11];
    // motor speeds scaled to [-1, 1]: w = 2 * W / W_MAX_N - 1
    for (int i = 0; i < 4; ++i) {
        float W = world_state[12 + motor_order[i]] * MOTOR_OMEGA_SCALE;
        obs[12 + i] = 2.0f * W / W_MAX_N - 1.0f;
    }

    // Advances nn_hidden. This call is the ONLY writer of the hidden state.
    float action[NN_ACT_DIM];
    nn_forward(obs, nn_hidden, action);

    for (int i = 0; i < 4; ++i) {
        // clip to [-1, 1.0] (1.0 MOTOR LIMIT), then map [-1,1] -> [0,1]
        float a = action[i];
        if (a > 1.0f) { a = 1.0f; }
        if (a < -1.0f) { a = -1.0f; }
        motor_cmds[motor_order[i]] = (a + 1.0f) * 0.5f;
    }
}
