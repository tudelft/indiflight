#include "nn_controller.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Conventions inherited from training (JADS/tasks/navigate_real.py)
//   world_state[0:3]   position      NED, metres
//   world_state[3:6]   velocity      NED, m/s, world frame
//   world_state[6:9]   roll, pitch, yaw   rad
//   world_state[9:12]  body rates    rad/s
//   world_state[12:18] motor speeds  rad/s (see MOTOR_OMEGA_SCALE)
// ---------------------------------------------------------------------------

#define W_MAX_N     4000.0f   // motor speed mapping to obs = +1 (rad/s)
// Set to (2*M_PI/60) if world_state[12:16] arrives in RPM instead of rad/s.
#define MOTOR_OMEGA_SCALE 1.0f
#define MOTOR_CMD_MAX 1.0f   // 1.0 motor limit, in [-1, 1] units
// Yaw of the training world frame +x axis expressed in the estimator frame.
// Nonzero if the EKF heading origin is not the training origin.
#define WORLD_YAW_OFFSET 0.0f

// motor_order[i] is the indiflight motor carrying training motor i.
// The identified coefficients are per-motor and asymmetric, so a wrong
// order flies badly rather than obviously. Check against the mixer.
static const uint8_t motor_order[6] = {0, 1, 2, 3, 4, 5};

float target_pos[NUM_TARGETS][3] = {
    {3.25f, 0.0f, -1.0f},
};

// Expected arming pose in the training world frame. Not used by
// nn_control(); exported for the indiflight side to position/check against.
const float start_pos[3] = {
    -3.25f, 0.0f, -1.0f
};

const float start_yaw = 0.0f;

const float features_freespace[] = {
    -0.145947874f, 0.540325999f, -0.307154745f, -0.680087566f, -0.0119008999f, 0.122020222f, -0.404148757f, -0.370422304f, 0.0945681781f, -0.355282366f, -0.0905494615f, 0.2092731f,
    0.173121095f, 0.119331293f, -0.189711079f, -0.207872406f, 0.79685992f, 0.296602577f, 0.479117125f, 0.0454734825f, 0.148873225f, 0.764538705f, 0.189659506f, 0.546174109f,
    -0.370996296f, -0.373209357f, 0.710771322f, 0.621634662f, -0.621029615f, 0.881854355f, -0.290433109f, 0.145432368f, 0.306297213f, -1.1954354f, 0.804680049f, -0.0456464402f,
    0.445437551f, 0.69818908f, -0.535972297f, -0.186873168f, -0.178250253f, 0.257198066f, 0.542786658f, -0.326333523f, -0.459024161f, 0.268875986f, -0.568891048f, -0.130068764f,
    -0.626365364f, -0.255895764f, 0.333914459f, 0.184003338f, 0.102049261f, 0.421885818f, 0.523681641f, -0.69059515f, 0.30737868f, -0.802259088f, -0.639752388f, -0.594847202f,
    -0.663417637f, 0.365930885f, -0.583878398f, -0.391922414f
};

static float    nn_hidden[NN_HIDDEN_DIM];
static float    nn_features[NN_FEATURE_DIM];
static uint32_t nn_feature_age  = 0;    // control steps since the last nn_set_features()
static bool     nn_have_feature = false;

uint8_t target_index = 0;

void nn_set_target(uint8_t index, const float p[3])
{
    if (index < NUM_TARGETS) {
        target_pos[index][0] = p[0];
        target_pos[index][1] = p[1];
        target_pos[index][2] = p[2];
    }
}

void nn_reset(void)
{
    target_index = 0;
    nn_reset_hidden(nn_hidden);
    // Seed with the encoder's own output for an empty scene, not with zeros:
    // zeros are not a vector the CNN can produce.
    memcpy(nn_features, features_freespace, sizeof(nn_features));
    nn_feature_age  = 0;
    nn_have_feature = false;
}

// The whole interface to the companion half. Whatever transport delivers the
// vector, this is where it lands; validating it is the caller's job.
void nn_set_features(const float feat[NN_FEATURE_DIM])
{
    memcpy(nn_features, feat, sizeof(nn_features));
    nn_feature_age  = 0;
    nn_have_feature = true;
}

uint32_t nn_features_age(void) { return nn_feature_age; }

int nn_control(const float world_state[NN_OBS_DIM], float motor_cmds[NN_ACT_DIM])
{
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
    // position error, world frame (no target-frame rotation)
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
    for (int i = 0; i < NN_ACT_DIM; ++i) {
        float W = world_state[12 + motor_order[i]] * MOTOR_OMEGA_SCALE;
        obs[12 + i] = 2.0f * W / W_MAX_N - 1.0f;
    }

    // Advances nn_hidden. This call is the ONLY writer of the hidden state.
    // nn_features is held between camera frames, which is what training did.
    float action[NN_ACT_DIM];
    nn_forward(nn_features, obs, nn_hidden, action);

    for (int i = 0; i < NN_ACT_DIM; ++i) {
        // clip, then map [-1, 1] -> [0, 1]
        float a = action[i];
        if (a >  MOTOR_CMD_MAX) { a = MOTOR_CMD_MAX; }
        if (a < -1.0f)          { a = -1.0f; }
        motor_cmds[motor_order[i]] = (a + 1.0f) * 0.5f;
    }

    if (nn_feature_age < 0xFFFFFFFFu) { nn_feature_age++; }
    if (!nn_have_feature)                    { return NN_STATUS_NO_FEATURES; }
    if (nn_feature_age > NN_FEATURE_MAX_AGE) { return NN_STATUS_STALE; }
    return NN_STATUS_OK;
}
