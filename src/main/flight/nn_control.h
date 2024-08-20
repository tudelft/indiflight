#ifndef NN_CONTROL_H
#define NN_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pg/pg.h"
#include "flight/pos_ctl.h"

typedef struct nnConfig_s {
    uint8_t rate_denom;     // run net every rate_denom times the inner loop is run
} nnConfig_t;

PG_DECLARE(nnConfig_t, nnConfig);

#define NN_DEADRECKONING_TIMEOUT_US 450000      // switch to position recovery mode if no position update in 0.75s
#if (DEADRECKONING_TIMEOUT_HOLD_POSITION_US <= NN_DEADRECKONING_TIMEOUT_US)
#error "NN_DEADRECKONING_TIMEOUT_US must be lower than DEADRECKONING_TIMEOUT_HOLD_POSITION_US"
#endif

void nn_init(void);                             // initializes controller and sets a starting point

void nn_activate(void);                         // activates the controller
void nn_deactivate(void);                       // deactivates the controller
bool nn_is_active(void);                        // returns true if the controller is active

void nn_compute_motor_cmds(void);               // computes motor commands based on the world_state[16] (pos, vel, att, rate, motorspeeds)
float* nn_get_motor_cmds(void);                 // returns the computed motor commands

#endif // NN_CONTROL_H
