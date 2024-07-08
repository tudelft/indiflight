#ifndef NN_CONTROL_H
#define NN_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pg/pg.h"

typedef struct nnConfig_s {
    uint8_t rate_denom;     // run net every rate_denom times the inner loop is run
} nnConfig_t;

PG_DECLARE(nnConfig_t, nnConfig);

void nn_init(void);                             // initializes controller and sets a starting point

void nn_activate(void);                         // activates the controller
void nn_deactivate(void);                       // deactivates the controller
bool nn_is_active(void);                        // returns true if the controller is active

void nn_compute_motor_cmds(void);               // computes motor commands based on the world_state[16] (pos, vel, att, rate, motorspeeds)
float* nn_get_motor_cmds(void);                 // returns the computed motor commands

#endif // NN_CONTROL_H
