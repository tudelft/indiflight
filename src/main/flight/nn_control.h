/*
 *
 *
 * Copyright 2024 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Added parameter group and compile time macros
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

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

#define NN_DEADRECKONING_TIMEOUT_US 1000000      // switch to position recovery mode if no position update in 0.75s
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
