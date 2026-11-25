/*
 * Position control: mappings from the inertial acceleration setpoint
 * (accSpNedFromPos) to attitude (attSpNedFromPos), body specific force
 * (spfSpBodyFromPos) and, for some, body rate (rateSpBodyFromPos) setpoints.
 * Selected with position_attitude_mapping.
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
 * Copyright 2024 Robin Ferede (Delft University of Technology)
 *     Improved dynamic inversion code that maps desired accel to attitude.
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


#pragma once

#include "common/time.h"

// pos_att_ndi.c: multirotors
void posGetAttSpNedAndSpfSpBody(timeUs_t current);

// pos_att_indi.c: hybrids, incremental
void posAttInit_INDI(void);
void posGetAttSpNedAndSpfSpBody_INDI(timeUs_t current);

// pos_att_df.c: tailsitters, Tal (2022)
void posAttInit_DF(void);
void posGetAttSpNedAndSpfSpBody_DF(timeUs_t current);
