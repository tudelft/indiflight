/*
 * Get local NED position from difference sources (uplink/GPS)
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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

#ifndef LOCAL_POS_H
#define LOCAL_POS_H

#include "drivers/time.h"
#include "common/maths.h"

typedef enum {
    LOCAL_POS_SOURCE_PI,
    LOCAL_POS_SOURCE_UROS,
    LOCAL_POS_SOURCE_GPS,
    LOCAL_POS_SOURCE_MOCKUP,
} local_pos_source_e;

// todo: reformulate using fp_vector_t
typedef struct __local_pos_ned_t {
    uint32_t time_us;
    local_pos_source_e source;
    bool new;
    fp_vector_t pos;
    fp_vector_t vel;
    fp_quaternion_t quat;
} local_pos_ned_t;

typedef struct __local_pos_sp_ned_t {
    uint32_t time_us;
    local_pos_source_e source;
    bool new;
    fp_vector_t pos;
    fp_vector_t vel;
    float psi;
    bool trackPsi;
} local_pos_sp_ned_t;

extern local_pos_ned_t posMeasNed;
extern local_pos_sp_ned_t posSpNed;

void setLocalPosMeas(local_pos_ned_t* pos);
void setLocalPosSp(local_pos_sp_ned_t* sp);


#endif // LOCAL_POS_H