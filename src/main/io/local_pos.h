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


#include "drivers/time.h"
#include "common/maths.h"

typedef enum {
    LOCAL_POS_NO_SIGNAL,
    LOCAL_POS_STILL_VALID,
    LOCAL_POS_NEW_MESSAGE,
} local_pos_measurement_state_t;

typedef struct __local_pos_ned_t {
    uint32_t time_us;
    fp_vector_t pos;
    fp_vector_t vel;
    fp_quaternion_t quat;
} local_pos_ned_t;

typedef struct __vio_pos_ned_t {
    uint32_t time_us;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float p;
    float q;
    float r;
    float qw;
    float qx;
    float qy;
    float qz;
} vio_pos_ned_t;

typedef struct __local_pos_setpoint_ned_t {
    fp_vector_t pos;
    fp_vector_t vel;
    float psi;
    bool trackPsi;
} local_pos_setpoint_ned_t;

extern local_pos_ned_t posMeasNed;
extern local_pos_measurement_state_t posMeasState;
extern local_pos_setpoint_ned_t posSpNed;
extern local_pos_measurement_state_t posSpState;
extern timeUs_t posLatestMsgTime;

// structs used for VIO_POSE message
extern vio_pos_ned_t vioPosNed;
extern local_pos_measurement_state_t vioPosState;
extern timeUs_t vioLatestMsgTime;

#define POS_MEAS_TIMEOUT_US 300000
#define VIO_POS_TIMEOUT_US 30000000

void getLocalPos(timeUs_t current);
void checkNewVioPos(void);
void getVioPos(timeUs_t current);
void getFakeGps(timeUs_t current);
void getPosSetpoint(timeUs_t current);
