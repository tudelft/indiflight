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

#ifdef USE_GPS_PI

typedef enum {
    EXT_POS_NO_SIGNAL,
    EXT_POS_STILL_VALID,
    EXT_POS_NEW_MESSAGE,
} ext_pos_state_t;

// todo: reformulate using fp_vector_t
typedef struct __ext_pos_ned_t {
    uint32_t time_us;
    fp_vector_t pos;
    fp_vector_t vel;
    fp_euler_t att;
} ext_pos_ned_t;

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

typedef struct __pos_setpoint_ned_t {
    fp_vector_t pos;
    fp_vector_t vel;
    float psi;
    bool trackPsi;
} pos_setpoint_ned_t;

// structs used for EXTERNAL_POSE message
extern ext_pos_ned_t extPosNed;
extern ext_pos_state_t extPosState;
extern timeUs_t extLatestMsgTime;

// structs used for VIO_POSE message
#ifdef USE_VIO_POSE
extern vio_pos_ned_t vioPosNed;
extern ext_pos_state_t vioPosState;
extern timeUs_t vioLatestMsgTime;
#endif

// structs used for POS_SETPOINT message
extern pos_setpoint_ned_t posSpNed;
extern ext_pos_state_t posSetpointState;

#define EXT_POS_FREQ 50
#define EXT_POS_TIMEOUT_US 300000

#ifdef USE_VIO_POSE
#define VIO_POS_TIMEOUT_US 30000000
#endif

#define POS_SETPOINT_OUTDATED_US 1000000

void checkNewPos(void);
void getExternalPos(timeUs_t current);
#ifdef USE_VIO_POSE
void checkNewVioPos(void);
void getVioPos(timeUs_t current);
#endif
void getFakeGps(timeUs_t current);
void getPosSetpoint(timeUs_t current);
#endif