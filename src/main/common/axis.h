/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/utils.h"

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define XYZ_AXIS_COUNT 3

// See http://en.wikipedia.org/wiki/Flight_dynamics
typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum {
    AI_ROLL = 0,
    AI_PITCH
} angle_index_t;

#define RP_AXIS_COUNT 2
#define EF_AXIS_COUNT 2

// Earth-frame axis indices.
//
// These names encode three things at every call site: the frame, the axis,
// and the positive direction. Indexing a vector with e.g. NED_E makes it
// obvious that the value is East-positive, so the sign of any conversion is
// visible in the code rather than carried in a reader's head.
// Do NOT reintroduce bare .x/.y for earth-frame quantities.
//
// indiflight: the whole firmware uses the aerospace convention, body frame
// FRD (X forward, Y right, Z down; sensors are converted right after board
// alignment) and earth frame NED.

// NED (North-East-Down): the body->earth frame produced by rMat in imu.c.
// matrixVectorMul(&v, &rMat, &bodyVec) yields a vector indexed by these.
typedef enum {
    NED_N = 0,  // +North
    NED_E = 1,  // +East
    NED_D = 2   // +Down
} axisNED_e;

// Horizontal earth-frame (North-East) index for the 2-axis position/velocity
// PID arrays. Matches the first two NED indices (see EF_AXIS_COUNT). The
// position estimator, autopilot and navigation use NED (cm) throughout.
typedef enum {
    EF_NORTH = 0,  // +North
    EF_EAST = 1    // +East
} efAxis_e;

// The autopilot stores into EF_AXIS_COUNT arrays by EF_NORTH/EF_EAST while
// reading the estimate by NED_N/NED_E (and casts the 3D NED estimate to its
// 2D horizontal part); that is only correct because the two frames share
// index values. Lock the invariant so a reordering fails to compile.
STATIC_ASSERT((int)EF_NORTH == (int)NED_N, ef_north_must_match_ned_north);
STATIC_ASSERT((int)EF_EAST == (int)NED_E, ef_east_must_match_ned_east);

#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)
