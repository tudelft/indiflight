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

// indiflight: frame convention switch (the single place it is documented).
//
//   USE_FRD_NED (indiflight builds, defined by USE_INDIFLIGHT in
//     target/common_pre.h): the whole firmware uses the aerospace convention,
//     right handed, body frame FRD (X forward, Y right, Z down) and earth frame
//     NED. Gyro, acc and mag are converted right after board alignment
//     (sensors/boardalignment.c); attitude angles are roll right +, pitch nose
//     up +, yaw = heading; rates, setpoints, PID sums and mixer columns are FRD;
//     the position estimator / autopilot / navigation are NED.
//   otherwise: upstream Betaflight's convention (body FLU, earth NWU for the
//     attitude, ENU for navigation), the original upstream code.
//
// The unit test Makefile defines it with INDIFLIGHT=yes. Code
// that differs between the conventions is written as
//   #ifdef USE_FRD_NED  <FRD/NED code>  #else  <upstream code>  #endif

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

#ifdef USE_FRD_NED
// Earth-frame axis indices (frame, axis and positive direction in the name).
// Do NOT reintroduce bare .x/.y for earth-frame quantities.

// NED (North-East-Down): the body (FRD) -> earth frame produced by rMat in imu.c,
// and the frame of the position estimator, autopilot and navigation (cm / m).
// matrixVectorMul(&v, &rMat, &bodyVec) yields a vector indexed by these.
typedef enum {
    NED_N = 0,  // +North
    NED_E = 1,  // +East
    NED_D = 2   // +Down
} axisNED_e;

// Horizontal earth-frame (North-East) index for the 2-axis position/velocity
// PID arrays. Matches the first two NED indices (see EF_AXIS_COUNT).
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

#else // upstream frame
// Earth-frame axis indices.
//
// These names encode three things at every call site: the frame, the axis,
// and the positive direction. Indexing a vector with e.g. NWU_W makes it
// obvious that the value is West-positive, so the sign of any North/East/Up
// conversion is visible in the code rather than carried in a reader's head.
// Do NOT reintroduce bare .x/.y for earth-frame quantities - the inability to
// tell which axis (and which sign) .x meant was the root cause of the
// getLinearAccelENU East-West inversion.

// NWU (North-West-Up): the body->earth frame produced by rMat in imu.c.
// matrixVectorMul(&v, &rMat, &bodyVec) yields a vector indexed by these.
typedef enum {
    NWU_N = 0,  // +North
    NWU_W = 1,  // +West
    NWU_U = 2   // +Up
} axisNWU_e;

// ENU (East-North-Up): the navigation frame used by the position estimator
// and autopilot. The position/velocity vector3_t fields are indexed by these.
typedef enum {
    ENU_E = 0,  // +East
    ENU_N = 1,  // +North
    ENU_U = 2   // +Up
} axisENU_e;

// Horizontal earth-frame (East-North) index for the 2-axis position/velocity
// PID arrays. Matches the first two ENU indices (see EF_AXIS_COUNT).
typedef enum {
    EF_EAST = 0,   // +East
    EF_NORTH = 1   // +North
} efAxis_e;

// The autopilot stores into EF_AXIS_COUNT arrays by EF_EAST/EF_NORTH while
// reading the estimate by ENU_E/ENU_N; that is only correct because the two
// horizontal frames share index values. Lock the invariant so a future
// reordering of either enum fails to compile rather than silently swapping axes.
STATIC_ASSERT((int)EF_EAST == (int)ENU_E, ef_east_must_match_enu_east);
STATIC_ASSERT((int)EF_NORTH == (int)ENU_N, ef_north_must_match_enu_north);

#endif // USE_FRD_NED

#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)
