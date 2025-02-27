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


#include "pi-messages.h"
#include "local_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "io/local_pos.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/ahrs.h"
#include "io/gps.h"

#ifdef USE_LOCAL_POSITION

#ifndef USE_ACC
#error "USE_EKF requires USE_ACC"
#endif

#ifndef USE_GYRO
#error "USE_EKF requires USE_GYRO"
#endif

#ifndef USE_EKF
#error "USE_LOCAL_POSITION must be used with USE_EKF"
#endif

//extern
local_pos_ned_t posMeasNed;
local_pos_state_t posMeasState = LOCAL_POS_NO_SIGNAL;
timeUs_t posLatestMsgTime = 0;

local_pos_sp_ned_t posSpNed;
local_pos_state_t posSpState = LOCAL_POS_NO_SIGNAL;


void checkNewPos(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, posLatestMsgTime);
        if (deltaMsgs != 0) {
            // new message available
            posMeasState = LOCAL_POS_NEW_MESSAGE;
            posLatestMsgTime = currentMsgTime;
        } else {
            // assume still valid for now
            posMeasState = LOCAL_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), posLatestMsgTime);
        if (delta > LOCAL_POS_TIMEOUT_US) {
            // signal lost
            posMeasState = LOCAL_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        posMeasState = LOCAL_POS_NO_SIGNAL;
    }
}

void getLocalPos(timeUs_t current) {
    UNUSED(current);

#ifdef USE_GPS
    if (realGPSConfigured) {
        return;
    }
#endif

    checkNewPos();

    switch (posMeasState) {
        case LOCAL_POS_NEW_MESSAGE:
            // time stamp
            posMeasNed.time_us = piMsgExternalPoseRx->time_us;
            // process new message (should be NED)
            posMeasNed.pos.V.X = piMsgExternalPoseRx->ned_x;
            posMeasNed.pos.V.Y = piMsgExternalPoseRx->ned_y;
            posMeasNed.pos.V.Z = piMsgExternalPoseRx->ned_z;
            posMeasNed.vel.V.X = piMsgExternalPoseRx->ned_xd;
            posMeasNed.vel.V.Y = piMsgExternalPoseRx->ned_yd;
            posMeasNed.vel.V.Z = piMsgExternalPoseRx->ned_zd;
            // the quaternion x,y,z should be NED
            posMeasNed.quat.w = piMsgExternalPoseRx->body_qi;
            posMeasNed.quat.x = piMsgExternalPoseRx->body_qx;
            posMeasNed.quat.y = piMsgExternalPoseRx->body_qy;
            posMeasNed.quat.z = piMsgExternalPoseRx->body_qz;

            sensorsSet(SENSOR_GPS);
            ENABLE_STATE(GPS_FIX);
            ENABLE_STATE(GPS_FIX_EVER);
            break;
        case LOCAL_POS_NO_SIGNAL:
            DISABLE_STATE(GPS_FIX);
            break;
        default:
            break;
    }
}

void getFakeGps(timeUs_t current) {
    // not that critical, because this is only for display purposes
    // just dump without checking anything
    UNUSED(current);

#ifdef USE_GPS
    if (!realGPSConfigured && piMsgFakeGpsRx) {
        gpsSol.llh.lat = piMsgFakeGpsRx->lat;
        gpsSol.llh.lon = piMsgFakeGpsRx->lon;
        gpsSol.llh.altCm = piMsgFakeGpsRx->altCm;
        gpsSol.dop.hdop = piMsgFakeGpsRx->hdop;
        gpsSol.groundSpeed = piMsgFakeGpsRx->groundSpeed;
        gpsSol.groundCourse = piMsgFakeGpsRx->groundCourse;
        gpsSol.numSat = piMsgFakeGpsRx->numSat;
    }
#endif
}

void getPosSetpoint(timeUs_t current) {
    UNUSED(current);

    static timeUs_t latestSetpointTime = 0;

    if (piMsgPosSetpointRx) {
        timeUs_t currentSetpointTime = piMsgPosSetpointRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentSetpointTime, latestSetpointTime);
        bool newMsg = (deltaMsgs != 0);
        if (newMsg) {
            latestSetpointTime = currentSetpointTime;
            posSpNed.pos.V.X = piMsgPosSetpointRx->ned_x;
            posSpNed.pos.V.Y = piMsgPosSetpointRx->ned_y;
            posSpNed.pos.V.Z = piMsgPosSetpointRx->ned_z;
            posSpNed.vel.V.X = piMsgPosSetpointRx->ned_xd;
            posSpNed.vel.V.Y = piMsgPosSetpointRx->ned_yd;
            posSpNed.vel.V.Z = piMsgPosSetpointRx->ned_zd;
            posSpNed.psi = DEGREES_TO_RADIANS(piMsgPosSetpointRx->yaw);
            posSpNed.trackPsi = true;
            posSpState = LOCAL_POS_NEW_MESSAGE;
        } else if (posSpState != LOCAL_POS_NO_SIGNAL) {
            // if not no-signalled by other means, just keep this
            posSpState = LOCAL_POS_STILL_VALID;
        }
    }
}

#endif
