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
#include "external_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/imu.h"
#include "io/gps.h"

#ifdef USE_GPS_PI

#ifndef USE_TELEMETRY_PI
#error "USE_GPS_PI requires the use of USE_TELEMETRY_PI"
#endif

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;
timeUs_t extLatestMsgTimeReceived = 0;

vio_pos_ned_t vioPosNed;
ext_pos_state_t vioPosState = EXT_POS_NO_SIGNAL;
timeUs_t vioLatestMsgTime = 0;

ext_pos_ned_t offboardPosNed;

pos_setpoint_ned_t posSpNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;

// maximal allowed timing jitter
#define MAX_TIMING_JITTER_US 15000


void checkNewPos(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // Get delta Msg sent
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, extLatestMsgTime);
        // Get delta Msg received
        timeUs_t currentMsgTimeReceived = micros();
        timeDelta_t deltaMsgsReceived = cmpTimeUs(currentMsgTimeReceived, extLatestMsgTimeReceived);
        // Get timing jitter between mocap sample deltas and receive time deltas
        timeDelta_t timingJitter = deltaMsgsReceived - deltaMsgs;

        if ((timingJitter < MAX_TIMING_JITTER_US) && (timingJitter > -MAX_TIMING_JITTER_US)) {
            // new message available
            extPosState = EXT_POS_NEW_MESSAGE;
            extLatestMsgTime = currentMsgTime;
            extLatestMsgTimeReceived = currentMsgTimeReceived;
        } else {
            // assume still valid for now
            extPosState = EXT_POS_STILL_VALID;
        }

        // if (deltaMsgs != 0) {
        //     // new message available
        //     extPosState = EXT_POS_NEW_MESSAGE;
        //     extLatestMsgTime = currentMsgTime;
        //     extLatestMsgTimeReceived = micros();
        // } else {
        //     // assume still valid for now
        //     extPosState = EXT_POS_STILL_VALID;
        // }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), currentMsgTimeReceived);
        if (delta > EXT_POS_TIMEOUT_US) {
            // signal lost
            extPosState = EXT_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        extPosState = EXT_POS_NO_SIGNAL;
    }
}

void getExternalPos(timeUs_t current) {
    UNUSED(current);

#ifdef USE_GPS
    if (realGPSConfigured) {
        return;
    }
#endif

    checkNewPos();

    switch (extPosState) {
        case EXT_POS_NEW_MESSAGE:
            // time stamp
            extPosNed.time_us = piMsgExternalPoseRx->time_us;
            // process new message (should be NED)
            extPosNed.pos.V.X = piMsgExternalPoseRx->ned_x;
            extPosNed.pos.V.Y = piMsgExternalPoseRx->ned_y;
            extPosNed.pos.V.Z = piMsgExternalPoseRx->ned_z;
            extPosNed.vel.V.X = piMsgExternalPoseRx->ned_xd;
            extPosNed.vel.V.Y = piMsgExternalPoseRx->ned_yd;
            extPosNed.vel.V.Z = piMsgExternalPoseRx->ned_zd;
            // the quaternion x,y,z should be NED
            extPosNed.quat.w = piMsgExternalPoseRx->body_qi;
            extPosNed.quat.x = piMsgExternalPoseRx->body_qx;
            extPosNed.quat.y = piMsgExternalPoseRx->body_qy;
            extPosNed.quat.z = piMsgExternalPoseRx->body_qz;

            sensorsSet(SENSOR_GPS);
            ENABLE_STATE(GPS_FIX);
            ENABLE_STATE(GPS_FIX_EVER);
            break;
        case EXT_POS_NO_SIGNAL:
            DISABLE_STATE(GPS_FIX);
            break;
        default:
            break;
    }
}

#ifdef USE_VIO_POSE
void checkNewVioPos(void) {
    if (piMsgVioPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgVioPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, vioLatestMsgTime);
        if (deltaMsgs != 0) {
            // new message available
            vioPosState = EXT_POS_NEW_MESSAGE;
            vioLatestMsgTime = currentMsgTime;
        } else {
            // assume still valid for now
            vioPosState = EXT_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), vioLatestMsgTime);
        if (delta > VIO_POS_TIMEOUT_US) {
            // signal lost
            vioPosState = EXT_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        vioPosState = EXT_POS_NO_SIGNAL;
    }
}

void getVioPos(timeUs_t current) {
    UNUSED(current);

    checkNewVioPos();
    if (vioPosState == EXT_POS_NO_SIGNAL)
        return;

    if (vioPosState == EXT_POS_NEW_MESSAGE) {
        // time stamp
        vioPosNed.time_us = piMsgVioPoseRx->time_us;
        // process new message from UNKNOWN REFERENCE FRAME to NED
        vioPosNed.x = piMsgVioPoseRx->x;
        vioPosNed.y = piMsgVioPoseRx->y;
        vioPosNed.z = piMsgVioPoseRx->z;
        vioPosNed.vx = piMsgVioPoseRx->vx;
        vioPosNed.vy = piMsgVioPoseRx->vy;
        vioPosNed.vz = piMsgVioPoseRx->vz;
        vioPosNed.p = piMsgVioPoseRx->p;
        vioPosNed.q = piMsgVioPoseRx->q;
        vioPosNed.r = piMsgVioPoseRx->r;
        vioPosNed.qw = piMsgVioPoseRx->qw;
        vioPosNed.qx = piMsgVioPoseRx->qx;
        vioPosNed.qy = piMsgVioPoseRx->qy;
        vioPosNed.qz = piMsgVioPoseRx->qz;
    }
}
#endif

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
            posSetpointState = EXT_POS_NEW_MESSAGE;
        } else if (posSetpointState != EXT_POS_NO_SIGNAL) {
            // if not no-signalled by other means, just keep this
            posSetpointState = EXT_POS_STILL_VALID;
        }
    }
}

#endif
