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

#ifdef USE_POS_CTL

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;

vio_pos_ned_t vioPosNed;
ext_pos_state_t vioPosState = EXT_POS_NO_SIGNAL;
timeUs_t vioLatestMsgTime = 0;

pos_setpoint_ned_t posSpNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;
timeUs_t extLatestMsgGpsTime = 0;

#ifdef USE_GPS_PI
void checkNewPosPi(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, extLatestMsgTime);
        if (deltaMsgs != 0) {
            // new message available
            extPosState = EXT_POS_NEW_MESSAGE;
            extLatestMsgTime = currentMsgTime;
        } else {
            // assume still valid for now
            extPosState = EXT_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), extLatestMsgTime);
        if (delta > EXT_POS_TIMEOUT_US) {
            // signal lost
            extPosState = EXT_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        extPosState = EXT_POS_NO_SIGNAL;
    }
}
#endif

void checkNewPosGps(void) {
    if (!STATE(GPS_FIX_EVER)) {
        extPosState = EXT_POS_NO_SIGNAL;
    } else {
        // assume still valid for now
        extPosState = EXT_POS_STILL_VALID;

        timeDelta_t deltaMsgs = cmpTimeUs(gpsData.lastMessage, extLatestMsgGpsTime);
        if (STATE(GPS_FIX) && (deltaMsgs != 0)) {
            extPosState = EXT_POS_NEW_MESSAGE;
            extLatestMsgTime = micros();
            extLatestMsgGpsTime = gpsData.lastMessage;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), extLatestMsgTime);
        if (delta > EXT_POS_TIMEOUT_US) {
            // signal lost
            extPosState = EXT_POS_NO_SIGNAL;
        }
    }
}

void getExternalPos(timeUs_t current) {
    UNUSED(current);

#ifdef USE_GPS
    if (realGPSConfigured) {
        return;
    }
#endif

#ifdef USE_GPS_PI
    checkNewPosPi();
#else
    checkNewPosGps();
#endif

    switch (extPosState) {
        case EXT_POS_NEW_MESSAGE:
#ifdef USE_GPS_PI
            // time stamp
            extPosNed.time_us = piMsgExternalPoseRx->time_us;
            // process new message (should be NED)
            extPosNed.pos.V.X = piMsgExternalPoseRx->ned_x;
            extPosNed.pos.V.Y = piMsgExternalPoseRx->ned_y;
            extPosNed.pos.V.Z = piMsgExternalPoseRx->ned_z;
            extPosNed.vel.V.X = piMsgExternalPoseRx->ned_xd;
            extPosNed.vel.V.Y = piMsgExternalPoseRx->ned_yd;
            extPosNed.vel.V.Z = piMsgExternalPoseRx->ned_zd;
            fp_euler_t eulers;
            fp_quaternion_t quat;
            // the quaternion x,y,z should be NED
            quat.w = piMsgExternalPoseRx->body_qi;
            quat.x = piMsgExternalPoseRx->body_qx;
            quat.y = piMsgExternalPoseRx->body_qy;
            quat.z = piMsgExternalPoseRx->body_qz;
            fp_quaternionProducts_t qP;
            quaternionProducts_of_quaternion(&qP, &quat);
            fp_euler_of_quaternionProducts (&eulers, &qP);
            extPosNed.att.angles.roll = eulers.angles.roll;
            extPosNed.att.angles.pitch = eulers.angles.pitch;
            extPosNed.att.angles.yaw = eulers.angles.yaw;
#else
#define REARTH 6371000.f
            fp_vector_t newPos;
            newPos.V.X = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lat - GPS_home[GPS_LATITUDE]) * REARTH;
            newPos.V.Y = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lon - GPS_home[GPS_LONGITUDE]) * REARTH
                * cosf(1e-7f * DEGREES_TO_RADIANS(GPS_home[GPS_LONGITUDE]));
            newPos.V.Z = -1e-2f * gpsSol.llh.altCm;
            timeDelta_t delta_ms = cmpTimeUs(gpsData.lastMessage, gpsData.lastLastMessage); // ms, not us
            if ((delta_ms > 0) && (delta_ms < 1000)) {
                float iDeltaS = 1. / (0.001f*delta_ms);
                extPosNed.vel = newPos;
                VEC3_SCALAR_MULT_ADD(extPosNed.vel, -1., extPosNed.pos);
                VEC3_SCALAR_MULT(extPosNed.vel, iDeltaS);
            }
            extPosNed.pos = newPos;
            extPosNed.att.angles.roll = 0;
            extPosNed.att.angles.pitch = 0;
            extPosNed.att.angles.yaw = DECIDEGREES_TO_RADIANS(gpsSol.trueYaw);
#endif
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

#ifdef USE_GPS_PI
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
#endif
}

#endif
