
#include "pi-messages.h"
#include "external_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/imu.h"

#ifdef USE_GPS_PI

#ifndef USE_TELEMETRY_PI
#error "USE_GPS_PI requires the use of USE_TELEMETRY_PI"
#endif

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;

vio_pos_ned_t vioPosNed;
ext_pos_state_t vioPosState = EXT_POS_NO_SIGNAL;
timeUs_t vioLatestMsgTime = 0;

pos_setpoint_ned_t posSpNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;


void checkNewPos(void) {
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

void getExternalPos(timeUs_t current) {

    UNUSED(current);

    checkNewPos();
    if (extPosState == EXT_POS_NO_SIGNAL)
        return;

    if (extPosState == EXT_POS_NEW_MESSAGE) {
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
    if (piMsgFakeGpsRx) {
        gpsSol.llh.lat = piMsgFakeGpsRx->lat;
        gpsSol.llh.lon = piMsgFakeGpsRx->lon;
        gpsSol.llh.altCm = piMsgFakeGpsRx->altCm;
        gpsSol.dop.hdop = piMsgFakeGpsRx->hdop;
        gpsSol.groundSpeed = piMsgFakeGpsRx->groundSpeed;
        gpsSol.groundCourse = piMsgFakeGpsRx->groundCourse;
        gpsSol.numSat = piMsgFakeGpsRx->numSat;
        // let the configurator know that we have a good GPS, and a good fix
        sensorsSet(SENSOR_GPS);
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_EVER);
    }

    if (extPosState == EXT_POS_NO_SIGNAL) {
        DISABLE_STATE(GPS_FIX);
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
            posSetpointState = EXT_POS_NEW_MESSAGE;
        } else {
            posSetpointState = EXT_POS_STILL_VALID;
            // no upper bound on still_valid for setpoints
        }
    }
}

#endif
