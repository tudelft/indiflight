
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

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
pos_setpoint_ned_t posSetpointNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;

//local
timeUs_t latestMsgTime = 0;

void checkNewPos(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_ms*1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, latestMsgTime);
        if (deltaMsgs != 0) {
        //if (deltaMsgs > 0) {
            // new message available
            // todo: how to catch stuck values? Like new messages comming in,
            // but the values are frozed, which is also basically signal loss
            extPosState = EXT_POS_NEW_MESSAGE;
            latestMsgTime = currentMsgTime;
        //} else if (deltaMsgs) < 0) {
            // I think this may happen if there is a 35 to 70min delay between
            // messages (or multiples of that). This will cause the difference 
            // calculations to roll-over (expected behaviour)
            // panic for one call, but update latestMsgTime
        //    extPosState = EXT_POS_NO_SIGMAL;
        //    latestMsgTime = currentMsgTime;
        //    return;
        } else {
            // assume still valid for now
            extPosState = EXT_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), latestMsgTime);
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
        // process new message into NED
        extPosNed.pos.V.X = piMsgExternalPoseRx->enu_y;
        extPosNed.pos.V.Y = piMsgExternalPoseRx->enu_x;
        extPosNed.pos.V.Z = -piMsgExternalPoseRx->enu_z;
        extPosNed.vel.V.X = piMsgExternalPoseRx->enu_yd;
        extPosNed.vel.V.Y = piMsgExternalPoseRx->enu_xd;
        extPosNed.vel.V.Z = -piMsgExternalPoseRx->enu_zd;
        fp_angles_t eulers;
        fp_quaternion_t quat;
        // the quaternion x,y,z are in ENU, we convert them to NED
        quat.qi = piMsgExternalPoseRx->body_qi;
        quat.qx = piMsgExternalPoseRx->body_qy;
        quat.qy = piMsgExternalPoseRx->body_qx;
        quat.qz =-piMsgExternalPoseRx->body_qz;
        float_eulers_of_quat(&eulers, &quat);
        extPosNed.att.angles.roll = eulers.angles.roll;
        extPosNed.att.angles.pitch = eulers.angles.pitch;
        extPosNed.att.angles.yaw = eulers.angles.yaw;
    }

    // extrapolate position with speed info
    /* removed, because now we include this info the observer in imu.h
    static timeUs_t latestExtrapolationTime = 0;
    timeUs_t currentExtrapolationTime = micros();
    timeDelta_t delta = cmpTimeUs(currentExtrapolationTime, latestExtrapolationTime);
    if ((latestExtrapolationTime > 0) && (delta > 0)) {
        extPosNed.pos.V.X += ((float) delta) * 1e-6f * extPosNed.vel.V.X;
        extPosNed.pos.V.Y += ((float) delta) * 1e-6f * extPosNed.vel.V.Y;
        extPosNed.pos.V.Z += ((float) delta) * 1e-6f * extPosNed.vel.V.Z;
    }
    latestExtrapolationTime = currentExtrapolationTime;
    */
        // todo: DONE in imu.c extrapolate psi from body rates somehow using the quat
}

void getFakeGps(timeUs_t current) {
    // not that critical, because this is only for display purposes
    // just dump without checking anything
    UNUSED(current);

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
}

void getPosSetpoint(timeUs_t current) {
    UNUSED(current);

    static timeUs_t latestSetpointTime = 0;

    if (piMsgPosSetpointRx) {
        timeUs_t currentSetpointTime = piMsgPosSetpointRx->time_ms * 1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentSetpointTime, latestSetpointTime);
        bool newMsg = (deltaMsgs != 0);
        if (newMsg) {
            latestSetpointTime = currentSetpointTime;
            posSetpointNed.pos.V.X = piMsgPosSetpointRx->enu_y;
            posSetpointNed.pos.V.Y = piMsgPosSetpointRx->enu_x;
            posSetpointNed.pos.V.Z = -piMsgPosSetpointRx->enu_z;
            posSetpointNed.vel.V.X = piMsgPosSetpointRx->enu_yd;
            posSetpointNed.vel.V.Y = piMsgPosSetpointRx->enu_xd;
            posSetpointNed.vel.V.Z = -piMsgPosSetpointRx->enu_zd;
            posSetpointNed.psi = DEGREES_TO_RADIANS(-piMsgPosSetpointRx->yaw);
            posSetpointState = EXT_POS_NEW_MESSAGE;
        } else {
            posSetpointState = EXT_POS_STILL_VALID;
            // no upper bound on still_valid for setpoints
        }
    }
}

#endif