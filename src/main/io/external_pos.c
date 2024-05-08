
#include "pi-messages.h"
#include "external_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/imu.h"

#ifdef USE_POS_CTL

//#ifndef USE_TELEMETRY_PI
//#error "USE_GPS_PI requires the use of USE_TELEMETRY_PI"
//#endif
// UGLY HACK ----------------------------------------------
#include "flight/trajectory_tracker.h"
#include "fc/core.h"
// --------------------------------------------------------

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
pos_setpoint_ned_t posSpNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;
timeUs_t extLatestMsgGpsTime = 0;

#ifdef USE_GPS_PI
void checkNewPosPi(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_ms*1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, extLatestMsgTime);
        if (deltaMsgs != 0) {
        //if (deltaMsgs > 0) {
            // new message available
            // todo: how to catch stuck values? Like new messages comming in,
            // but the values are frozed, which is also basically signal loss
            extPosState = EXT_POS_NEW_MESSAGE;
            extLatestMsgTime = currentMsgTime;
        //} else if (deltaMsgs) < 0) {
            // I think this may happen if there is a 35 to 70min delay between
            // messages (or multiples of that). This will cause the difference 
            // calculations to roll-over (expected behaviour)
            // panic for one call, but update extLatestMsgTime
        //    extPosState = EXT_POS_NO_SIGMAL;
        //    extLatestMsgTime = currentMsgTime;
        //    return;
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

#ifdef USE_GPS_PI
    checkNewPosPi();
#else
    checkNewPosGps();
#endif

    if (extPosState == EXT_POS_NO_SIGNAL)
        return;

    if (extPosState == EXT_POS_NEW_MESSAGE) {
#ifdef USE_GPS_PI
        // process new message into NED
        extPosNed.pos.V.X = piMsgExternalPoseRx->enu_y;
        extPosNed.pos.V.Y = piMsgExternalPoseRx->enu_x;
        extPosNed.pos.V.Z = -piMsgExternalPoseRx->enu_z;
        extPosNed.vel.V.X = piMsgExternalPoseRx->enu_yd;
        extPosNed.vel.V.Y = piMsgExternalPoseRx->enu_xd;
        extPosNed.vel.V.Z = -piMsgExternalPoseRx->enu_zd;
        fp_euler_t eulers;
        fp_quaternion_t quat;
        // the quaternion x,y,z are in ENU, we convert them to NED
        quat.w = piMsgExternalPoseRx->body_qi;
        quat.x = piMsgExternalPoseRx->body_qy;
        quat.y = piMsgExternalPoseRx->body_qx;
        quat.z =-piMsgExternalPoseRx->body_qz;
        fp_quaternionProducts_t qP;
        quaternionProducts_of_quaternion(&qP, &quat);
        fp_euler_of_quaternionProducts (&eulers, &qP);
        extPosNed.att.angles.roll = eulers.angles.roll;
        extPosNed.att.angles.pitch = eulers.angles.pitch;
        extPosNed.att.angles.yaw = eulers.angles.yaw;
#else
#define REARTH 6371000.f
        t_fp_vector newPos;
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

#ifdef USE_GPS_PI
    if (piMsgPosSetpointRx) {
        timeUs_t currentSetpointTime = piMsgPosSetpointRx->time_ms * 1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentSetpointTime, latestSetpointTime);
        bool newMsg = (deltaMsgs != 0);
        if (newMsg) {
#ifdef USE_TRAJECTORY_TRACKER
            // UGLY HACK: velocity setpoint is used for communication with the trajectory tracker -----------------
            
            // call initTrajectoryTracker when piMsgPosSetpointRx->enu_xd == 1
            if (piMsgPosSetpointRx->enu_xd == 1) {
                initTrajectoryTracker();
                return;
            }

            // call stopTrajectoryTracker when piMsgPosSetpointRx->enu_xd == 2
            if (piMsgPosSetpointRx->enu_xd == 2) {
                stopTrajectoryTracker();
                return;
            }

            // disarm when piMsgPosSetpointRx->enu_xd == 3
            if (piMsgPosSetpointRx->enu_xd == 3) {
                disarm(DISARM_REASON_SWITCH);
                return;
            }

            // call setSpeedTrajectoryTracker when piMsgPosSetpointRx->enu_yd > 0 and use piMsgPosSetpointRx->enu_yd as speed
            if (piMsgPosSetpointRx->enu_yd > 0) {
                setSpeedTrajectoryTracker(piMsgPosSetpointRx->enu_yd);
                return;
            }
#endif
            
            // -----------------------------------------------------------------------------------------------
            latestSetpointTime = currentSetpointTime;
            posSpNed.pos.V.X = piMsgPosSetpointRx->enu_y;
            posSpNed.pos.V.Y = piMsgPosSetpointRx->enu_x;
            posSpNed.pos.V.Z = -piMsgPosSetpointRx->enu_z;
            
            posSpNed.vel.V.X = 0; //piMsgPosSetpointRx->enu_yd;
            posSpNed.vel.V.Y = 0; //piMsgPosSetpointRx->enu_xd;
            posSpNed.vel.V.Z = 0; //-piMsgPosSetpointRx->enu_zd;
            posSpNed.psi = DEGREES_TO_RADIANS(-piMsgPosSetpointRx->yaw);
            posSetpointState = EXT_POS_NEW_MESSAGE;
            
        } else {
            posSetpointState = EXT_POS_STILL_VALID;
            // no upper bound on still_valid for setpoints
        }
    }
#else
    if (posSpNed.time_ms > 0)
        posSetpointState = EXT_POS_STILL_VALID;

    timeUs_t currentSetpointTime = posSpNed.time_ms * 1e3;
    timeDelta_t deltaMsgs = cmpTimeUs(currentSetpointTime, latestSetpointTime);
    if (deltaMsgs != 0)
        posSetpointState = EXT_POS_NEW_MESSAGE;
#endif
}

#endif