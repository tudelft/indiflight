
#include "pi-messages.h"
#include "local_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/imu.h"
#include "io/serial.h"

#ifdef USE_LOCAL_POSITION

#if !defined(USE_GPS) && !defined(USE_LOCAL_POSITION_PI)
#error "USE_LOCAL_POSITION requires either USE_GPS or USE_LOCAL_POSITION_PI"
#endif

// UGLY HACK ----------------------------------------------
#include "flight/trajectory_tracker.h"
#include "fc/core.h"
// --------------------------------------------------------

//extern
local_pos_ned_t posMeasNed;
local_pos_measurement_state_t posMeasState = LOCAL_POS_NO_SIGNAL;
local_pos_setpoint_ned_t posSpNed;
local_pos_measurement_state_t posSpState = LOCAL_POS_NO_SIGNAL;
timeUs_t posLatestMsgTime = 0;
timeUs_t latestMsgGpsTime = 0;

#ifdef USE_LOCAL_POSITION_PI
static void checkNewPosPi(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_ms*1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, posLatestMsgTime);
        if (deltaMsgs != 0) {
        //if (deltaMsgs > 0) {
            // new message available
            // todo: how to catch stuck values? Like new messages comming in,
            // but the values are frozed, which is also basically signal loss
            posMeasState = LOCAL_POS_NEW_MESSAGE;
            posLatestMsgTime = currentMsgTime;
        //} else if (deltaMsgs) < 0) {
            // I think this may happen if there is a 35 to 70min delay between
            // messages (or multiples of that). This will cause the difference 
            // calculations to roll-over (expected behaviour)
            // panic for one call, but update posLatestMsgTime
        //    posMeasState = LOCAL_POS_NO_SIGMAL;
        //    posLatestMsgTime = currentMsgTime;
        //    return;
        } else {
            // assume still valid for now
            posMeasState = LOCAL_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), posLatestMsgTime);
        if (delta > POS_MEAS_TIMEOUT_US) {
            // signal lost
            posMeasState = LOCAL_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        posMeasState = LOCAL_POS_NO_SIGNAL;
    }
}
#endif

#ifdef USE_GPS
static void checkNewPosGps(void) {
    if (!STATE(GPS_FIX_EVER)) {
        posMeasState = LOCAL_POS_NO_SIGNAL;
    } else {
        // assume still valid for now
        posMeasState = LOCAL_POS_STILL_VALID;

        timeDelta_t deltaMsgs = cmpTimeUs(gpsData.lastMessage, latestMsgGpsTime);
        if (STATE(GPS_FIX) && (deltaMsgs != 0)) {
            posMeasState = LOCAL_POS_NEW_MESSAGE;
            posLatestMsgTime = micros();
            latestMsgGpsTime = gpsData.lastMessage;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), posLatestMsgTime);
        if (delta > POS_MEAS_TIMEOUT_US) {
            // signal lost
            posMeasState = LOCAL_POS_NO_SIGNAL;
        }
    }
}
#endif

void getLocalPos(timeUs_t current) {

    UNUSED(current);

#ifdef USE_LOCAL_POSITION_PI
    static bool piTelemConfigured;
    piTelemConfigured = (findSerialPortConfig(FUNCTION_TELEMETRY_PI) != NULL);
    if (piTelemConfigured) {
        checkNewPosPi();
        if (posMeasState == LOCAL_POS_NO_SIGNAL) return;
        if (posMeasState == LOCAL_POS_NEW_MESSAGE) {
            // process new message into NED
            posMeasNed.pos.V.X = piMsgExternalPoseRx->enu_y;
            posMeasNed.pos.V.Y = piMsgExternalPoseRx->enu_x;
            posMeasNed.pos.V.Z = -piMsgExternalPoseRx->enu_z;
            posMeasNed.vel.V.X = piMsgExternalPoseRx->enu_yd;
            posMeasNed.vel.V.Y = piMsgExternalPoseRx->enu_xd;
            posMeasNed.vel.V.Z = -piMsgExternalPoseRx->enu_zd;
            fp_euler_t eulers;
            fp_quaternion_t quat;
            // the quaternion x,y,z are in ENU, we convert them to NED
            quat.w = piMsgExternalPoseRx->body_qi;
            quat.x = piMsgExternalPoseRx->body_qy;
            quat.y = piMsgExternalPoseRx->body_qx;
            quat.z =-piMsgExternalPoseRx->body_qz;
            fp_quaternionProducts_t qP;
            quaternionProducts_of_quaternion(&qP, &quat);
            fp_euler_of_quaternionProducts(&eulers, &qP);
            posMeasNed.att.angles.roll = eulers.angles.roll;
            posMeasNed.att.angles.pitch = eulers.angles.pitch;
            posMeasNed.att.angles.yaw = eulers.angles.yaw;
        }
    } else
#endif
    {
#ifdef USE_GPS
#define REARTH 6371000.f
        checkNewPosGps();
        if (posMeasState == LOCAL_POS_NO_SIGNAL) return;
        if (posMeasState == LOCAL_POS_NEW_MESSAGE) {
            fp_vector_t newPos;
            newPos.V.X = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lat - GPS_home[GPS_LATITUDE]) * REARTH;
            newPos.V.Y = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lon - GPS_home[GPS_LONGITUDE]) * REARTH
                * cosf(1e-7f * DEGREES_TO_RADIANS(GPS_home[GPS_LONGITUDE]));
            newPos.V.Z = -1e-2f * gpsSol.llh.altCm;
            timeDelta_t delta_ms = cmpTimeUs(gpsData.lastMessage, gpsData.lastLastMessage); // ms, not us
            if ((delta_ms > 0) && (delta_ms < 1000)) {
                float iDeltaS = 1. / (0.001f*delta_ms);
                posMeasNed.vel = newPos;
                VEC3_SCALAR_MULT_ADD(posMeasNed.vel, -1., posMeasNed.pos);
                VEC3_SCALAR_MULT(posMeasNed.vel, iDeltaS);
            }
            posMeasNed.pos = newPos;
            posMeasNed.att.angles.roll = 0;
            posMeasNed.att.angles.pitch = 0;
            posMeasNed.att.angles.yaw = DECIDEGREES_TO_RADIANS(gpsSol.trueYaw);
        }
#endif
    }

    // extrapolate position with speed info
    /* removed, because now we include this info the observer in imu.h
    static timeUs_t latestExtrapolationTime = 0;
    timeUs_t currentExtrapolationTime = micros();
    timeDelta_t delta = cmpTimeUs(currentExtrapolationTime, latestExtrapolationTime);
    if ((latestExtrapolationTime > 0) && (delta > 0)) {
        posMeasNed.pos.V.X += ((float) delta) * 1e-6f * posMeasNed.vel.V.X;
        posMeasNed.pos.V.Y += ((float) delta) * 1e-6f * posMeasNed.vel.V.Y;
        posMeasNed.pos.V.Z += ((float) delta) * 1e-6f * posMeasNed.vel.V.Z;
    }
    latestExtrapolationTime = currentExtrapolationTime;
    */
        // todo: DONE in imu.c extrapolate psi from body rates somehow using the quat
}

void getFakeGps(timeUs_t current) {
    // not that critical, because this is only for display purposes
    // just dump without checking anything
    UNUSED(current);

#if defined(USE_LOCAL_POSITION_PI) && defined(USE_GPS)
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

    if (posMeasState == LOCAL_POS_NO_SIGNAL) {
        DISABLE_STATE(GPS_FIX);
    }
#endif
}

void getPosSetpoint(timeUs_t current) {
    UNUSED(current);

    static timeUs_t latestPosSetpointTime = 0;

#ifdef USE_TELEMETRY_PI
    static timeUs_t latestPiSetpointTime = 0;
    if (piMsgPosSetpointRx) {
        timeUs_t currentPiSetpointTime = piMsgPosSetpointRx->time_ms * 1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentPiSetpointTime, latestPiSetpointTime);
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
            latestPiSetpointTime = currentPiSetpointTime;
            posSpNed.time_ms = currentPiSetpointTime;
            posSpNed.pos.V.X = piMsgPosSetpointRx->enu_y;
            posSpNed.pos.V.Y = piMsgPosSetpointRx->enu_x;
            posSpNed.pos.V.Z = -piMsgPosSetpointRx->enu_z;

            posSpNed.vel.V.X = 0; //piMsgPosSetpointRx->enu_yd;
            posSpNed.vel.V.Y = 0; //piMsgPosSetpointRx->enu_xd;
            posSpNed.vel.V.Z = 0; //-piMsgPosSetpointRx->enu_zd;
            posSpNed.psi = DEGREES_TO_RADIANS(-piMsgPosSetpointRx->yaw);
        }
    }
#endif
    // could also have been set from MSP, just use latest and dont ask questions
    if (posSpNed.time_ms > 0)
        posSpState = LOCAL_POS_STILL_VALID;

    timeUs_t currentPosSetpointTime = posSpNed.time_ms * 1e3;
    timeDelta_t deltaMsgs = cmpTimeUs(currentPosSetpointTime, latestPosSetpointTime);
    if (deltaMsgs != 0) {
        posSpState = LOCAL_POS_NEW_MESSAGE;
        latestPosSetpointTime = currentPosSetpointTime;
    }
}

#endif