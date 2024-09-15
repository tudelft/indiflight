
#include "pi-messages.h"
#include "local_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "flight/imu.h"
#include "io/gps.h"


#if defined(USE_LOCAL_POSITION_GPS) && !defined(USE_GPS)
#error "USE_LOCAL_POSITION_GPS requires USE_GPS"
#endif

#if defined(USE_LOCAL_POSITION_PI) && !defined(USE_TELEMETRY_PI)
#error "USE_LOCAL_POSITION_PI requires USE_TELEMETRY_PI"
#endif

#if defined(USE_LOCAL_POSITION_PI) && !defined(USE_LOCAL_POSITION)
#error "USE_LOCAL_POSITION_PI requires USE_LOCAL_POSITION"
#endif

#if defined(USE_LOCAL_POSITION_GPS) && !defined(USE_LOCAL_POSITION)
#error "USE_LOCAL_POSITION_GPS requires USE_LOCAL_POSITION"
#endif



#ifdef USE_LOCAL_POSITION

#if !defined(USE_LOCAL_POSITION_GPS) && !defined(USE_LOCAL_POSITION_PI)
#error "USE_LOCAL_POSITION requires either USE_LOCAL_POSITION_GPS or USE_LOCAL_POSITION_PI"
#endif

#if defined(USE_LOCAL_POSITION_GPS) && defined(USE_LOCAL_POSITION_PI)
#error "Define only one of USE_LOCAL_POSITION_GPS or USE_LOCAL_POSITION_PI" 
#endif


//extern
local_pos_ned_t posMeasNed;
local_pos_measurement_state_t posMeasState = LOCAL_POS_NO_SIGNAL;
local_pos_setpoint_ned_t posSpNed;
local_pos_measurement_state_t posSpState = LOCAL_POS_NO_SIGNAL;
timeUs_t posLatestMsgTime = 0;
timeUs_t latestMsgGpsTime = 0;

vio_pos_ned_t vioPosNed;
local_pos_measurement_state_t vioPosState = LOCAL_POS_NO_SIGNAL;
timeUs_t vioLatestMsgTime = 0;

#ifdef USE_LOCAL_POSITION_PI
static void checkNewPosPi(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, posLatestMsgTime);
        if (deltaMsgs != 0) {
            posMeasState = LOCAL_POS_NEW_MESSAGE;
            posLatestMsgTime = currentMsgTime;
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

#ifdef USE_LOCAL_POSITION_GPS
static void checkNewPosGps(void) {
    if (!STATE(GPS_FIX_EVER) || !STATE(GPS_FIX_HOME)) {
        posMeasState = LOCAL_POS_NO_SIGNAL;
    } else {
        // assume still valid for now
        posMeasState = LOCAL_POS_STILL_VALID;

        timeDelta_t deltaMsgs = cmpTimeUs(gpsData.lastNavMessage, latestMsgGpsTime);
        if (STATE(GPS_FIX) && (deltaMsgs != 0)) {
            posMeasState = LOCAL_POS_NEW_MESSAGE;
            posLatestMsgTime = micros();
            latestMsgGpsTime = gpsData.lastNavMessage;
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
    checkNewPosPi();
#ifdef USE_LOCAL_POSITION_GPS
    UNUSED(checkNewPosGps);
#endif
#else
    checkNewPosGps();
#endif

    switch (posMeasState) {
        case LOCAL_POS_NEW_MESSAGE:
#ifdef USE_LOCAL_POSITION_PI
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
#else
#define REARTH 6371000.f
            {
                fp_vector_t newPos;
                newPos.V.X = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lat - GPS_home[GPS_LATITUDE]) * REARTH;
                newPos.V.Y = 1e-7f * DEGREES_TO_RADIANS(gpsSol.llh.lon - GPS_home[GPS_LONGITUDE]) * REARTH
                    * cosf(1e-7f * DEGREES_TO_RADIANS(GPS_home[GPS_LONGITUDE]));
                newPos.V.Z = -1e-2f * gpsSol.llh.altCm;
                timeDelta_t delta_ms = cmpTimeUs(gpsData.lastNavMessage, gpsData.lastLastNavMessage); // ms, not us
                if ((delta_ms > 0) && (delta_ms < 1000)) {
                    float iDeltaS = 1. / (0.001f*delta_ms);
                    posMeasNed.vel = newPos;
                    VEC3_SCALAR_MULT_ADD(posMeasNed.vel, -1., posMeasNed.pos);
                    VEC3_SCALAR_MULT(posMeasNed.vel, iDeltaS);
                }
                posMeasNed.pos = newPos;
                posMeasNed.quat.w = 1.f; // nope
                posMeasNed.quat.x = 0.f;
                posMeasNed.quat.y = 0.f;
                posMeasNed.quat.z = 0.f;
            }
#endif
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

#ifdef USE_VIO_POSE
void checkNewVioPos(void) {
    if (piMsgVioPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgVioPoseRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, vioLatestMsgTime);
        if (deltaMsgs != 0) {
            // new message available
            vioPosState = LOCAL_POS_NEW_MESSAGE;
            vioLatestMsgTime = currentMsgTime;
        } else {
            // assume still valid for now
            vioPosState = LOCAL_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(micros(), vioLatestMsgTime);
        if (delta > VIO_POS_TIMEOUT_US) {
            // signal lost
            vioPosState = LOCAL_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        vioPosState = LOCAL_POS_NO_SIGNAL;
    }
}

void getVioPos(timeUs_t current) {
    UNUSED(current);

    checkNewVioPos();
    if (vioPosState == LOCAL_POS_NO_SIGNAL)
        return;

    if (vioPosState == LOCAL_POS_NEW_MESSAGE) {
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

#if defined(USE_LOCAL_POSITION_PI) && defined(USE_GPS)
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

#ifdef USE_TELEMETRY_PI
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
#endif
}

#endif
