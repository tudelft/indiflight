
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
// UGLY HACK ----------------------------------------------
#include "flight/trajectory_tracker.h"
#include "fc/core.h"
// --------------------------------------------------------
// ALSO UGLY HACK -----------------------------------------
#ifdef USE_NN_CONTROL
#include "flight/nn_control.h"
#endif
// --------------------------------------------------------

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;
timeUs_t extLatestMsgTime = 0;

vio_pos_ned_t vioPosNed;
ext_pos_state_t vioPosState = EXT_POS_NO_SIGNAL;
timeUs_t vioLatestMsgTime = 0;

pos_setpoint_ned_t posSetpointNed;
ext_pos_state_t posSetpointState = EXT_POS_NO_SIGNAL;


void checkNewPos(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_ms*1e3;
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
        extPosNed.time_ms = piMsgExternalPoseRx->time_ms;
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
}

#ifdef USE_VIO_POSE
void checkNewVioPos(void) {
    if (piMsgVioPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgVioPoseRx->time_ms*1e3;
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
        vioPosNed.time_ms = piMsgVioPoseRx->time_ms;
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
        timeUs_t currentSetpointTime = piMsgPosSetpointRx->time_ms * 1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentSetpointTime, latestSetpointTime);
        bool newMsg = (deltaMsgs != 0);
        if (newMsg) {
            latestSetpointTime = currentSetpointTime;
            posSetpointNed.pos.V.X = piMsgPosSetpointRx->enu_y;
            posSetpointNed.pos.V.Y = piMsgPosSetpointRx->enu_x;
            posSetpointNed.pos.V.Z = -piMsgPosSetpointRx->enu_z;

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

            // call initRecoveryMode when piMsgPosSetpointRx->enu_xd == 6
            if (piMsgPosSetpointRx->enu_xd == 6) {
                initRecoveryMode();
                return;
            }
#endif
#ifdef USE_NN_CONTROL
            // UGLY HACK: velocity setpoint is used for communication with the neural network controller -----------------
            
            // init when piMsgPosSetpointRx->enu_xd == 4
            if (piMsgPosSetpointRx->enu_xd == 4) {
                nn_init();
                return;
            }

            // activate/deactivate when piMsgPosSetpointRx->enu_xd == 5
            if (piMsgPosSetpointRx->enu_xd == 5) {
                if (nn_is_active()) {
                    nn_deactivate();
                } else {
                    nn_activate();
                }
            }
#endif
            // -----------------------------------------------------------------------------------------------
            
            posSetpointNed.vel.V.X = 0; //piMsgPosSetpointRx->enu_yd;
            posSetpointNed.vel.V.Y = 0; //piMsgPosSetpointRx->enu_xd;
            posSetpointNed.vel.V.Z = 0; //-piMsgPosSetpointRx->enu_zd;
            posSetpointNed.psi = DEGREES_TO_RADIANS(-piMsgPosSetpointRx->yaw);
            posSetpointState = EXT_POS_NEW_MESSAGE;
            
        } else {
            posSetpointState = EXT_POS_STILL_VALID;
            // no upper bound on still_valid for setpoints
        }
    }
}

#endif