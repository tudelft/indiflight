/*
 * Multirotor position mode. PID for accel, dynamic inversion for attitude.
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
 * Copyright 2024 Robin Ferede (Delft University of Technology)
 *     Improved dynamic inversion code that maps desired accel to attitude.
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


#include "pos_ctl.h"

#include "io/local_pos.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "common/maths.h"
#include "common/filter.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"
#include "setupWLS.h"
#include "flight/trajectory_tracker.h"
#include "flight/geofence.h"

#ifdef USE_LOCAL_POSITION

#ifndef USE_INDI
#error "USE_LOCAL_POSITION requires the use of USE_INDI"
#endif

#ifndef USE_EKF
#error "USE_LOCAL_POSITION require the use of USE_EKF"
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(positionProfile_t, POSITION_PROFILE_COUNT, positionProfiles, PG_POSITION_PROFILE, 0);

void pgResetFn_positionProfiles(positionProfile_t *positionProfiles) {
    for (int i = 0; i < POSITION_PROFILE_COUNT; i++) {
        positionProfile_t *p = &positionProfiles[i];
        p->horz_p = 30;
        p->horz_i = 2;
        p->horz_d = 40;
        p->horz_max_v = 250;
        p->horz_max_a = 500;
        p->horz_max_iterm = 500;
        p->max_tilt = 30;
        p->vert_p = 30;
        p->vert_i = 2;
        p->vert_d = 30;
        p->vert_max_v_up = 100;
        p->vert_max_v_down = 100;
        p->vert_max_a_up = 500;
        p->vert_max_a_down = 500;
        p->vert_max_iterm = 50;
        p->yaw_p = 30;
        p->weathervane_p = 0;
        p->weathervane_min_v = 200;
        p->use_spf_attenuation = 1;
    }
}

void posCtlInit(void) {
    initPositionRuntime();
    posRuntime.arrest_motion = false;
    posRuntime.arrest_z_motion_only = false;
}

// filtered stick-based velocity (NED)
static biquadFilter_t posVelFilter[3];
static biquadFilter_t attitudeQuaternionFilter[4];
static biquadFilter_t spfBodyFilter[4];
static biquadFilter_t dfAccelerationFilter[3];
static biquadFilter_t dfModelSpfFilter[3];
static fp_quaternion_t filteredAttitudeQuaternion;
static fp_vector_t filteredSpfBody;
static bool attitudeQuaternionFilterPrimed;

// Edit this plan and POSITION_WAYPOINT_COUNT to define an automatic flight.
// The plan is disabled by default so position mode keeps its existing behavior.
#define POSITION_WAYPOINT_COUNT 2
static const positionWaypoint_t positionWaypoints[POSITION_WAYPOINT_COUNT] = { 
    { .location = { .V = { .X = 0.f,  .Y =  0.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
    { .location = { .V = { .X = 0.f,  .Y =  30.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
    //{ .location = { .V = { .X = 20.f, .Y =  30.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
    //{ .location = { .V = { .X = 20.f, .Y = -30.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
    //{ .location = { .V = { .X = 0.f,  .Y = -30.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
    //{ .location = { .V = { .X = 0.f,  .Y = - 0.f, .Z = -5.f } }, .tolerance = 3.f, .heightTolerance = 3.f, .velocityLimit = 10.f },
};

static unsigned int activeWaypointIndex;
static bool waypointRunActive;

static bool updateWaypointTraversal(bool allowWaypoints)
{
    if (!allowWaypoints || POSITION_WAYPOINT_COUNT == 0) {
        waypointRunActive = false;
        activeWaypointIndex = 0;
        return false;
    }

    if (!waypointRunActive) {
        waypointRunActive = true;
        activeWaypointIndex = 0;
    }

    while (activeWaypointIndex + 1 < POSITION_WAYPOINT_COUNT) {
        const positionWaypoint_t *waypoint = &positionWaypoints[activeWaypointIndex];
        const float northError = waypoint->location.V.X - posEstNed.V.X;
        const float eastError = waypoint->location.V.Y - posEstNed.V.Y;
        const float downError = waypoint->location.V.Z - posEstNed.V.Z;
        const float horizontalDistance = sqrtf(northError * northError + eastError * eastError);

        if (horizontalDistance > waypoint->tolerance || fabsf(downError) > waypoint->heightTolerance) {
            break;
        }

        activeWaypointIndex++;
    }

    posSpNed.pos = positionWaypoints[activeWaypointIndex].location;
    posSpNed.valid = true;
    posSpNed.trackPsi = false;
    return true;
}

static float activeWaypointVelocityLimit(void)
{
    if (waypointRunActive && activeWaypointIndex < POSITION_WAYPOINT_COUNT) {
        return positionWaypoints[activeWaypointIndex].velocityLimit;
    }

    return 0.f;
}

positionRuntime_t posRuntime;
void initPositionRuntime(void) {
    const positionProfile_t* p = positionProfiles(systemConfig()->positionProfileIndex);
    posRuntime.horz_p = p->horz_p * 0.1f;
    posRuntime.horz_i = p->horz_i * 0.1f;
    posRuntime.horz_d = MAX(p->horz_d, 1) * 0.1f;
    posRuntime.horz_max_v = p->horz_max_v * 0.01f;
    posRuntime.horz_max_a = p->horz_max_a * 0.01f; 
    posRuntime.horz_max_iterm = p->horz_max_iterm * 0.01f;
    posRuntime.max_tilt = DEGREES_TO_RADIANS(p->max_tilt);
    posRuntime.vert_p = p->vert_p * 0.1f;
    posRuntime.vert_i = p->vert_i * 0.1f;
    posRuntime.vert_d = MAX(p->vert_d, 1) * 0.1f;
    posRuntime.vert_max_v_up = p->vert_max_v_up * 0.01f;
    posRuntime.vert_max_v_down = p->vert_max_v_down * 0.01f;
    posRuntime.vert_max_a_up = p->vert_max_a_up * 0.01f;
    posRuntime.vert_max_a_down = p->vert_max_a_down * 0.01f;
    posRuntime.vert_max_iterm = p->vert_max_iterm * 0.01f;
    posRuntime.yaw_p = p->yaw_p * 0.1f;
    posRuntime.weathervane_p = p->weathervane_p * 0.1f;
    posRuntime.weathervane_min_v = p->weathervane_min_v * 0.01f;
    posRuntime.use_spf_attenuation = (bool) p->use_spf_attenuation;

    // init stick velocity filters (2nd order LPF)
    for (int i = 0; i < 3; i++) {
        biquadFilterInitLPF(&posVelFilter[i], 5.0f, 500); // TODO: make dynamic
        biquadFilterInitLPF(&spfBodyFilter[i], 5.0f, 500);
        // biquadFilterInit(&dfAccelerationFilter[i], sqrtf(0.5f * 5.0f), 500,
        //     sqrtf(0.5f * 5.0f) / (5.0f - 0.5f), FILTER_BPF, 1.0f);
        biquadFilterInitLPF(&dfAccelerationFilter[i], 10.0f, 500);
        biquadFilterInitLPF(&dfModelSpfFilter[i], 10.0f, 500);
    }
    for (int i = 0; i < 4; i++) {
        biquadFilterInitLPF(&attitudeQuaternionFilter[i], 5.0f, 500);
    }
    filteredAttitudeQuaternion = (fp_quaternion_t) { .w = 1.f };
    filteredSpfBody = (fp_vector_t) { 0 };
    attitudeQuaternionFilterPrimed = false;
}

void changePositionProfile(uint8_t profileIndex)
{
    if (profileIndex < POSITION_PROFILE_COUNT) {
        systemConfigMutable()->positionProfileIndex = profileIndex;
    }

    initPositionRuntime();
}

// --- control variables
// externs
fp_vector_t accSpNedFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 1. };
fp_quaternion_t attSpNedFromPos = { .w = 1., .x = 0., .y = 0., .z = 0. };
fp_vector_t spfSpBodyFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 1. };
fp_vector_t rateSpBodyFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 0. };

// locals
fp_vector_t velIErrorBody = {0};
void resetIterms(void) {
    velIErrorBody.V.X = 0.f;
    velIErrorBody.V.Y = 0.f;
    velIErrorBody.V.Z = 0.f;
}

void posArrestMotion(void) {
    posRuntime.arrest_motion = true;
    posSpNed.valid = true;
    resetIterms();
}

void posArrestZMotionOnly(void) {
    posRuntime.arrest_motion = true;
    posRuntime.arrest_z_motion_only = true;
    posSpNed.valid = true;
    resetIterms();
}

void updatePosCtl(timeUs_t current) {
    timeDelta_t timeInDeadreckoning = cmpTimeUs(current, posMeasNed.time_us);
    static bool latch_descend = false;
    static bool manual_takeover = false;

    if (manual_takeover && posSpNed.valid) {
        // no more manual control because new setpoint received
        // reset sticks, so that we can detect new stick movement later
        manual_takeover = false;
        setSticksReference();
    } else if (!manual_takeover && ARMING_FLAG(ARMED) && haveSticksMoved()) {
        manual_takeover = true;
        posSpNed.valid = false;
#ifdef USE_TRAJECTORY_TRACKER
        if (isActiveTrajectoryTracker()) {
            stopTrajectoryTracker();
        }
#endif
    }

    const bool waypointActive = updateWaypointTraversal(
        !manual_takeover && ARMING_FLAG(ARMED) && FLIGHT_MODE(POSITION_MODE));
    UNUSED(waypointActive);

    static timeUs_t lastHoldSpTimeUs = 0;
#ifdef USE_GEOFENCE
    if (geofenceAction == GEOFENCE_ACTION_HOLD) {
        if ((lastHoldSpTimeUs > 0) && (
                (cmpTimeUs(posSpNed.time_us, lastHoldSpTimeUs) && posSpNed.valid) // new setpoint sent from somewhere
                || (manual_takeover)                                              // or manual takeover
            )) 
        {
            geofenceClearHold();
            lastHoldSpTimeUs = 0;
        }
    }
#else
    UNUSED(lastHoldSpTimeUs);
#endif

    if ( latch_descend
            || (!posSpNed.valid && !manual_takeover) || !isConvergedEkf()
            || (timeInDeadreckoning > DEADRECKONING_TIMEOUT_DESCEND_SLOWLY_US) ) {
        // panic and level craft in slight downwards motion
        accSpNedFromPos.V.X = 0.f;
        accSpNedFromPos.V.Y = 0.f;
        accSpNedFromPos.V.Z = 2.f; // slight downwards motion
        rateSpBodyFromPos.V.X = 0.f;
        rateSpBodyFromPos.V.Y = 0.f;
        rateSpBodyFromPos.V.Z = 0.f;
        posSpNed.trackPsi = false;

        // latch reactivation until new arming cycle or non-position mode
        latch_descend = ARMING_FLAG(ARMED) && FLIGHT_MODE(POSITION_MODE);
    } else if (timeInDeadreckoning > DEADRECKONING_TIMEOUT_HOLD_POSITION_US
#ifdef USE_GEOFENCE
                || (geofenceAction == GEOFENCE_ACTION_HOLD)
                || (geofenceAction == GEOFENCE_ACTION_DESCEND)
#endif
        ) {
        // more than 2 sec but less than 3.5 seconds --> arrest motion
#ifdef USE_TRAJECTORY_TRACKER
        updateTrajectoryTracker(current);
        if (isActiveTrajectoryTracker() && !isActiveTrajectoryTrackerRecovery()) {
            stopTrajectoryTracker();
        }
        if (isActiveTrajectoryTrackerRecovery()) {
            lastHoldSpTimeUs = current;
        } else 
#endif
        {
            posSpNed.pos = posEstNed; // hold position
            posSpNed.vel.V.X = 0.;
            posSpNed.vel.V.Y = 0.;
            posSpNed.vel.V.Z = 0.;
            posSpNed.valid = true; // simulate new message
            posSpNed.time_us = current - 1; // avoid case that messages are in sync (e.g. simulation)
            lastHoldSpTimeUs = posSpNed.time_us;
            posGetVelSpNedFromPosSp();

#ifdef USE_GEOFENCE
            if (geofenceAction == GEOFENCE_ACTION_DESCEND) {
                posSpNed.vel.V.Z = 1.; // 1 m/s downwards
            }
#endif

            posGetAccSpNed(current);
            rateSpBodyFromPos.V.X = 0; // TODO: implement weathervaning?
            rateSpBodyFromPos.V.Y = 0;
            rateSpBodyFromPos.V.Z = 0;
        }
    } else {
        // not deadreckoning for too long, setpoint valid. lets fly
        if ( (!ARMING_FLAG(ARMED)) || (!FLIGHT_MODE(POSITION_MODE | VELOCITY_MODE | GPS_RESCUE_MODE)) ) {
            resetIterms();
        }

#ifdef USE_TRAJECTORY_TRACKER
        // use acc and body rate setpoints from trajectory tracker if it is active
        updateTrajectoryTracker(current);
        if (!isActiveTrajectoryTracker() || manual_takeover)
#endif
        {
            if (manual_takeover) {
                posGetVelSpNedFromSticks();

                // yaw stuff
                posSpNed.trackPsi = false;
                rateSpBodyFromPos = extrinsicYaw(DEGREES_TO_RADIANS(getSetpointRate(YAW)));
            } else if (posRuntime.arrest_motion) {
                // just command zero velocity until it is reached
                if (posRuntime.arrest_z_motion_only && posSpNed.valid) {
                    posGetVelSpNedFromPosSp();
                    posSpNed.vel.V.Z = 0.f;
                } else {
                    posRuntime.arrest_z_motion_only = false;
                    posSpNed.vel.V.X = 0.f;
                    posSpNed.vel.V.Y = 0.f;
                    posSpNed.vel.V.Z = 0.f;
                }
                posSpNed.trackPsi = false;
                rateSpBodyFromPos.V.X = 0;
                rateSpBodyFromPos.V.Y = 0;
                rateSpBodyFromPos.V.Z = 0;
#define ARREST_MOTION_VEL_THRESHOLD 0.5f
                if (VEC3_LENGTH(velEstNed) < ARREST_MOTION_VEL_THRESHOLD) {
                    posRuntime.arrest_motion = false;
                    if (posRuntime.arrest_z_motion_only) {
                        posSpNed.pos.V.Z = posEstNed.V.Z;
                    } else {
                        posSpNed.pos = posEstNed; // reset position setpoint to current position
                    }
                    posSpNed.valid = true; // simulate new message
                    posSpNed.time_us = current;
                    resetIterms();
                }
            } else {
                // normal position control
                posGetVelSpNedFromPosSp();
                rateSpBodyFromPos.V.X = 0; // TODO: implement weathervaning?
                rateSpBodyFromPos.V.Y = 0;
                rateSpBodyFromPos.V.Z = 0;
            }

            // convert velocity setpoint to acceleration setpoint
            posGetAccSpNed(current);
        }
    }

    // always use NDI function to map acc setpoints
    // posGetAttSpNedAndSpfSpBody(current);
    // posGetAttSpNedAndSpfSpBody_INDI(current);
    posGetAttSpNedAndSpfSpBody_DF(current);
}

void posGetVelSpNedFromPosSp(void) {
    // precalculations
    float horzPCasc = posRuntime.horz_p / posRuntime.horz_d; // emulate parallel PD with Casc system
    float vertPCasc = posRuntime.vert_p / posRuntime.vert_d; // emulate parallel PD with Casc system

    // pos error = pos setpoint - pos estimate
    fp_vector_t posError = posSpNed.pos;
    VEC3_SCALAR_MULT_ADD(posError, -1.0f, posEstNed); // posMeasNed.pos

    // vel setpoint = posGains * posError
    posSpNed.vel.V.X = posError.V.X * horzPCasc;
    posSpNed.vel.V.Y = posError.V.Y * horzPCasc;
    posSpNed.vel.V.Z = posError.V.Z * vertPCasc;

    // constrain magnitude here
    VEC3_CONSTRAIN_XY_LENGTH(posSpNed.vel, posRuntime.horz_max_v);

    posSpNed.vel.V.Z = constrainf(posSpNed.vel.V.Z, -posRuntime.vert_max_v_up, posRuntime.vert_max_v_down);

    const float waypointVelocityLimit = activeWaypointVelocityLimit();
    if (waypointVelocityLimit > 0.f) {
        const float velocityLength = VEC3_LENGTH(posSpNed.vel);
        if (velocityLength > waypointVelocityLimit) {
            VEC3_SCALAR_MULT(posSpNed.vel, waypointVelocityLimit / velocityLength);
        }
    }
}

void posGetVelSpNedFromSticks(void) {
    static fp_vector_t velEstNedFilt = {0};
    static bool useVelocityBearing = false;

#define POSCTL_VEL_BEARING_THRESH_CMS 300

    // get velocity setpoints from sticks in body frame
    float velSpBodyX = -getRcDeflection(PITCH) * posRuntime.horz_max_v;
    float velSpBodyY = getRcDeflection(ROLL) * posRuntime.horz_max_v;

    // hysteresis: enter velocity-bearing when speed > 3.0 m/s, exit when < 2.5 m/s
    velEstNedFilt.V.X = biquadFilterApply(&posVelFilter[0], velEstNed.V.X);
    velEstNedFilt.V.Y = biquadFilterApply(&posVelFilter[1], velEstNed.V.Y);
    float xy_groundspeed = VEC3_XY_LENGTH(velEstNedFilt);
    if (useVelocityBearing) {
        if (xy_groundspeed < (POSCTL_VEL_BEARING_THRESH_CMS*0.01f - 0.5f)) {
            useVelocityBearing = false;
        }
    } else {
        if (xy_groundspeed > (POSCTL_VEL_BEARING_THRESH_CMS*0.01f)) {
            useVelocityBearing = true;
        }
    }

    // use inferred bearing angle to convert to NED frame
    float cPsi, sPsi;
    if (useVelocityBearing && (xy_groundspeed > 1e-6f)) {
        cPsi = velEstNedFilt.V.X / xy_groundspeed;
        sPsi = velEstNedFilt.V.Y / xy_groundspeed;
    } else {
        float Psi = getYawWithoutSingularity();
        cPsi = cos_approx(Psi);
        sPsi = sin_approx(Psi);
    }

    // convert sticks
    posSpNed.vel.V.X = velSpBodyX * cPsi - velSpBodyY * sPsi;
    posSpNed.vel.V.Y = velSpBodyX * sPsi + velSpBodyY * cPsi;

    // vertical velocity from throttle
    float normThrottle = (rcCommand[THROTTLE] - 1000.f) * 1e-3f; // 0..1
    normThrottle -= 0.5f; // -0.5 .. +0.5
    normThrottle *= 2.f; // -1 .. +1
    posSpNed.vel.V.Z = (normThrottle) > 0 ? (-normThrottle * posRuntime.vert_max_v_up) : (-normThrottle * posRuntime.vert_max_v_down);
}

void posGetAccSpNed(timeUs_t current) {
    // if (cmpTimeUs(current, 6000000) > 0) {
    //     posSpNed.vel.V.X = 0.f;
    //     posSpNed.vel.V.Y = 10.f;
    //     posSpNed.vel.V.Z = 0.f;
    // }

    // vel error = vel setpoint - vel estimate
    fp_vector_t velError = posSpNed.vel;

    VEC3_SCALAR_MULT_ADD(velError, -1.0f, velEstNed);

    // use quaternion attitude to decompose velocity error to body frame
    fp_quaternion_t quat, iquat;
    getHoverAttitudeQuaternion(&quat);
    iquat = quat;
    iquat.w = -iquat.w;

    // rotate velError to body frame
    fp_vector_t velErrorBody;
    velErrorBody = velError;
    rotate_vector_with_quaternion(&velErrorBody, &iquat);

    static bool accSpXYSaturated = true;
    static bool accSpZSaturated = true;
    static timeUs_t lastCall = 0;
    timeDelta_t delta = cmpTimeUs(current, lastCall);
    if ((lastCall > 0) && (delta > 0) && (delta < 50000)) {
        if (!accSpXYSaturated) {
            velIErrorBody.V.X += delta * 1e-6f * velErrorBody.V.X;
            velIErrorBody.V.Y += delta * 1e-6f * velErrorBody.V.Y;
        }

        if (!accSpZSaturated) {
            velIErrorBody.V.Z += delta * 1e-6f * velErrorBody.V.Z;
        }

        VEC3_CONSTRAIN_XY_LENGTH(velIErrorBody, posRuntime.horz_max_iterm);
        velIErrorBody.V.Z = constrainf(velIErrorBody.V.Z, -posRuntime.vert_max_iterm, posRuntime.vert_max_iterm);
    }
    lastCall = current;

    // acceleration setpoint = velGains * velError
    fp_vector_t velIError;
    velIError = velIErrorBody;
    rotate_vector_with_quaternion(&velIError, &quat);

    accSpNedFromPos.V.X = velError.V.X * posRuntime.horz_d  +  velIError.V.X * posRuntime.horz_i;
    accSpNedFromPos.V.Y = velError.V.Y * posRuntime.horz_d  +  velIError.V.Y * posRuntime.horz_i;
    accSpNedFromPos.V.Z = velError.V.Z * posRuntime.vert_d  +  velIError.V.Z * posRuntime.vert_i;

    // limit such that max acceleration likely results in bank angle below 40 deg
    // but log if acceleration saturated, so we can pause error integration
    accSpXYSaturated = VEC3_XY_LENGTH(accSpNedFromPos) > posRuntime.horz_max_a;
    accSpZSaturated = (accSpNedFromPos.V.Z < -posRuntime.vert_max_a_up) || (accSpNedFromPos.V.Z > posRuntime.vert_max_a_down);

    VEC3_CONSTRAIN_XY_LENGTH(accSpNedFromPos, posRuntime.horz_max_a);
    accSpNedFromPos.V.Z = constrainf(accSpNedFromPos.V.Z, -posRuntime.vert_max_a_up, posRuntime.vert_max_a_down);
}


static void getSpfInertialPhi(
    fp_vector_t* f_I,
    const float** PHI,
    const fp_vector_t* AX,
    const float* K,
    const float* omega,
    const int n,
    const fp_quaternion_t* q,
    const fp_vector_t* v_I
)
{
    // equation: 
    // f_I = R @ (        f_w_B           +          f_T_B          )
    // f_I = R @ ( -V * PHI @ R.T @ v_I   +   ax_B * K.T * omega**2 )

    // term f_w_B
    fp_quaternion_t qinv = *q;
    qinv.w *= -1.f;
    fp_vector_t v_B = *v_I; // R.T @ v_I
    rotate_vector_with_quaternion(&v_B, &qinv);

    fp_vector_t f_w_B = { 0 }; // PHI @ R.T @ v_I
    for (int row=0; row < 3; row++) {
        for (int col=0; col < 3; col++) {
            f_w_B.A[row] += PHI[row][col] * v_B.A[col];
        }
    }

    float V = sqrtf(sq(v_I->V.X) + sq(v_I->V.Y) + sq(v_I->V.Z));
    VEC3_SCALAR_MULT(f_w_B, -V); // -V * PHI @ R.T @ v_I

    // term f_T_B
    fp_vector_t f_T_B = *AX;
    float T = 0;
    for (int i=0; i < n; i++) {
        T += K[i]*sq(omega[i]);
    }
    VEC3_SCALAR_MULT(f_T_B, T);

    // add terms
    *f_I = f_w_B;
    VEC3_SCALAR_MULT_ADD((*f_I), 1.f, f_T_B);

    // rotate into inertial frame
    rotate_vector_with_quaternion(f_I, q);
}

static bool spfDFT(
    fp_quaternion_t *q_r,
    float *T_r,
    const float Psi_r,
    const fp_vector_t* f_I_r,
    const float** PHI,
    const fp_vector_t* AX,
    const fp_vector_t* v_I
)
{
    // Tal (2022)
    // bank angle phi_r
    float sPsi = sin_approx(Psi_r);
    float cPsi = cos_approx(Psi_r);
    float num = f_I_r->V.X * sPsi - f_I_r->V.Y * cPsi;
    float den = f_I_r->V.Z;

    if ((fabsf(num) < 1e-6) && (fabsf(den) < 1e-6)) {
        return false;
    }

    float phi_r = atan2_approx(num, den);

    // check modulo pi
    if (phi_r > 0.5f*M_PIf) {
        phi_r -= M_PIf;
    } else if (phi_r < -0.5f*M_PIf) {
        phi_r += M_PIf;
    }


    // pitch angle
    // theta_cos_terms * cos(theta_r) + theta_sin_terms * sin(theta_r) == 0
    //     theta_cos_terms / theta_sin_terms  +  sin(theta_r) / cos(theta_r) == 0
    //     tan(theta_r) = - theta_cos_terms / theta_sin_terms
    float theta_r, theta_cos_terms, theta_sin_terms;

    fp_euler_t e_phi = { .angles = { .yaw = Psi_r, .roll = phi_r, .pitch = 0.f } };
    fp_quaternion_t q_phi_inv;
    quaternion_of_fp_euler(&q_phi_inv, &e_phi);
    q_phi_inv.w *= -1.f;

    fp_vector_t f_phi_r = *f_I_r;
    rotate_vector_with_quaternion(&f_phi_r, &q_phi_inv);
    fp_vector_t v_phi = *v_I;
    rotate_vector_with_quaternion(&v_phi, &q_phi_inv);

    float V = sqrtf(sq(v_I->V.X) + sq(v_I->V.Y) + sq(v_I->V.Z));
    theta_cos_terms  = - V * PHI[0][0] * AX->V.Z * v_phi.V.X;
    theta_cos_terms +=   V * PHI[2][2] * AX->V.X * v_phi.V.Z;
    theta_cos_terms +=   AX->V.X * f_phi_r.V.Z;
    theta_cos_terms += - AX->V.Z * f_phi_r.V.X;

    theta_sin_terms  =   V * PHI[0][0] * AX->V.Z * v_phi.V.Z;
    theta_sin_terms +=   V * PHI[2][2] * AX->V.X * v_phi.V.X;
    theta_sin_terms +=   AX->V.X * f_phi_r.V.X;
    theta_sin_terms +=   AX->V.Z * f_phi_r.V.Z;

    if ((fabsf(theta_cos_terms) < 1e-6) && (fabsf(theta_sin_terms) < 1e-6)) {
        return false;
    }

    theta_r = atan2_approx(-theta_cos_terms, theta_sin_terms);

    fp_euler_t e_Psi_phi_r = {.angles={.yaw=Psi_r, .pitch=0.f, .roll=phi_r}};
    fp_quaternion_t q_Psi_phi_r;
    quaternion_of_fp_euler(&q_Psi_phi_r, &e_Psi_phi_r);

    fp_euler_t e_theta_r = {.angles={.yaw=0.f, .pitch=theta_r, .roll=0.f}};
    fp_quaternion_t q_theta_r;
    quaternion_of_fp_euler(&q_theta_r, &e_theta_r);

    *q_r = chain_quaternion(&q_Psi_phi_r, &q_theta_r);

    float sTheta_r, cTheta_r;
    sTheta_r = sin_approx(theta_r);
    cTheta_r = cos_approx(theta_r);
    *T_r  =   V * PHI[2][2] * v_phi.V.X * sTheta_r;
    *T_r +=   V * PHI[2][2] * v_phi.V.Z * cTheta_r;
    *T_r +=   f_phi_r.V.Z / AX->V.Z     * sTheta_r;
    *T_r +=   f_phi_r.V.Z / AX->V.Z     * cTheta_r;

    return true;
}

void posGetAttSpNedAndSpfSpBody_DF(timeUs_t current) {
    UNUSED(current);
    // state
    fp_quaternion_t q;
    getHoverAttitudeQuaternion(&q);

    fp_vector_t a_I;
    fp_vector_t gravity_I = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -GRAVITYf };
    a_I = indiRun.spfIMU;
    rotate_vector_with_quaternion(&a_I, &q);
    VEC3_SCALAR_MULT_ADD(a_I, -1.f, gravity_I);

    fp_vector_t v_I = velEstNed;

    // craft parameters
    float PHI[3][3] = {
        { 0.000f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.000f },
    };
    const float *PHIRows[3] = { PHI[0], PHI[1], PHI[2] };
    const fp_vector_t AX = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -1.f };
    // const float K[2] = { 1.413e-6f/0.7f, 1.413e-6f/0.7f }; // propeller constant divided by mass
    // const float* omega = indiRun.omega; // just use indiRun.omega
    const float K[4] = { 1.88e-7f/0.41f, 1.88e-7f/0.41f, 1.88e-7f/0.41f, 1.88e-7f/0.41f }; // propeller constant divided by mass
    const float* omega = indiRun.omega; // just use indiRun.omega

    // step 1: calculate INDI update on specfic force level
    //
    //     f_I_r = (a_I_r - a_I_bpf) + f_I_lpf
    //
    //     where:
    //         a_I_r = accSpNedFromPos; // acceleration setpoint
    //         a_I_bpf: band-pass filtered (lpf _and_ hpf) version of the a_I
    //         f_I_lpf: low-pass filtered version of the current estimated f_I from getSpfInertiaPhi()
    //
    // step 2: calculate required attitude and thrust using the inversion spfDFT()

    // Preserve current yaw unless the position setpoint explicitly tracks yaw.
    float Psi_r = posSpNed.trackPsi ? posSpNed.psi : getYawWithoutSingularity();

    fp_vector_t a_I_bpf;
    fp_vector_t f_I_model;
    fp_vector_t f_I_lpf;
    getSpfInertialPhi(&f_I_model, PHIRows, &AX, K, omega, 4, &q, &v_I);
    for (int axis = 0; axis < 3; axis++) {
        a_I_bpf.A[axis] = biquadFilterApply(&dfAccelerationFilter[axis], a_I.A[axis]);
        f_I_lpf.A[axis] = biquadFilterApply(&dfModelSpfFilter[axis], f_I_model.A[axis]);
    }

    fp_vector_t f_I_r = accSpNedFromPos;
    VEC3_SCALAR_MULT_ADD(f_I_r, -1.f, a_I_bpf);
    VEC3_SCALAR_MULT_ADD(f_I_r, 1.f, f_I_lpf);

    fp_quaternion_t q_r;
    float T_r;
    if (spfDFT(&q_r, &T_r, Psi_r, &f_I_r, PHIRows, &AX, &v_I)) {
        attSpNedFromPos = q_r;
        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = -MAX(0.f, T_r);
    } else {
        attSpNedFromPos = q;
        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = AX.V.Z * (K[0] * sq(omega[0]) + K[1] * sq(omega[1]));
    }
}

static void getAttitudeThrustJacobian(
    float** J,
    const float** A_B,
    const fp_vector_t* ax_B,
    const fp_quaternion_t* q_0,
    const fp_vector_t* f_B_0,
    const fp_vector_t* v_I
) {
    /* output
        J: 3x4 jacobian that maps [dRx, dRy, dRz, d(||f||)] to [dfIx, dfIy, dfIz], 
           where dRx, dRy, dRz is small attitude increment in rad/s
           and ||f|| is the thrust
           and dfI is inertial specific forces
       inputs:
        A_B: 3x3 phi_r matrix mapping body air velocity to body aerodynamic forces
        ax_B: thrust axis in body-frame (unit vector)
        q_0: attitude quaterion
        f_B_0: current specific forces in body frame
        v_I: current air velocity in inertial frame
    */
    float a11 = A_B[1][1];
    float a22 = A_B[2][2];
    float axx = ax_B->V.X;
    float axy = ax_B->V.Y;
    float axz = ax_B->V.Z;
    float w = q_0->w;
    float x = q_0->x;
    float y = q_0->y;
    float z = q_0->z;
    float fBz = -f_B_0->V.Z;
    float vIx = v_I->V.X;
    float vIy = v_I->V.Y;
    float vIz = v_I->V.Z;
    float V = sqrtf(sq(vIx) + sq(vIy) + sq(vIz));

    UNUSED(V);
    UNUSED(axx);
    UNUSED(axy);
    UNUSED(axz);
    UNUSED(a11);
    UNUSED(a22);

    // J[0][0] = -V*a11*y*vIy - V*a22*z*vIz + axx*fBz*x + axy*fBz*y + axz*fBz*z;
    // J[0][1] = -V*a11*x*vIy + V*a22*w*vIz - axx*fBz*y + axy*fBz*x - axz*fBz*w;
    // J[0][2] = -V*a11*w*vIy - V*a22*x*vIz - axx*fBz*z + axy*fBz*w + axz*fBz*x;
    // J[0][3] = axx*pow(w, 2) + axx*pow(x, 2) - axx*pow(y, 2) - axx*pow(z, 2) + 2*axy*w*z + 2*axy*x*y - 2*axz*w*y + 2*axz*x*z;
    // J[1][0] = V*a11*x*vIy - V*a22*w*vIz + axx*fBz*y - axy*fBz*x + axz*fBz*w;
    // J[1][1] = -V*a11*y*vIy - V*a22*z*vIz + axx*fBz*x + axy*fBz*y + axz*fBz*z;
    // J[1][2] = V*a11*z*vIy - V*a22*y*vIz - axx*fBz*w - axy*fBz*z + axz*fBz*y;
    // J[1][3] = -2*axx*w*z + 2*axx*x*y + axy*pow(w, 2) - axy*pow(x, 2) + axy*pow(y, 2) - axy*pow(z, 2) + 2*axz*w*x + 2*axz*y*z;
    // J[2][0] = V*a11*w*vIy + V*a22*x*vIz + axx*fBz*z - axy*fBz*w - axz*fBz*x;
    // J[2][1] = -V*a11*z*vIy + V*a22*y*vIz + axx*fBz*w + axy*fBz*z - axz*fBz*y;
    // J[2][2] = -V*a11*y*vIy - V*a22*z*vIz + axx*fBz*x + axy*fBz*y + axz*fBz*z;
    // J[2][3] = 2*axx*w*y + 2*axx*x*z - 2*axy*w*x + 2*axy*y*z + axz*pow(w, 2) - axz*pow(x, 2) - axz*pow(y, 2) + axz*pow(z, 2);

    J[0][0] = -V*a11*y*vIy - V*a22*z*vIz - fBz*z;
    J[0][1] = -V*a11*x*vIy + V*a22*w*vIz - fBz*w;
    J[0][2] = -V*a11*w*vIy - V*a22*x*vIz - fBz*x;
    J[0][3] = -2*w*y - 2*x*z;
    J[1][0] = V*a11*x*vIy - V*a22*w*vIz + fBz*w;
    J[1][1] = -V*a11*y*vIy - V*a22*z*vIz - fBz*z;
    J[1][2] = V*a11*z*vIy - V*a22*y*vIz - fBz*y;
    J[1][3] = 2*w*x - 2*y*z;
    J[2][0] = V*a11*w*vIy + V*a22*x*vIz + fBz*x;
    J[2][1] = -V*a11*z*vIy + V*a22*y*vIz + fBz*y;
    J[2][2] = -V*a11*y*vIy - V*a22*z*vIz - fBz*z;
    J[2][3] = -pow(w, 2) + pow(x, 2) + pow(y, 2) - pow(z, 2);
}

static void getYawConstraintRowZXY(
    float* c,
    const fp_quaternion_t* q
) {
    /* output
        c: 3-element vector such that   [dRx dRy dRz] c = 0  if
          Yaw(q) = Yaw(q * [dRx dRy dRz]), where '*' denotes rotation
          Yaw(.) here is defined with ZXY rotation order
       input
        q: attitude quaternion
    */
    c[0] = -2.f * q->w*q->y + 2.f * q->x*q->z;
    c[1] = 0.f;
    c[2] = -2.f * q->x*q->x - 2.f * q->y*q->y + 1.f;
}

void posGetAttSpNedAndSpfSpBody_INDI(timeUs_t current) {
    UNUSED(current);
    /*
     * with indi we use an incremental model of the form
     *
     *   Delta aI  \approx   J * Delta Chi  (1)
     *
     * where:
     *
     *   Delta aI = aI_r - aI_0 = aI_r  -  ( fI_0  -  gI )
     *            = aI_r - ( q_0 fB_0 q_0^-  -  gI )
     *
     *   aI_r: desired linear accelerations in inertial frame
     *   fI_0: filtered accelerometer reading transformed to inertial frame using filtered attitude q_0
     *   gI: (0, 0, -9.81)
     *
     *   Delta Chi = (dx, dy, dz, Delta fBz): delta rotation reference and delta thrust
     * 
     *      we can recover attitude reference q_r and thrust reference fBz_r from 
     *      Delta Chi and the filtered current q_0 and accelerometer fBz_0
     *
     *   J: 3x4 matrix of partial derivatives of the kinetic relationship between the two at the current airspeed vector
     * 
     * We can solve (1) for Delta Chi by incorperating a constraint that makes
     * sure that the euler yaw of q_r stays unchanged from q_0. That constraint
     * is 
     * 
     *    0 = C(q_0) Delta Chi
     * 
     */
//     enum { N_INDI_OUTPUTS = 4, N_INDI_VARIABLES = 4 };
#define N_INDI_OUTPUTS 4
#define N_INDI_VARIABLES 4

    float J[4][4] = { 0 }; // last row is C(q_0)
    float *JRows[4] = { J[0], J[1], J[2], J[3] };
    float c[4] = { 0 };
    fp_quaternion_t q_0;
    getHoverAttitudeQuaternion(&q_0);
    if (attitudeQuaternionFilterPrimed) {
        const float quaternionDot = q_0.w * filteredAttitudeQuaternion.w
            + q_0.x * filteredAttitudeQuaternion.x
            + q_0.y * filteredAttitudeQuaternion.y
            + q_0.z * filteredAttitudeQuaternion.z;
        if (quaternionDot < 0.f) {
            QUAT_SCALAR_MULT(q_0, -1.f);
        }
    }
    q_0.w = biquadFilterApply(&attitudeQuaternionFilter[0], q_0.w);
    q_0.x = biquadFilterApply(&attitudeQuaternionFilter[1], q_0.x);
    q_0.y = biquadFilterApply(&attitudeQuaternionFilter[2], q_0.y);
    q_0.z = biquadFilterApply(&attitudeQuaternionFilter[3], q_0.z);
    const float quaternionNorm = sqrtf(sq(q_0.w) + sq(q_0.x) + sq(q_0.y) + sq(q_0.z));
    if (quaternionNorm > 1e-6f) {
        const float inverseQuaternionNorm = 1.f / quaternionNorm;
        q_0.w *= inverseQuaternionNorm;
        q_0.x *= inverseQuaternionNorm;
        q_0.y *= inverseQuaternionNorm;
        q_0.z *= inverseQuaternionNorm;
        filteredAttitudeQuaternion = q_0;
        attitudeQuaternionFilterPrimed = true;
    } else if (attitudeQuaternionFilterPrimed) {
        q_0 = filteredAttitudeQuaternion;
    }

    fp_vector_t aI_r = accSpNedFromPos; 
    // fp_vector_t aI_r = { .V.X=0, .V.Y=5.f, .V.Z=-0.5f };
    fp_vector_t gI = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -GRAVITYf };

    fp_vector_t fB_0 = indiRun.spfIMU; // TODO: filter this with LP at 5Hz
    filteredSpfBody.V.X = biquadFilterApply(&spfBodyFilter[0], fB_0.V.X);
    filteredSpfBody.V.Y = biquadFilterApply(&spfBodyFilter[1], fB_0.V.Y);
    filteredSpfBody.V.Z = biquadFilterApply(&spfBodyFilter[2], fB_0.V.Z);
    fp_vector_t fI_0 = filteredSpfBody;
    rotate_vector_with_quaternion(&fI_0, &q_0);

    float LHS[4]; // left hand side of equation
    LHS[0] = aI_r.V.X  -  ( fI_0.V.X - gI.V.X );
    LHS[1] = aI_r.V.Y  -  ( fI_0.V.Y - gI.V.Y );
    LHS[2] = aI_r.V.Z  -  ( fI_0.V.Z - gI.V.Z );
    LHS[3] = 0.f; // constrant row forced to be zero

    float PHI[3][3] = {
        { 0.000f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.000f },
    };
    const float *PHIRows[3] = { PHI[0], PHI[1], PHI[2] };

    fp_vector_t axB = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -1.f };
    static fp_vector_t velEstNedFilt = { 0 };
    velEstNedFilt.V.X = biquadFilterApply(&posVelFilter[0], velEstNed.V.X);
    velEstNedFilt.V.Y = biquadFilterApply(&posVelFilter[1], velEstNed.V.Y);
    velEstNedFilt.V.Z = biquadFilterApply(&posVelFilter[2], velEstNed.V.Z);
    fp_vector_t vI = velEstNedFilt; // EKF vel is used for now, will update it once airspeed is available

    getAttitudeThrustJacobian(JRows, PHIRows, &axB, &q_0, &filteredSpfBody, &vI);
    getYawConstraintRowZXY(c, &q_0);

    float B[MAXV * MAXU] = { 0.f };
    float desired[MAXV] = { LHS[0], LHS[1], LHS[2], 0.f };
    for (int i = 0; i < 3; i++) {
        desired[i] = constrainf(desired[i], -10.f, +10.f);
    }
    float Wv[MAXV] = { 0.f };
    float Wu[MAXU] = { 0.f };
    for (int i = 0; i < 4; i++) {
        Wv[i] = 1.f;
    }
    for (int i = 0; i < N_INDI_VARIABLES; i++) {
        Wu[i] = 1.f;
        for (int row = 0; row < 3; row++) {
            B[row + i * N_INDI_OUTPUTS] = J[row][i];
        }
        B[3 + i * N_INDI_OUTPUTS] = i < 3 ? c[i] : 0.f;
    }
    Wu[3] = 0.01f;

    float gamma_used;
    float A_as[(MAXU + MAXV) * MAXU];
    float b_as[MAXU + MAXV];
    float delta[MAXU] = { 0.f };
    float delta_min[MAXU] = { 0.f };
    float delta_max[MAXU] = { 0.f };
    for (int i = 0; i < 3; i++) {
        delta_min[i] = -DEGREES_TO_RADIANS(180.f);
        delta_max[i] = DEGREES_TO_RADIANS(180.f);
    }
    const float currentThrust = MAX(0.f, -fB_0.V.Z);
    delta_min[3] = 5.f - currentThrust;
    delta_max[3] = 10.f;

    setupWLS_A(B, Wv, Wu, N_INDI_OUTPUTS, N_INDI_VARIABLES,
        indiRun.wlsTheta, indiRun.wlsCondBound, A_as, &gamma_used);
    setupWLS_b(desired, delta, Wv, Wu, N_INDI_OUTPUTS, N_INDI_VARIABLES,
        gamma_used, b_as);

    static int8_t workingSet[MAXU] = { 0 };
    static activeSetExitCode solverResult = AS_SUCCESS;
#ifdef AS_RECORD_COST
    static float allocationCosts[AS_RECORD_COST_N] = { 0.f };
#else
    static float allocationCosts[1] = { 0.f };
#endif
    int iterations;
    int freeVariables;
    solverResult = solveActiveSet(indiRun.wlsAlgo)(
        A_as, b_as, delta_min, delta_max, delta, workingSet,
        10, N_INDI_VARIABLES, N_INDI_OUTPUTS,
        &iterations, &freeVariables, allocationCosts);

    if (solverResult < AS_NAN_FOUND_Q) {
        fp_vector_t deltaRotation = { .V = { .X = delta[0], .Y = delta[1], .Z = delta[2] } };
        const float rotationIncrement = VEC3_LENGTH(deltaRotation);
        fp_quaternion_t deltaAttitude = { .w = 1.f, .x = 0.f, .y = 0.f, .z = 0.f };
        if (rotationIncrement > 1e-6f) {
            VEC3_SCALAR_MULT(deltaRotation, 1.f / rotationIncrement);
            quaternion_of_axis_angle(&deltaAttitude, &deltaRotation, rotationIncrement);
        }
        attSpNedFromPos = chain_quaternion(&filteredAttitudeQuaternion, &deltaAttitude);
        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = filteredSpfBody.V.Z - delta[3];
    } else {
        attSpNedFromPos = q_0;
        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = fB_0.V.Z;
    }
}

void posGetAttSpNedAndSpfSpBody(timeUs_t current) {
    UNUSED(current);
    /*
     * We want 
     * 1. point the negative body z axis (thrust) towards accSpNed - Gravity
     * 2. point the positive body x axis (nose) as close to (cosYaw sinYaw 0)**T as possible, while respecting 1.
     */
    float Psi = getYawWithoutSingularity(); // current heading
    fp_quaternion_t attitude_q;
    getAttitudeQuaternion(&attitude_q); // current attitude
    // current body axes in inertial
    fp_vector_t currentX = { .A = { rMat.m[0][0], rMat.m[1][0], rMat.m[2][0] } };
    fp_vector_t currentZ = { .A = { rMat.m[0][2], rMat.m[1][2], rMat.m[2][2] } };

    // convert acc setpoint to specific forces in NED.
    // TODO: could add drag term here
    fp_vector_t spfSpNed = accSpNedFromPos;
    spfSpNed.V.Z -= GRAVITYf;

    // thrust setpoint in body frame for a multicopter:
    float spfSpLength = VEC3_LENGTH(spfSpNed);
    spfSpBodyFromPos.V.X = 0.f;
    spfSpBodyFromPos.V.Y = 0.f;
    spfSpBodyFromPos.V.Z = -spfSpLength;

    if (spfSpLength < 1e-6) {
        // when falling is commanded (spfSpNed = 0), keep current attitude apart
        // from yawing towards the commanded headingSp
        if (posSpNed.trackPsi) {
            fp_quaternion_t yawNed = {
                .w = cos_approx( 0.5f * (posSpNed.psi - Psi) ),
                .x = 0.f,
                .y = 0.f,
                .z = sin_approx( 0.5f * (posSpNed.psi - Psi) ),
            };

            attSpNedFromPos = chain_quaternion(&attitude_q, &yawNed);
        } else {
            // not asked to track yaw, we're done, just copy current attitude
            // to setpoint
            attSpNedFromPos = attitude_q;
        }
        return;
    }

    // base case: we need to set up the unit directions x, y, z to generate our
    //            attitude setpoint

    // z is easy:  z = -spfSpNed / || spfSpNed ||
    fp_vector_t xSp,ySp,zSp;
    zSp = spfSpNed;
    VEC3_SCALAR_MULT(zSp, -1.f);
    VEC3_NORMALIZE(zSp);

    if (posRuntime.use_spf_attenuation) {
        // if we havent reached out attitude yet, we may need to reduce thrust
        // setpoint to avoid thrusting into the wrong direction.
        // This is done by trying to match the thrust along z-axis, but limiting
        // the total thrust to the total thrust commanded
        fp_quaternion_t qHover;
        getHoverAttitudeQuaternion(&qHover);
        fp_vector_t zB = quatRotMatCol(&qHover, 2);

        float ratio;
        if (fabsf(zB.V.Z) < 1e-6f) {
            ratio = (zSp.V.Z > 0.f) ? 1.f : -1.f;
        } else {
            ratio = zSp.V.Z / zB.V.Z;
        }
        ratio = constrainf(ratio, 0.f, 1.f);
        spfSpBodyFromPos.V.Z *= ratio;
    }

    if (!posSpNed.trackPsi) {
        // just use minimum-norm quaternion rotation that rotates current z axis
        // to the desired z axis.
        fp_quaternion_t attError; // in NED coordinates!
        quaternion_of_two_vectors(&attError, &currentZ, &zSp, &currentX);

        // exterinsic rotation, first attitude_q then attError.
        attSpNedFromPos = chain_quaternion(&attError, &attitude_q);
        return;
    }

    // x = (starboardSp x z) / || starboardSp x z ||
    // this is because it has to be in the starboardSp-plane and also perp to z
    fp_vector_t headingSp   = { .A = { cos_approx(posSpNed.psi), sin_approx(posSpNed.psi), 0} };
    fp_vector_t starboardSp = { .A = {-sin_approx(posSpNed.psi), cos_approx(posSpNed.psi), 0} };

    VEC3_CROSS(xSp, starboardSp, zSp);
    if (VEC3_LENGTH(xSp) < 1e-6f) {
        // thrust is perp to the heading, so our nose should point towards
        // the heading
        xSp = headingSp;
    } else {
        VEC3_NORMALIZE(xSp);
    }
    VEC3_CROSS(ySp, zSp, xSp);

    // convert to rotation matrix
    fp_rotationMatrix_t rotM;
    for (int row = 0; row < 3; row++) {
        rotM.m[row][0] = xSp.A[row];
        rotM.m[row][1] = ySp.A[row];
        rotM.m[row][2] = zSp.A[row];
    }
    quaternion_of_rotationMatrix( &attSpNedFromPos, &rotM );
}

bool isWeathervane = false;

// TODO
// 1. velocity control..
// 2. weathervaning

#endif // USE_LOCAL_POSITION
