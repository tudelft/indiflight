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
#include "flight/ahrs_compat.h"
#include "flight/ekf.h"
#include "common/maths_indi.h"
#include "common/filter_indi.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/indi_glue.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"
#ifdef USE_TRAJECTORY_TRACKER
#include "flight/trajectory_tracker.h"
#endif
#ifdef USE_GEOFENCE
#include "flight/geofence.h"
#endif
#include "flight/pos_att.h"
#include "io/gps.h"
#include "pg/autopilot.h"
#include "pg/flight_plan.h"

#ifdef USE_LOCAL_POSITION

#ifndef USE_INDI
#error "USE_LOCAL_POSITION requires the use of USE_INDI"
#endif

#ifndef USE_EKF
#error "USE_LOCAL_POSITION require the use of USE_EKF"
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(positionProfile_t, POSITION_PROFILE_COUNT, positionProfiles, PG_POSITION_PROFILE, 3);

// active position profile. Not "positionConfig": upstream owns that name (PG_POSITION).
PG_REGISTER_WITH_RESET_TEMPLATE(posCtlConfig_t, posCtlConfig, PG_POS_CTL_CONFIG, 0);
PG_RESET_TEMPLATE(posCtlConfig_t, posCtlConfig,
    .profileIndex = 0,
);

uint8_t getCurrentPositionProfileIndex(void)
{
    return posCtlConfig()->profileIndex;
}

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
        p->stick_takeover = 1;
#if defined(INDI_IS_TAILSITTER) || defined(LEARNER_IS_TAILSITTER)
        p->attitude_mapping = POSITION_ATTITUDE_MAPPING_DF;
#else
        p->attitude_mapping = POSITION_ATTITUDE_MAPPING_NDI;
#endif
        p->waypoint_source = POSITION_WAYPOINT_SOURCE_LOCAL;
    }
}

// local waypoints, one list per position profile; empty by default
PG_REGISTER_ARRAY(positionWaypointList_t, POSITION_PROFILE_COUNT, positionWaypointLists, PG_POSITION_WAYPOINTS, 0);

void posCtlInit(void) {
    initPositionRuntime();
    posRuntime.arrest_motion = false;
    posRuntime.arrest_z_motion_only = false;
}

// filtered stick-based velocity (NED)
static svfLowpassFilter_t posVelFilter[3];

static unsigned int activeWaypointIndex;
static positionWaypoint_t activeWaypoint; // decoded when it becomes active
static bool activeWaypointHolding;        // reached, hold time running
static timeUs_t activeWaypointReachedUs;
static bool waypointRunActive;  // waypoints drive the position setpoint in this loop
static bool waypointResumable;  // false: next engagement of WAYPOINT_MODE starts at the first waypoint

// Global waypoints are only meaningful while the EKF flies on GPS, because the
// local NED frame is then anchored at GPS home. Indoors (mocap/uplink) the
// local list is always used, whatever position_waypoint_source says.
static position_waypoint_source_t getWaypointSource(void)
{
#if defined(USE_GPS) && ENABLE_FLIGHT_PLAN
    if ((positionProfiles(getCurrentPositionProfileIndex())->waypoint_source == POSITION_WAYPOINT_SOURCE_GLOBAL)
            && (ekfConfig()->meas_source == LOCAL_POS_SOURCE_GPS)) {
        return POSITION_WAYPOINT_SOURCE_GLOBAL;
    }
#endif
    return POSITION_WAYPOINT_SOURCE_LOCAL;
}

static unsigned int getWaypointCount(position_waypoint_source_t source)
{
#if defined(USE_GPS) && ENABLE_FLIGHT_PLAN
    if (source == POSITION_WAYPOINT_SOURCE_GLOBAL) {
        return MIN(flightPlanConfig()->waypointCount, MAX_WAYPOINTS);
    }
#else
    UNUSED(source);
#endif
    return MIN(positionWaypointLists(getCurrentPositionProfileIndex())->count, POSITION_WAYPOINT_MAX);
}

// Decode waypoint index into local NED (m). Returns false if it can't be flown;
// the caller then leaves WAYPOINT_MODE and holds position.
static bool decodeWaypoint(position_waypoint_source_t source, unsigned int index, positionWaypoint_t *out)
{
#if defined(USE_GPS) && ENABLE_FLIGHT_PLAN
    if (source == POSITION_WAYPOINT_SOURCE_GLOBAL) {
        // upstream flight plan: lat/lon 1e-7 deg, altitude cm AMSL. Relative to
        // GPS home, the origin of the EKF's GPS frame (localPosOnGpsNewData()).
        if (!STATE(GPS_FIX_HOME)) {
            return false;
        }
        const waypoint_t *wp = &flightPlanConfig()->waypoints[index];
        switch (wp->type) {
        case WAYPOINT_TYPE_FLYOVER:
        case WAYPOINT_TYPE_FLYBY:
        case WAYPOINT_TYPE_HOLD:
            break;
        default:
            return false; // LAND, TAKEOFF and the modifier types are not supported here
        }
        const gpsLocation_t llh = { .lat = wp->latitude, .lon = wp->longitude, .altCm = wp->altitude };
        llh_to_local(&llh, &GPS_home_llh, &out->location);
        out->tolerance = autopilotConfig()->waypointArrivalRadius * 0.01f;
        out->heightTolerance = out->tolerance;
        out->velocityLimit = wp->speed * 0.01f;
        out->heading = -1.f;
        out->holdTime = (wp->type == WAYPOINT_TYPE_HOLD) ? wp->duration * 0.1f : 0.f;
        return true;
    }
#else
    UNUSED(source);
#endif
    const positionWaypointNed_t *wp = &positionWaypointLists(getCurrentPositionProfileIndex())->waypoints[index];
    out->location.V.X = wp->north * 0.01f;
    out->location.V.Y = wp->east * 0.01f;
    out->location.V.Z = wp->down * 0.01f;
    out->tolerance = wp->tolerance * 0.01f;
    out->heightTolerance = wp->height_tolerance * 0.01f;
    out->velocityLimit = wp->speed * 0.01f;
    out->heading = (wp->heading >= 0) ? DEGREES_TO_RADIANS(wp->heading) : -1.f;
    out->holdTime = wp->duration * 0.1f;
    return true;
}

static bool activateWaypoint(position_waypoint_source_t source, unsigned int index)
{
    activeWaypointIndex = index;
    activeWaypointHolding = false;
    return decodeWaypoint(source, index, &activeWaypoint);
}

static bool isWaypointReached(const positionWaypoint_t *waypoint)
{
    const float northError = waypoint->location.V.X - posEstNed.V.X;
    const float eastError = waypoint->location.V.Y - posEstNed.V.Y;
    const float downError = waypoint->location.V.Z - posEstNed.V.Z;
    const float horizontalDistance = sqrtf(northError * northError + eastError * eastError);

    return (horizontalDistance <= waypoint->tolerance) && (fabsf(downError) <= waypoint->heightTolerance);
}

// reached, and the hold time (if any) has elapsed since first reaching it
static bool isActiveWaypointDone(timeUs_t current)
{
    if (!activeWaypointHolding) {
        if (!isWaypointReached(&activeWaypoint)) {
            return false;
        }
        activeWaypointHolding = true;
        activeWaypointReachedUs = current;
    }
    return cmpTimeUs(current, activeWaypointReachedUs) >= (timeDelta_t)(activeWaypoint.holdTime * 1e6f);
}

static void leaveWaypointMode(void)
{
    // hold the current setpoint in normal position mode; core.c only
    // re-enables WAYPOINT_MODE after the switch is cycled
    waypointRunActive = false;
    waypointResumable = false;
    DISABLE_INDIFLIGHT_MODE(WAYPOINT_MODE);
}

// WAYPOINT_MODE runs on top of POSITION_MODE (it is only enabled by core.c while
// POSITION_MODE is active). Waypoints come from the local list of the position
// profile or the upstream flight plan (getWaypointSource()), and each one is
// decoded into local NED when it becomes active. Edge handling:
// - rising edge: start at the first waypoint if the plan was never started, was
//   finished, or POSITION_MODE was left in between. Otherwise resume the last waypoint.
// - falling edge (switch off, POSITION_MODE still on): arrest motion and hold position.
// - plan finished, empty, or a waypoint can't be decoded: WAYPOINT_MODE is switched
//   off here and the last setpoint is held.
static void updateWaypointMode(timeUs_t current)
{
    static bool waypointModePrev = false;
    const bool positionMode = INDIFLIGHT_MODE(POSITION_MODE);
    const bool waypointMode = positionMode && INDIFLIGHT_MODE(WAYPOINT_MODE);
    const position_waypoint_source_t source = getWaypointSource();
    const unsigned int count = getWaypointCount(source);

    if (!positionMode) {
        waypointResumable = false;
    }

    bool decoded = true;
    if (waypointMode && !waypointModePrev) {
        const unsigned int startIndex = (waypointResumable && activeWaypointIndex < count) ? activeWaypointIndex : 0;
        decoded = (count > 0) && activateWaypoint(source, startIndex);
        waypointResumable = true;
    } else if (!waypointMode && waypointModePrev && positionMode && ARMING_FLAG(ARMED)) {
        posArrestMotion();
    }
    waypointModePrev = waypointMode;

    if (!decoded || (waypointMode && activeWaypointIndex >= count)) {
        // empty plan, undecodable waypoint, or the list shrank under us
        waypointModePrev = false; // not a falling edge, don't arrest motion
        leaveWaypointMode();
        return;
    }

    waypointRunActive = waypointMode && ARMING_FLAG(ARMED);
    if (!waypointRunActive) {
        return;
    }

    // advance through all waypoints that are already done
    while ((activeWaypointIndex + 1 < count) && isActiveWaypointDone(current)) {
        if (!activateWaypoint(source, activeWaypointIndex + 1)) {
            waypointModePrev = false;
            leaveWaypointMode();
            return;
        }
    }

    const float heading = activeWaypoint.heading;
    posSpNed.pos = activeWaypoint.location;
    posSpNed.valid = true;
    posSpNed.trackPsi = heading >= 0.f;
    if (posSpNed.trackPsi) {
        posSpNed.psi = heading;
    }

    if ((activeWaypointIndex + 1 == count) && isActiveWaypointDone(current)) {
        // plan finished: hold the final waypoint in normal position mode
        waypointModePrev = false; // not a falling edge, don't arrest motion
        leaveWaypointMode();
    }
}

static float activeWaypointVelocityLimit(void)
{
    if (waypointRunActive) {
        return activeWaypoint.velocityLimit;
    }

    return 0.f;
}

positionRuntime_t posRuntime;
void initPositionRuntime(void) {
    const positionProfile_t* p = positionProfiles(getCurrentPositionProfileIndex());
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
    posRuntime.stick_takeover = (bool) p->stick_takeover;

    posRuntime.attitude_mapping = (position_attitude_mapping_t) p->attitude_mapping;
    waypointResumable = false; // the waypoint list may differ per profile

    // init stick velocity filters (2nd order LPF)
    for (int i = 0; i < 3; i++) {
        svfLowpassFilterInit(&posVelFilter[i], 5.0f, 1e-3f); // TODO: make dynamic
    }

    posAttInit_INDI();
    posAttInit_DF();
}

void changePositionProfile(uint8_t profileIndex)
{
    if (profileIndex < POSITION_PROFILE_COUNT) {
        posCtlConfigMutable()->profileIndex = profileIndex;
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

    updateWaypointMode(current);

    if (manual_takeover && (posSpNed.valid || waypointRunActive)) {
        // no more manual control because new setpoint received
        // reset sticks, so that we can detect new stick movement later
        manual_takeover = false;
        setSticksReference();
    } else if (!manual_takeover && ARMING_FLAG(ARMED) && posRuntime.stick_takeover
            && !INDIFLIGHT_MODE(WAYPOINT_MODE) && haveSticksMoved()) {
        // stick takeover is never allowed in waypoint mode
        manual_takeover = true;
        posSpNed.valid = false;
#ifdef USE_TRAJECTORY_TRACKER
        if (isActiveTrajectoryTracker()) {
            stopTrajectoryTracker();
        }
#endif
    }

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
        latch_descend = ARMING_FLAG(ARMED) && INDIFLIGHT_MODE(POSITION_MODE);
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
        if ( (!ARMING_FLAG(ARMED)) || (!INDIFLIGHT_MODE(POSITION_MODE | GPS_RESCUE_MODE)) ) {
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

    // map acceleration setpoint to attitude and thrust setpoints (position_attitude_mapping)
    switch (posRuntime.attitude_mapping) {
        case POSITION_ATTITUDE_MAPPING_INDI:
            posGetAttSpNedAndSpfSpBody_INDI(current);
            break;
        case POSITION_ATTITUDE_MAPPING_DF:
            posGetAttSpNedAndSpfSpBody_DF(current);
            break;
        case POSITION_ATTITUDE_MAPPING_NDI:
        default:
            posGetAttSpNedAndSpfSpBody(current);
            break;
    }
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
    velEstNedFilt.V.X = svfLowpassFilterApply(&posVelFilter[0], velEstNed.V.X);
    velEstNedFilt.V.Y = svfLowpassFilterApply(&posVelFilter[1], velEstNed.V.Y);
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


bool isWeathervane = false;

// TODO
// 1. velocity control..
// 2. weathervaning

#endif // USE_LOCAL_POSITION
