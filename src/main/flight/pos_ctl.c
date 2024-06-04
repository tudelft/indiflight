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
#include "flight/imu.h"
#include "common/maths.h"
#include "fc/runtime_config.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"
#include "flight/trajectory_tracker.h"

#ifdef USE_LOCAL_POSITION

#ifndef USE_INDI
#pragma message "USE_LOCAL_POSITION currently only has any effect with USE_INDI"
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
        p->horz_max_iterm = 200;
        p->max_tilt = 30;
        p->vert_p = 30;
        p->vert_i = 2;
        p->vert_d = 30;
        p->vert_max_v_up = 100;
        p->vert_max_v_down = 100;
        p->vert_max_a_up = 500;
        p->vert_max_a_down = 500;
        p->vert_max_iterm = 100;
        p->yaw_p = 30;
        p->weathervane_p = 0;
        p->weathervane_min_v = 200;
        p->use_spf_attenuation = 1;
    }
}

void posCtlInit(void) {
    initPositionRuntime();
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
    posRuntime.vert_d = p->vert_d * 0.1f;
    posRuntime.vert_max_v_up = p->vert_max_v_up * 0.01f;
    posRuntime.vert_max_v_down = p->vert_max_v_down * 0.01f;
    posRuntime.vert_max_a_up = p->vert_max_a_up * 0.01f;
    posRuntime.vert_max_a_down = p->vert_max_a_down * 0.01f;
    posRuntime.vert_max_iterm = p->vert_max_iterm * 0.01f;
    posRuntime.yaw_p = p->yaw_p * 0.1f;
    posRuntime.weathervane_p = p->weathervane_p * 0.1f;
    posRuntime.weathervane_min_v = p->weathervane_min_v * 0.01f;
    posRuntime.use_spf_attenuation = (bool) p->use_spf_attenuation;
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
fp_vector_t velIError = {0};
void resetIterms(void) {
    velIError.V.X = 0.f;
    velIError.V.Y = 0.f;
    velIError.V.Z = 0.f;
}

void updatePosCtl(timeUs_t current) {
    timeDelta_t timeInDeadreckoning = cmpTimeUs(current, extLatestMsgTime);

    if ((posMeasState == LOCAL_POS_NO_SIGNAL) 
        || (timeInDeadreckoning > DEADRECKONING_TIMEOUT_DESCEND_SLOWLY_US)) {
        // panic and level craft in slight downwards motion
        accSpNedFromPos.V.X = 0.f;
        accSpNedFromPos.V.Y = 0.f;
        accSpNedFromPos.V.Z = 2.f; // slight downwards motion
        rateSpBodyFromPos.V.X = 0.f;
        rateSpBodyFromPos.V.Y = 0.f;
        rateSpBodyFromPos.V.Z = 0.f;
        posSpNed.trackPsi = false;

        // latch reactivation until new actual setpoint arrives
        posSpState = LOCAL_POS_NO_SIGNAL;
    } else if (timeInDeadreckoning > DEADRECKONING_TIMEOUT_HOLD_POSITION_US) {
        // more than 0.5 sec but less than 2 seconds --> arrest motion
#ifdef USE_TRAJECTORY_TRACKER
        updateTrajectoryTracker(current);
        if (isActiveTrajectoryTracker() && !isActiveTrajectoryTrackerRecovery()) {
            stopTrajectoryTracker();
        }
        if (!isActiveTrajectoryTrackerRecovery())
#endif
        {
            posSpNed.pos = posEstNed; // hold position
            posSpState = LOCAL_POS_NEW_MESSAGE;

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
        if (!isActiveTrajectoryTracker())
#endif
        {
            posGetAccSpNed(current);
            rateSpBodyFromPos.V.X = 0; // TODO: implement weathervaning?
            rateSpBodyFromPos.V.Y = 0;
            rateSpBodyFromPos.V.Z = 0;
        }
    }

    // always use NDI function to map acc setpoints
    posGetAttSpNedAndSpfSpBody(current);
}

void posGetAccSpNed(timeUs_t current) {
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

    // vel error = vel setpoint - vel estimate
    fp_vector_t velError = posSpNed.vel;
    //VEC3_SCALAR_MULT_ADD(velError, -1.0f, posMeasNed.vel);
    VEC3_SCALAR_MULT_ADD(velError, -1.0f, velEstNed);

    static bool accSpXYSaturated = true;
    static bool accSpZSaturated = true;
    static timeUs_t lastCall = 0;
    timeDelta_t delta = cmpTimeUs(current, lastCall);
    if ((lastCall > 0) && (delta > 0) && (delta < 50000)) {
        if (!accSpXYSaturated) {
            velIError.V.X += delta * 1e-6f * velError.V.X;
            velIError.V.Y += delta * 1e-6f * velError.V.Y;
        }

        if (!accSpZSaturated)
            velIError.V.Z += delta * 1e-6f * velError.V.Z;

        VEC3_CONSTRAIN_XY_LENGTH(velIError, posRuntime.horz_max_iterm);
        velIError.V.Z = constrainf(velIError.V.Z, -posRuntime.vert_max_iterm, posRuntime.vert_max_iterm);
    }
    lastCall = current;

    // acceleration setpoint = velGains * velError
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
    spfSpBodyFromPos.V.X = 0.;
    spfSpBodyFromPos.V.Y = 0.;
    spfSpBodyFromPos.V.Z = -spfSpLength;

    if (spfSpLength < 1e-6) {
        // when falling is commanded (spfSpNed = 0), keep current attitude apart
        // from yawing towards the commanded headingSp
        if (posSpNed.trackPsi) {
            fp_quaternion_t yawNed = {
                .w = cos_approx( (posSpNed.psi - Psi) / 2.f ),
                .x = 0.f,
                .y = 0.f,
                .z = sin_approx( (posSpNed.psi - Psi) / 2.f ),
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
    fp_vector_t x,y,z;
    z = spfSpNed;
    VEC3_SCALAR_MULT(z, -1.f);
    VEC3_NORMALIZE(z);

    if (posRuntime.use_spf_attenuation) {
        // discount thrust if we have not yet reached our attitude
        fp_vector_t zDesNed = quatRotMatCol(&attSpNedFromPos, 2);
        fp_quaternion_t qHover;
        getHoverAttitudeQuaternion(&qHover);
        fp_vector_t zHoverFrame = quatRotMatCol(&qHover, 2);
        float zDotProd;
        zDotProd = VEC3_DOT(zDesNed, zHoverFrame);

        spfSpBodyFromPos.V.Z *= constrainf(VEC3_DOT(z, currentZ), 0.f, 1.f);
    }

    if (!posSpNed.trackPsi) {
        // just use minimum-norm quaternion rotation that rotates current z axis
        // to the desired z axis.
        fp_quaternion_t attError; // in NED coordinates!
        quaternion_of_two_vectors(&attError, &currentZ, &z, &currentX);

        // exterinsic rotation, first attitude_q then attError.
        attSpNedFromPos = chain_quaternion(&attError, &attitude_q);
        return;
    }

    // x = (starboardSp x z) / || starboardSp x z ||
    // this is because it has to be in the starboardSp-plane and also perp to z
    fp_vector_t headingSp   = { .A = { cos_approx(posSpNed.psi), sin_approx(posSpNed.psi), 0} };
    fp_vector_t starboardSp = { .A = {-sin_approx(posSpNed.psi), cos_approx(posSpNed.psi), 0} };

    VEC3_CROSS(x, starboardSp, z);
    if (VEC3_LENGTH(x) < 1e-6) {
        // thrust is perp to the heading, so our nose should point towards
        // the heading
        x = headingSp;
    } else {
        VEC3_NORMALIZE(x);
    }
    VEC3_CROSS(y, z, x);

    // convert to rotation matrix
    fp_rotationMatrix_t rotM;
    for (int row = 0; row < 3; row++) {
        rotM.m[row][0] = x.A[row];
        rotM.m[row][1] = y.A[row];
        rotM.m[row][2] = z.A[row];
    }
    quaternion_of_rotationMatrix( &attSpNedFromPos, &rotM );
}

bool isWeathervane = false;

// TODO
// 1. velocity control..
// 2. weathervaning

#endif // USE_LOCAL_POSITION
