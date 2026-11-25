/*
 * Position control: acceleration setpoint to attitude and thrust setpoints by
 * nonlinear dynamic inversion for multirotors (position_attitude_mapping = NDI).
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


#include "flight/pos_ctl.h"
#include "flight/pos_att.h"

#include "io/local_pos.h"
#include "flight/ahrs_compat.h"
#include "flight/ekf.h"
#include "common/maths_indi.h"
#include "common/filter_indi.h"
#include "fc/runtime_config.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"

#ifdef USE_LOCAL_POSITION

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

    if (spfSpLength < 1e-6f) {
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

#endif // USE_LOCAL_POSITION
