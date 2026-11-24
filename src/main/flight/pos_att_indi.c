/*
 * Position control: acceleration setpoint to attitude and thrust setpoints by
 * incremental inversion with a yaw constraint, for hybrids
 * (position_attitude_mapping = INDI). Untested.
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
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "common/maths.h"
#include "common/filter.h"
#include "fc/runtime_config.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"
#include "setupWLS.h"

#ifdef USE_LOCAL_POSITION

static biquadFilter_t indiVelFilter[3];
static biquadFilter_t attitudeQuaternionFilter[4];
static biquadFilter_t spfBodyFilter[4];
static fp_quaternion_t filteredAttitudeQuaternion;
static fp_vector_t filteredSpfBody;
static bool attitudeQuaternionFilterPrimed;

void posAttInit_INDI(void) {
    for (int i = 0; i < 3; i++) {
        biquadFilterInitLPF(&indiVelFilter[i], 5.0f, 1000); // was shared with the stick velocity filter
        biquadFilterInitLPF(&spfBodyFilter[i], 5.0f, 1000);
    }
    for (int i = 0; i < 4; i++) {
        biquadFilterInitLPF(&attitudeQuaternionFilter[i], 5.0f, 1000);
    }
    filteredAttitudeQuaternion = (fp_quaternion_t) { .w = 1.f };
    filteredSpfBody = (fp_vector_t) { 0 };
    attitudeQuaternionFilterPrimed = false;
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
    velEstNedFilt.V.X = biquadFilterApply(&indiVelFilter[0], velEstNed.V.X);
    velEstNedFilt.V.Y = biquadFilterApply(&indiVelFilter[1], velEstNed.V.Y);
    velEstNedFilt.V.Z = biquadFilterApply(&indiVelFilter[2], velEstNed.V.Z);
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

#endif // USE_LOCAL_POSITION
