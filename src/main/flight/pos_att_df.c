/*
 * Position control: acceleration setpoint to attitude and thrust setpoints by
 * differential flatness with an aerodynamic model, Tal (2022), for
 * tailsitters (position_attitude_mapping = DF). The craft model (PHI, K, n)
 * in posGetAttSpNedAndSpfSpBody_DF is hardcoded for the DarkO tailsitter.
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

#ifdef USE_LOCAL_POSITION

static biquadFilter_t dfAccelerationFilter[3];
static biquadFilter_t dfModelSpfFilter[3];
// static biquadFilter_t rateSpBodyFilter[3];

void posAttInit_DF(void) {
    for (int i = 0; i < 3; i++) {
        // biquadFilterInit(&dfAccelerationFilter[i], sqrtf(0.5f * 5.0f), 500,
        //     sqrtf(0.5f * 5.0f) / (5.0f - 0.5f), FILTER_BPF, 1.0f);
        biquadFilterInitLPF(&dfAccelerationFilter[i], 3.0f, 1000);
        biquadFilterInitLPF(&dfModelSpfFilter[i], 3.0f, 1000);
//         biquadFilterInitLPF(&rateSpBodyFilter[i], 1.5f, 1000);
    }
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
    *T_r  =   V * PHI[2][2] * v_phi.V.X   * sTheta_r;
    *T_r +=   V * PHI[2][2] * v_phi.V.Z   * cTheta_r;
    *T_r +=                   f_phi_r.V.X * sTheta_r;
    *T_r +=                   f_phi_r.V.Z * cTheta_r;
    *T_r /= AX->V.Z;

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
        { 0.31f/0.7f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.000f },
        { 0.000f, 0.000f, 0.0465f/0.7f },
    };
    const float *PHIRows[3] = { PHI[0], PHI[1], PHI[2] };
    const fp_vector_t AX = { .V.X = 0.f, .V.Y = 0.f, .V.Z = -1.f };
    const float K[2] = { 1.413e-6f/0.7f, 1.413e-6f/0.7f }; // propeller constant divided by mass
    const float* omega = indiRun.omega; // just use indiRun.omega
    const int n = 2;
    // const float K[4] = { 1.88e-7f/0.41f, 1.88e-7f/0.41f, 1.88e-7f/0.41f, 1.88e-7f/0.41f }; // propeller constant divided by mass
    // const float* omega = indiRun.omega; // just use indiRun.omega
    // const int n = 4;

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
    fp_euler_t e_current;
    fp_quaternionProducts_t qp_current;
    quaternionProducts_of_quaternion(&qp_current, &q);
    fp_euler_of_quaternionProducts_ZXY(&e_current, &qp_current);

    // float Psi_r = posSpNed.trackPsi ? posSpNed.psi : getYawWithoutSingularity();
    float Psi_r = posSpNed.trackPsi ? posSpNed.psi : e_current.angles.yaw;

    fp_vector_t a_I_bpf;
    fp_vector_t f_I_model;
    fp_vector_t f_I_lpf;
    getSpfInertialPhi(&f_I_model, PHIRows, &AX, K, omega, n, &q, &v_I);
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
        // sanity check, run getSpfInertialPhi again on inversion
        // fp_vector_t f_I_model_r;
        // getSpfInertialPhi(&f_I_model_r, PHIRows, &AX, K, omega, n, &q_r, &v_I);
        // UNUSED(f_I_model_r);

        attSpNedFromPos = q_r;
        fp_quaternion_t qi_r = q_r;
        qi_r.w *= -1.f;
        fp_vector_t a_B_r = accSpNedFromPos;
        a_B_r.V.Z -= GRAVITYf;
        rotate_vector_with_quaternion(&a_B_r, &qi_r);

        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = a_B_r.V.Z;
        // spfSpBodyFromPos.V.Z = -T_r;

        // we can calculate body rates to maintain V_B as good as possible using 
        //   omega_B = R_B^I  (v_I x A_I) / (v_I^T v_I)
        fp_quaternion_t qinv = q;
        qinv.w *= -1.f;
        float vel2 = sq(v_I.V.X) + sq(v_I.V.Y) + sq(v_I.V.Z);
        if (vel2 > sq(8.f)) {
            fp_vector_t filteredRateSpBody;
            VEC3_CROSS(filteredRateSpBody, v_I, accSpNedFromPos)
            VEC3_SCALAR_MULT(filteredRateSpBody, (1.f / vel2));
            // for (int axis = 0; axis < 3; axis++) {
            //     filteredRateSpBody.A[axis] = biquadFilterApply(&rateSpBodyFilter[axis], filteredRateSpBody.A[axis]);
            // }
            rateSpBodyFromPos = filteredRateSpBody;
            rotate_vector_with_quaternion(&rateSpBodyFromPos, &qinv);
        }
    } else {
        attSpNedFromPos = q;
        spfSpBodyFromPos.V.X = 0.f;
        spfSpBodyFromPos.V.Y = 0.f;
        spfSpBodyFromPos.V.Z = AX.V.Z * (K[0] * sq(omega[0]) + K[1] * sq(omega[1]));
    }
}

#endif // USE_LOCAL_POSITION
