/*
 * This file is part of Indiflight.
 *
 * Indiflight-owned math extensions, see maths_indi.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include <stdint.h>
#include <math.h>

#include "platform.h"

// indiflight-only code: compiled out of stock builds
#if defined(USE_INDI) || defined(USE_EKF) || defined(USE_LOCAL_POSITION) || defined(USE_PITOT) || defined(USE_GEOFENCE)

#include "common/axis.h"
#include "common/maths_indi.h"

FAST_CODE void i16_euler_of_fp_euler(i16_euler_t *ei, const fp_euler_t *ef) {
    for (int axis=0; axis<3; axis++)
        ei->raw[axis] = lrintf(ef->raw[axis] * (1800.f / M_PIf));
}

FAST_CODE void fp_euler_of_i16_euler(fp_euler_t *ef, const i16_euler_t *ei) {
    for (int axis=0; axis<3; axis++)
        ef->raw[axis] = (M_PIf / 1800.f) * (ei->raw[axis]);
}

FAST_CODE void fp_euler_of_rotationMatrix(fp_euler_t *e, const fp_rotationMatrix_t *r) {
    // https://www.eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
    // to following is valid for cos(pitch) > 0, which is given for pitch on (-pi/2, +pi/2)
    if (fabsf(r->m[2][0]) < 0.9999f) {
        e->angles.pitch = -asin_approx(r->m[2][0]); // or +pi, but we want the interval [-pi/2, +pi/2]
        e->angles.roll = atan2_approx(r->m[2][1], r->m[2][2]);
        e->angles.yaw = atan2_approx(r->m[1][0], r->m[0][0]);
    } else {
        // handle gimbal lock
        e->angles.pitch = (r->m[2][0] < 0.f) ? (0.5f*M_PIf) : (-0.5f*M_PIf);
        e->angles.yaw = 0.f; // choice
        e->angles.roll = (r->m[2][0] < 0.f) ? (atan2_approx(r->m[0][1], r->m[0][2])) : (atan2_approx(-r->m[0][1], -r->m[0][2])); // +- yaw, if non0
    }
}

FAST_CODE void fp_euler_of_quaternionProducts(fp_euler_t *e, const fp_quaternionProducts_t *qp) {
    // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Conversion_formulae_between_formalisms
    // z-y'-x'' rotation order: intrisic rotations around yaw then pitch then roll
    e->angles.roll  = atan2_approx((2.0f * (qp->wx + qp->yz)), (1.0f - 2.0f * (qp->xx + qp->yy)));
    e->angles.pitch = asin_approx(2.0f * (qp->wy - qp->xz));
    e->angles.yaw   = atan2_approx((2.0f * (qp->wz + qp->xy)), (1.0f - 2.0f * (qp->yy + qp->zz)));
}

FAST_CODE void fp_euler_of_quaternionProducts_ZXY(fp_euler_t *e, const fp_quaternionProducts_t *qp) {
    // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Conversion_formulae_between_formalisms
    // z-x'-y'' rotation order: intrinsic rotations around yaw then roll then pitch
    e->angles.roll  = asin_approx(2.0f * (qp->wx + qp->yz));
    e->angles.pitch = atan2_approx((2.0f * (qp->wy - qp->xz)), (1.0f - 2.0f * (qp->xx + qp->yy)));
    e->angles.yaw   = atan2_approx((2.0f * (qp->wz - qp->xy)), (1.0f - 2.0f * (qp->xx + qp->zz)));
}

FAST_CODE void rotationMatrix_of_fp_euler(fp_rotationMatrix_t *r, const fp_euler_t *e) {
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cos_approx(e->angles.roll);
    sinx = sin_approx(e->angles.roll);
    cosy = cos_approx(e->angles.pitch);
    siny = sin_approx(e->angles.pitch);
    cosz = cos_approx(e->angles.yaw);
    sinz = sin_approx(e->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    // row-wise:
    r->m[0][0] = cosz * cosy;
    r->m[0][1] = -sinzcosx + (coszsinx * siny);
    r->m[0][2] = (sinzsinx) + (coszcosx * siny);

    r->m[1][0] = cosy * sinz;
    r->m[1][1] = coszcosx + (sinzsinx * siny);
    r->m[1][2] = -(coszsinx) + (sinzcosx * siny);

    r->m[2][0] = -siny;
    r->m[2][1] = sinx * cosy;
    r->m[2][2] = cosy * cosx;
}

FAST_CODE void rotationMatrix_of_quaternionProducts(fp_rotationMatrix_t *r, const fp_quaternionProducts_t *qP) {
    //https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_%E2%86%94_quaternion
    r->m[0][0] = 1.0f - 2.0f * qP->yy - 2.0f * qP->zz;
    r->m[0][1] = 2.0f * (qP->xy + -qP->wz);
    r->m[0][2] = 2.0f * (qP->xz - -qP->wy);

    r->m[1][0] = 2.0f * (qP->xy - -qP->wz);
    r->m[1][1] = 1.0f - 2.0f * qP->xx - 2.0f * qP->zz;
    r->m[1][2] = 2.0f * (qP->yz + -qP->wx);

    r->m[2][0] = 2.0f * (qP->xz + -qP->wy);
    r->m[2][1] = 2.0f * (qP->yz - -qP->wx);
    r->m[2][2] = 1.0f - 2.0f * qP->xx - 2.0f * qP->yy;
}

FAST_CODE void rotate_vector_with_rotationMatrix(fp_vector_t *v, const fp_rotationMatrix_t *r) {
    // rotate vector with matrix. If r describes rotation from an Inertial to
    // a local coordinates system then, then the output is v expressed in
    //  inertial coordinates.
    fp_vector_t vTmp = *v;
    for (int i=0; i<3; i++)
        v->A[i] =  r->m[i][X] * vTmp.V.X  +  r->m[i][Y] * vTmp.V.Y  +  r->m[i][Z] * vTmp.V.Z;
}

FAST_CODE fp_rotationMatrix_t chain_rotationMatrix(const fp_rotationMatrix_t *rA_I, const fp_rotationMatrix_t *rB_A) {
    // if rA_I is rotation of frame A wrt Inertial, and rB_A is rotation of 
    // frame B wrt frame A (i.e. intrinsic rotation), then the output will be
    // the equivalent rotation of frame B wrt Inertial.
    // for extrinsic rotations (rB_A describes a rotation in Inertial frame), just
    // reverse the order of the arguments

    // rB_I = rA_I * rB_A
    fp_rotationMatrix_t out;
    for (int col=0; col<3; col++) {
        for (int row=0; row<3; row++)
            out.m[row][col] = rA_I->m[row][0] * rB_A->m[0][col]
                + rA_I->m[row][1] * rB_A->m[1][col]
                + rA_I->m[row][2] * rB_A->m[2][col];
    }
    return out;
}


FAST_CODE void quaternion_of_fp_euler(fp_quaternion_t *q, const fp_euler_t *e) {
    float sx2, cx2, sy2, cy2, sz2, cz2;
    sx2 = sin_approx(0.5f * e->angles.roll);
    cx2 = cos_approx(0.5f * e->angles.roll);
    sy2 = sin_approx(0.5f * e->angles.pitch);
    cy2 = cos_approx(0.5f * e->angles.pitch);
    sz2 = sin_approx(0.5f * e->angles.yaw);
    cz2 = cos_approx(0.5f * e->angles.yaw);

    q->x = sx2*cy2*cz2 - cx2*sy2*sz2;
    q->y = cx2*sy2*cz2 + sx2*cy2*sz2;
    q->z = cx2*cy2*sz2 - sx2*sy2*cz2;
    q->w = cx2*cy2*cz2 + sx2*sy2*sz2;
}

FAST_CODE void quaternion_of_rotationMatrix(fp_quaternion_t *q, const fp_rotationMatrix_t *r) {
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    float trace = r->m[0][0] + r->m[1][1] + r->m[2][2];
    float s, si;
    if (trace > 1e-6f) {
        s = 0.5f * sqrtf( 1.0f + trace );
        si = 0.25f / s;
        q->w = s;
        q->x = si * ( r->m[2][1] - r->m[1][2] );
        q->y = si * ( r->m[0][2] - r->m[2][0] );
        q->z = si * ( r->m[1][0] - r->m[0][1] );
    } else {
        if ( r->m[0][0] > r->m[1][1] && r->m[0][0] > r->m[2][2] ) {
            s = 0.5f * sqrtf( 1.0f + r->m[0][0] - r->m[1][1] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[2][1] - r->m[1][2] );
            q->x = s;
            q->y = si * (r->m[0][1] + r->m[1][0] );
            q->z = si * (r->m[0][2] + r->m[2][0] );
        } else if (r->m[1][1] > r->m[2][2]) {
            s = 0.5f * sqrtf( 1.0f + r->m[1][1] - r->m[0][0] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[0][2] - r->m[2][0] );
            q->x = si * (r->m[0][1] + r->m[1][0] );
            q->y = s;
            q->z = si * (r->m[1][2] + r->m[2][1] );
        } else {
            s = 0.5f * sqrtf( 1.0f + r->m[2][2] - r->m[0][0] - r->m[1][1] );
            si = 0.25f / s;
            q->w = si * (r->m[1][0] - r->m[0][1] );
            q->x = si * (r->m[0][2] + r->m[2][0] );
            q->y = si * (r->m[1][2] + r->m[2][1] );
            q->z = s;
        }
    }
}

FAST_CODE void quaternion_of_axis_angle(fp_quaternion_t *q, const fp_vector_t *ax, float angle) {
    // require ax to be normalized
    float ang2 = angle * 0.5f;
    float cang2 = cos_approx(ang2);
    float sang2 = sin_approx(ang2);

    q->w = cang2;
    q->x = ax->V.X * sang2;
    q->y = ax->V.Y * sang2;
    q->z = ax->V.Z * sang2;
}

FAST_CODE void quaternion_of_two_vectors(fp_quaternion_t *q, const fp_vector_t *a, const fp_vector_t *b, const fp_vector_t *orth_a) {
    // returns minimum rotation quaternion to rotate a to b
    // a, b, must be UNIT-NORM. orth_a must be UNIT vector orthogonal to a

    // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    // Half-Way Quaternion Solution
    float k_cos_theta = VEC3_DOT((*a), (*b));
    if (k_cos_theta <= -0.9999f) {
        // 180 degree rotation. needs sone (any) vector orth to a
        q->w = 0.0f;
        q->x = orth_a->V.X;
        q->y = orth_a->V.Y;
        q->z = orth_a->V.Z;
    } else {
        // normal case, cross product for vector part, then normalize
        q->w = 1.0f + k_cos_theta;
        q->x = a->V.Y * b->V.Z  -  a->V.Z * b->V.Y;
        q->y = a->V.Z * b->V.X  -  a->V.X * b->V.Z;
        q->z = a->V.X * b->V.Y  -  a->V.Y * b->V.X;
        QUAT_NORMALIZE((*q));
    }
}

FAST_CODE void quaternionProducts_of_quaternion(fp_quaternionProducts_t *qP, const fp_quaternion_t *q) {
    qP->ww = q->w * q->w;
    qP->wx = q->w * q->x;
    qP->wy = q->w * q->y;
    qP->wz = q->w * q->z;
    qP->xx = q->x * q->x;
    qP->xy = q->x * q->y;
    qP->xz = q->x * q->z;
    qP->yy = q->y * q->y;
    qP->yz = q->y * q->z;
    qP->zz = q->z * q->z;
}

FAST_CODE void rotate_vector_with_quaternion(fp_vector_t *v, const fp_quaternion_t *q) {
    // crazy algorithm due to Fabian Giesen (A faster quaternion-vector multiplication)
    // https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    // v' = v  +  q[0] * cross(2*q[1:], v)  +  cross(q[1:], cross(2*q[1:], v))
    // v' = v  +  q[0] * 2*cross(q[1:], v)  +  cross(q[1:], 2*cross(q[1:], v))
    fp_vector_t t;
    t.V.X  =  2.0f * ( q->y * v->V.Z  -  q->z * v->V.Y );
    t.V.Y  =  2.0f * ( q->z * v->V.X  -  q->x * v->V.Z );
    t.V.Z  =  2.0f * ( q->x * v->V.Y  -  q->y * v->V.X );

    v->V.X  +=  ( q->y * t.V.Z  -  q->z * t.V.Y )  +  ( q->w * t.V.X );
    v->V.Y  +=  ( q->z * t.V.X  -  q->x * t.V.Z )  +  ( q->w * t.V.Y );
    v->V.Z  +=  ( q->x * t.V.Y  -  q->y * t.V.X )  +  ( q->w * t.V.Z );
}

FAST_CODE fp_quaternion_t chain_quaternion(const fp_quaternion_t* qA_I, const fp_quaternion_t* qB_A) {
    // if qA_I is rotation of frame A wrt Inertial, and qB_A is rotation of 
    // frame B wrt frame A (i.e. intrinsic rotation), then the output will be
    // the equivalent rotation of frame B wrt Inertial.
    // for extrinsic rotations (qB_A describes a rotation in Inertial frame), just
    // reverse the order of the arguments

    // out = qA_I * qB_A
    fp_quaternion_t out = {
        .w = qA_I->w * qB_A->w - qA_I->x * qB_A->x - qA_I->y * qB_A->y - qA_I->z * qB_A->z,
        .x = qA_I->x * qB_A->w + qA_I->w * qB_A->x + qA_I->y * qB_A->z - qA_I->z * qB_A->y,
        .y = qA_I->w * qB_A->y - qA_I->x * qB_A->z + qA_I->y * qB_A->w + qA_I->z * qB_A->x,
        .z = qA_I->w * qB_A->z + qA_I->x * qB_A->y - qA_I->y * qB_A->x + qA_I->z * qB_A->w
    };
    return out;
}

FAST_CODE fp_vector_t quatRotMatCol(const fp_quaternion_t* q, uint8_t axis) {
    // basically q * v * qinv, where v = (0 1 0 0) or (0 0 1 0) or (0 0 0 1)
    // https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    fp_vector_t res = {0};
    switch(axis) {
        case 0:
            // X: v = (0 1 0 0)
            res.V.X = 1 - 2*(q->y*q->y + q->z*q->z);
            res.V.Y = 2*q->x*q->y + 2*q->w*q->z;
            res.V.Z = 2*q->x*q->z - 2*q->w*q->y;
            break;
        case 1:
            // Y: v = (0 0 1 0)
            res.V.X = 2*q->x*q->y - 2*q->w*q->z;
            res.V.Y = 1 - 2*(q->x*q->x + q->z*q->z);
            res.V.Z = 2*q->y*q->z + 2*q->w*q->x;
            break;
        case 2:
            // Z: v = (0 0 0 1)
            res.V.X = 2*q->x*q->z + 2*q->w*q->y;
            res.V.Y = 2*q->y*q->z - 2*q->w*q->x;
            res.V.Z = 1 - 2*(q->x*q->x + q->y*q->y);
            break;
    }
    return res;
}

// fwd euler integrate body rates w over interval dt: q <-- q + 0.5dt * q * (0 w)
void quaternion_integrate_body_rates(fp_quaternion_t* q, const fp_vector_t* w, float dt)
{
    fp_quaternion_t buffer = *q;
    QUAT_SCALAR_MULT(buffer, (0.5f*dt));

    q->w += (-buffer.x * w->V.X - buffer.y * w->V.Y - buffer.z * w->V.Z);
    q->x += (+buffer.w * w->V.X + buffer.y * w->V.Z - buffer.z * w->V.Y);
    q->y += (+buffer.w * w->V.Y - buffer.x * w->V.Z + buffer.z * w->V.X);
    q->z += (+buffer.w * w->V.Z + buffer.x * w->V.Y - buffer.y * w->V.X);

    QUAT_NORMALIZE((*q));
}

// columns major. upper factor
#ifdef STM32H7
FAST_CODE
#endif
void chol(float *U, float *A, float *iDiag, int n)
{
    // rosetta code
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < (i+1); j++) {
            float s = 0;
            for (int k = 0; k < j; k++) {
                s += U[i * n + k] * U[j * n + k];
            }
            if (i == j) {
                U[i * n + j] = sqrt(A[i * n + i] - s);
                iDiag[j] = 1.f / U[i * n + j];
            } else {
                U[i * n + j] = iDiag[j] * (A[i * n + j] - s);
            }
        }
    }
}

// column major, upper factor
#ifdef STM32H7
FAST_CODE
#endif
void chol_solve(float *U, float* iDiag, int n, float *b, float *x) {
    // Antoine Drouin, 2007, modified
	int j,k;
	float t;

    for(j = 0 ; j < n ; j++) { // solve Uty=b
        t = b[j];
        for(k = j - 1 ; k >= 0 ; k--)
            t -= U[k + n*j] * x[k];
        x[j] = t*iDiag[j];
    }
    for(j = n - 1 ; j >= 0 ; j--) { // solve Ux=y
        t = x[j];
        for(k = j + 1 ; k < n ; k++)
            t -= U[j + n*k] * x[k];
        x[j] = t*iDiag[j];
    }
}

void arraySubInt16(int16_t *dest, int16_t *array1, int16_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

void arraySubUint16(int16_t *dest, uint16_t *array1, uint16_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

#endif
