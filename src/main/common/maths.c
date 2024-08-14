/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "axis.h"
#include "maths.h"

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
#if defined(VERY_FAST_MATH)

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
#endif
float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return sinf(x);                            // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}
#endif

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

int gcd(int num, int denom)
{
    if (denom == 0) {
        return num;
    }

    return gcd(denom, num % denom);
}

int32_t applyDeadband(const int32_t value, const int32_t deadband)
{
    if (abs(value) < deadband) {
        return 0;
    }

    return value >= 0 ? value - deadband : value + deadband;
}

float fapplyDeadband(const float value, const float deadband)
{
    if (fabsf(value) < deadband) {
        return 0;
    }

    return value >= 0 ? value - deadband : value + deadband;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

float degreesToRadians(int16_t degrees)
{
    return degrees * RAD;
}

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo)
{
    long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
    long int b = (long int) srcTo - (long int) srcFrom;
    return (a / b) + destFrom;
}

float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo)
{
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v)
{
    int32_t p[7];
    QMF_COPY(p, v, 7);

    QMF_SORT(p[0], p[5]); QMF_SORT(p[0], p[3]); QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]); QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]); QMF_SORT(p[2], p[3]); QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]); QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v)
{
    int32_t p[9];
    QMF_COPY(p, v, 9);

    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]); QMF_SORT(p[5], p[8]); QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]); QMF_SORT(p[1], p[4]); QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]); QMF_SORT(p[4], p[2]); QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

float quickMedianFilter3f(float * v)
{
    float p[3];
    QMF_COPY(p, v, 3);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[0], p[1]) ;
    return p[1];
}

float quickMedianFilter5f(float * v)
{
    float p[5];
    QMF_COPY(p, v, 5);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[2], p[3]);
    QMF_SORTF(p[1], p[2]);
    return p[2];
}

float quickMedianFilter7f(float * v)
{
    float p[7];
    QMF_COPY(p, v, 7);

    QMF_SORTF(p[0], p[5]); QMF_SORTF(p[0], p[3]); QMF_SORTF(p[1], p[6]);
    QMF_SORTF(p[2], p[4]); QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[5]);
    QMF_SORTF(p[2], p[6]); QMF_SORTF(p[2], p[3]); QMF_SORTF(p[3], p[6]);
    QMF_SORTF(p[4], p[5]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[3]);
    QMF_SORTF(p[3], p[4]);
    return p[3];
}

float quickMedianFilter9f(float * v)
{
    float p[9];
    QMF_COPY(p, v, 9);

    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[6], p[7]);
    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[3]); QMF_SORTF(p[5], p[8]); QMF_SORTF(p[4], p[7]);
    QMF_SORTF(p[3], p[6]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[2], p[5]);
    QMF_SORTF(p[4], p[7]); QMF_SORTF(p[4], p[2]); QMF_SORTF(p[6], p[4]);
    QMF_SORTF(p[4], p[2]);
    return p[4];
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
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

int16_t qPercent(fix12_t q)
{
    return (100 * q) >> 12;
}

int16_t qMultiply(fix12_t q, int16_t input)
{
    return (input *  q) >> 12;
}

fix12_t  qConstruct(int16_t num, int16_t den)
{
    return (num << 12) / den;
}
