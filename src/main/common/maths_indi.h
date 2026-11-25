/*
 * This file is part of Indiflight.
 *
 * Indiflight-owned math extensions (vectors, quaternions, euler angles,
 * small dense linear algebra) on top of upstream common/maths.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#pragma once

#include <stdint.h>
#include <float.h>
#include <math.h>

#include "common/maths.h"

#define M_LN2f      0.69314718055994530942f

#define GRAVITYf 9.80665f
#define REARTHf 6371000.f
#define RADIANS_TO_DECIDEGREES(angle) ((angle) * 57.2957796f * 10)
#define SSL_AIR_DENSITY		1.225f // kg/m^3
#define SSL_AIR_PRESSURE	101325.01576f // Pascal
#define SSL_AIR_TEMPERATURE	288.15f // K

#define C_TO_KELVIN(temp)                   (temp + 273.15f)

// Floating point 3 vector.
typedef struct fp_vector {
    float X,Y,Z;
} fp_vector_def;

typedef union u_fp_vector {
    float A[3];
    fp_vector_def V;
} fp_vector_t;

// INT16 Euler angles in decidegrees. ZYX (yaw-pitch-roll) instrinsic rotation order.
typedef struct i16_euler {
    int16_t roll, pitch, yaw;
} i16_euler_def;

typedef union {
    int16_t raw[3];
    i16_euler_def angles;
} i16_euler_t;

// Floating point Euler angles.
typedef struct fp_euler {
    float roll,pitch,yaw;
} fp_euler_def;

// always radians
typedef union {
    float raw[3];
    fp_euler_def angles;
} fp_euler_t;
// identical text to flight/imu.h so both headers can be included together
#define EULER_INITIALIZE  { { 0, 0, 0 } }

typedef struct fp_rotationMatrix {
    float m[3][3];              // matrix
} fp_rotationMatrix_t;
#define ROTATION_MATRIX_INITIALIZE  { .m = {{1.,0.,0.}, {0.,1.,0.}, {0.,0.,1.}} }

// no quaternion union, because of the different conventions
typedef struct fp_quaternion {
    float w,x,y,z;
} fp_quaternion_t;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct fp_quaternionProducts {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} fp_quaternionProducts_t;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

// rotation functions
void i16_euler_of_fp_euler(i16_euler_t *ei, const fp_euler_t *ef);
void fp_euler_of_i16_euler(fp_euler_t *ef, const i16_euler_t *ei);
void fp_euler_of_rotationMatrix(fp_euler_t *e, const fp_rotationMatrix_t *r);
void fp_euler_of_quaternionProducts(fp_euler_t *e, const fp_quaternionProducts_t *qp);
void fp_euler_of_quaternionProducts_ZXY(fp_euler_t *e, const fp_quaternionProducts_t *qp);

void rotationMatrix_of_fp_euler(fp_rotationMatrix_t *r, const fp_euler_t *e);
void rotationMatrix_of_quaternionProducts(fp_rotationMatrix_t *r, const fp_quaternionProducts_t *qP);
void rotate_vector_with_rotationMatrix(fp_vector_t *v, const fp_rotationMatrix_t *r);
fp_rotationMatrix_t chain_rotationMatrix(const fp_rotationMatrix_t *rA_I, const fp_rotationMatrix_t *rB_A);

void quaternion_of_fp_euler(fp_quaternion_t *q, const fp_euler_t *e);
void quaternion_of_rotationMatrix(fp_quaternion_t *q, const fp_rotationMatrix_t *r);
void quaternion_of_axis_angle(fp_quaternion_t *q, const fp_vector_t *ax, float angle);
void quaternion_of_two_vectors(fp_quaternion_t *q, const fp_vector_t *a, const fp_vector_t *b, const fp_vector_t *orth_a);
void quaternionProducts_of_quaternion(fp_quaternionProducts_t *qP, const fp_quaternion_t *q);
void rotate_vector_with_quaternion(fp_vector_t *v, const fp_quaternion_t *q);
fp_quaternion_t chain_quaternion(const fp_quaternion_t* qA_I, const fp_quaternion_t* qB_A);
fp_vector_t quatRotMatCol(const fp_quaternion_t* q, uint8_t axis);
void quaternion_integrate_body_rates(fp_quaternion_t* q, const fp_vector_t* w, float dt);

// vector operation primitives

// ugly... convert to inline functions?
#define VEC3_SCALAR_MULT_ADD(_orig, _sc, _add) { \
    _orig.V.X += _sc * _add.V.X; \
    _orig.V.Y += _sc * _add.V.Y; \
    _orig.V.Z += _sc * _add.V.Z; \
}

#define VEC3_ELEM_MULT(_orig, _gains) { \
    _orig.V.X *= _gains.V.X; \
    _orig.V.Y *= _gains.V.Y; \
    _orig.V.Z *= _gains.V.Z; \
}

#define VEC3_ELEM_MULT_ADD(_orig, _gains, _add) { \
    _orig.V.X += _gains.V.X * _add.V.X; \
    _orig.V.Y += _gains.V.Y * _add.V.Y; \
    _orig.V.Z += _gains.V.Z * _add.V.Z; \
}

#define VEC3_SCALAR_MULT(_orig, _sc) { \
    _orig.V.X *= _sc; \
    _orig.V.Y *= _sc; \
    _orig.V.Z *= _sc; \
}

#define VEC3_LENGTH(_orig) \
    sqrtf( \
        _orig.V.X*_orig.V.X \
        + _orig.V.Y*_orig.V.Y \
        + _orig.V.Z*_orig.V.Z \
    )

#define VEC3_NORMALIZE(_orig) { \
    float _sc = VEC3_LENGTH(_orig) > 1e-8f ? 1.f / VEC3_LENGTH(_orig) : 0.f; \
    VEC3_SCALAR_MULT(_orig, _sc); \
}

#define QUAT_SCALAR_MULT(_orig, _sc) { \
    _orig.w *= _sc; \
    _orig.x *= _sc; \
    _orig.y *= _sc; \
    _orig.z *= _sc; \
}

#define QUAT_LENGTH(_orig) \
    sqrtf( \
        _orig.w*_orig.w \
        + _orig.x*_orig.x \
        + _orig.y*_orig.y \
        + _orig.z*_orig.z \
    )

#define QUAT_NORMALIZE(_orig) { \
    float _sc = QUAT_LENGTH(_orig) > 1e-8f ? 1.f / QUAT_LENGTH(_orig) : 0.f; \
    QUAT_SCALAR_MULT(_orig, _sc); \
}

// think of using hypot
#define VEC3_XY_LENGTH(_orig) \
    sqrtf( \
        _orig.V.X*_orig.V.X \
        + _orig.V.Y*_orig.V.Y \
    )

#define VEC3_CONSTRAIN_XY_LENGTH(_vec, _max_length) { \
    float _vec_len = VEC3_XY_LENGTH(_vec); \
    _vec.V.X /= constrainf(_vec_len / _max_length, 1.f, +FLT_MAX); \
    _vec.V.Y /= constrainf(_vec_len / _max_length, 1.f, +FLT_MAX); \
}

#define VEC3_DOT(_a, _b) ( \
    _a.V.X*_b.V.X  +  _a.V.Y*_b.V.Y +  _a.V.Z*_b.V.Z \
)

#define VEC3_CROSS(_c, _a, _b) { \
    _c.V.X = _a.V.Y * _b.V.Z   -   _a.V.Z * _b.V.Y; \
    _c.V.Y = _a.V.Z * _b.V.X   -   _a.V.X * _b.V.Z; \
    _c.V.Z = _a.V.X * _b.V.Y   -   _a.V.Y * _b.V.X; \
}

// linear algebra operations. Column major. 
// this is ugly, convert to inline functions?
// calculate c_ = a_ s
#define SGEVS(m_, a_, s, c_) {\
    for (int row = 0; row < m_; row++)\
        c_[row] = a_[row] * s;\
}

// calculate c_ = a_**T b_
#define SGEVV(m_, a_, b_, c_) {\
    c_ = 0.f;\
    for (int row = 0; row < m_; row++)\
        c_ += (a_)[row] * (b_)[row];\
}

// calculate c_ = A_ b_
#define SGEMV(m_, n_, A_, b_, c_) {\
    for (int row = 0; row < m_; row++) { \
        c_[row] = A_[row] * b_[0]; \
        for (int col = 1; col < n_; col++) { \
            c_[row] += A_[row + col*m_] * b_[col]; \
        } \
    } \
}

// calculate c_**T = b_**T A_ . Likely faster than SGEMV
#define SGEMVt(m_, n_, A_, b_, c_) {\
    for (int col = 0; col < n_; col++) { \
        c_[col] = A_[col*m_] * b_[0]; \
        for (int row = 1; row < m_; row++) { \
            c_[col] += A_[row + col*m_] * b_[row]; \
        } \
    } \
}

// calculate C_ = gamma_ * (alpha_ * C_ +  A_ B_)
#define SGEMM(m_, n_, p_, A_, B_, C_, alpha_, gamma_) { \
    for (int col = 0; col < n_; col++) { \
        for (int row = 0; row < m_; row++) { \
            if (alpha_ == 0.f) C_[row+col*m_] = 0.f; \
            else if (alpha_ != 1.f) C_[row+col*m_] *= alpha_; \
            for (int cA = 0; cA < p_; cA++) { \
                C_[row + col*m_] += A_[row + cA*m_] * B_[cA + col*p_]; \
            } \
            if (gamma_ != 1.f) C_[row+col*m_] *= gamma_; \
        } \
    } \
}

// calculate C_ = gamma_ * (alpha_ * C_ +  A_**T B_)
// p is rows of A_ and B_
#define SGEMMt(m_, n_, p_, A_, B_, C_, alpha_, gamma_) { \
    for (int col = 0; col < n_; col++) { \
        for (int row = 0; row < m_; row++) { \
            if (alpha_ == 0.f) C_[row+col*m_] = 0.f; \
            else if (alpha_ != 1.f) C_[row+col*m_] *= alpha_; \
            for (int cA = 0; cA < p_; cA++) { \
                C_[row + col*m_] += A_[cA + row*p_] * B_[cA + col*p_]; \
            } \
            if (gamma_ != 1.f) C_[row+col*m_] *= gamma_; \
        } \
    } \
}

// columns major. upper factor
void chol(float *U, float *A, float *iDiag, int n);

// Column major upper factor. solve A x = UT U x = b
void chol_solve(float *U, float* iDiag, int n, float *b, float *x);

void arraySubInt16(int16_t *dest, int16_t *array1, int16_t *array2, int count);
void arraySubUint16(int16_t *dest, uint16_t *array1, uint16_t *array2, int count);

static inline unsigned int constrainu(unsigned int amt, unsigned int low, unsigned int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
