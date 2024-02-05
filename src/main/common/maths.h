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

#pragma once

#include <stdint.h>
#include <float.h>
#include <math.h>

#ifndef sq
#define sq(x) ((x)*(x))
#endif
#define power3(x) ((x)*(x)*(x))
#define power5(x) ((x)*(x)*(x)*(x)*(x))

// Undefine this for use libc sinf/cosf. Keep this defined to use fast sin/cos approximations
#define FAST_MATH             // order 9 approximation
#define VERY_FAST_MATH        // order 7 approximation

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define M_EULERf    2.71828182845904523536f

#define GRAVITYf 9.80665f

#define RAD    (M_PIf / 180.0f)
#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle) / 10.0f * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)
#define RADIANS_TO_DEGREES(angle) ((angle) * 57.2957796f)
#define RADIANS_TO_DECIDEGREES(angle) ((angle) * 57.2957796f * 10)

#define CM_S_TO_KM_H(centimetersPerSecond) ((centimetersPerSecond) * 36 / 1000)
#define CM_S_TO_MPH(centimetersPerSecond) ((centimetersPerSecond) * 10000 / 5080 / 88)

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
#define SIGN(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  (_x > 0) - (_x < 0); })

#define Q12 (1 << 12)

#define HZ_TO_INTERVAL(x) (1.0f / (x))
#define HZ_TO_INTERVAL_US(x) (1000000 / (x))

typedef int32_t fix12_t;

typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

// Floating point 3 vector.
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union u_fp_vector {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// vector operation primitives

// probably code bloat... convert to static functions?
#define VEC3_SCALAR_MULT_ADD(_orig, _sc, _add) { \
    _orig.V.X += _sc * _add.V.X; \
    _orig.V.Y += _sc * _add.V.Y; \
    _orig.V.Z += _sc * _add.V.Z; \
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
    _orig.qi *= _sc; \
    _orig.qx *= _sc; \
    _orig.qy *= _sc; \
    _orig.qz *= _sc; \
}

#define QUAT_LENGTH(_orig) \
    sqrtf( \
        _orig.qi*_orig.qi \
        + _orig.qx*_orig.qx \
        + _orig.qy*_orig.qy \
        + _orig.qz*_orig.qz \
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

// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

typedef struct fp_rotationMatrix_s {
    float m[3][3];              // matrix
} fp_rotationMatrix_t;

typedef struct fp_quaternion {
    float qi;
    float qx;
    float qy;
    float qz;
} fp_quaternion_t;

// no quaternion union, because of the different conventions

/**
 * @brief euler rotation 'ZYX'. Taken from paparazzi!
 *
 * @param e Euler output
 * @param q Quat input
 */
void float_eulers_of_quat(fp_angles_t *e, fp_quaternion_t *q);


/**
 * @brief get quaternion from ax-ang rotation formalism
 * https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
 * 
 * @param q Quaternion output
 * @param ax axis to rotate about
 * @param angle to rotate about (keep within -2pi to +pi)
 * 
*/
void float_quat_of_axang(fp_quaternion_t *q, t_fp_vector *ax, float angle);


// https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
fp_quaternion_t quatMult(fp_quaternion_t* ql, fp_quaternion_t* qr);
t_fp_vector quatRotate(fp_quaternion_t* q, t_fp_vector* v);
t_fp_vector quatRotMatCol(fp_quaternion_t* q, uint8_t axis);

int gcd(int num, int denom);
int32_t applyDeadband(int32_t value, int32_t deadband);
float fapplyDeadband(float value, float deadband);

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float degreesToRadians(int16_t degrees);

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo);
float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo);

void buildRotationMatrix(fp_angles_t *delta, fp_rotationMatrix_t *rotation);
void applyMatrixRotation(float *v, fp_rotationMatrix_t *rotationMatrix);

int32_t quickMedianFilter3(int32_t * v);
int32_t quickMedianFilter5(int32_t * v);
int32_t quickMedianFilter7(int32_t * v);
int32_t quickMedianFilter9(int32_t * v);

float quickMedianFilter3f(float * v);
float quickMedianFilter5f(float * v);
float quickMedianFilter7f(float * v);
float quickMedianFilter9f(float * v);

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
#define tan_approx(x)       (sin_approx(x) / cos_approx(x))
float exp_approx(float val);
float log_approx(float val);
float pow_approx(float a, float b);
#else
#define sin_approx(x)       sinf(x)
#define cos_approx(x)       cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define tan_approx(x)       tanf(x)
#define exp_approx(x)       expf(x)
#define log_approx(x)       logf(x)
#define pow_approx(a, b)    powf(b, a)
#endif

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count);
void arraySubInt16(int16_t *dest, int16_t *array1, int16_t *array2, int count);
void arraySubUint16(int16_t *dest, uint16_t *array1, uint16_t *array2, int count);

int16_t qPercent(fix12_t q);
int16_t qMultiply(fix12_t q, int16_t input);
fix12_t qConstruct(int16_t num, int16_t den);

static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
