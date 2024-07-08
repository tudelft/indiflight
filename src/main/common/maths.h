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
#undef M_LN2f
#define M_LN2f      0.693147180559945309417f
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
#define EULER_INITIALIZE  { .raw = {0, 0, 0} }

typedef struct fp_rotationMatrix {
    float m[3][3];              // matrix
} fp_rotationMatrix_t;
#define ROTATION_MATRIX_INITIALIZE  { .m = {{1.,0.,0.}, {0.,1.,0.}, {0.,0.,1.}} }

// no quaternion union, because of the different conventions
typedef struct fp_quaternion {
    float w,x,y,z;
} fp_quaternion_t;
#define QUATERNION_INITIALIZE  { .w=1.f, .x=0.f, .y=0., .z=0. }

typedef struct fp_quaternionProducts {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} fp_quaternionProducts_t;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

// rotation functions
void i16_euler_of_fp_euler(i16_euler_t *ei, const fp_euler_t *ef);
void fp_euler_of_i16_euler(fp_euler_t *ef, const i16_euler_t *ei);
void fp_euler_of_rotationMatrix(fp_euler_t *e, const fp_rotationMatrix_t *r);
void fp_euler_of_quaternionProducts(fp_euler_t *e, const fp_quaternionProducts_t *qp);

void rotationMatrix_of_fp_euler(fp_rotationMatrix_t *r, const fp_euler_t *e);
void rotationMatrix_of_quaternionProducts(fp_rotationMatrix_t *r, const fp_quaternionProducts_t *qP);
void rotate_vector_with_rotationMatrix(fp_vector_t *v, const fp_rotationMatrix_t *r);
fp_rotationMatrix_t chain_rotationMatrix(const fp_rotationMatrix_t *rA_I, const fp_rotationMatrix_t *rB_A);

void quaternion_of_fp_euler(fp_quaternion_t *q, const fp_euler_t *e);
void quaternion_of_rotationMatrix(fp_quaternion_t *q, const fp_rotationMatrix_t *r);
void quaternion_of_axis_angle(fp_quaternion_t *q, const fp_vector_t *ax, float angle);
void quaternionProducts_of_quaternion(fp_quaternionProducts_t *qP, const fp_quaternion_t *q);
void rotate_vector_with_quaternion(fp_vector_t *v, const fp_quaternion_t *q);
fp_quaternion_t chain_quaternion(const fp_quaternion_t* qA_I, const fp_quaternion_t* qB_A);
fp_vector_t quatRotMatCol(const fp_quaternion_t* q, uint8_t axis);

// vector operation primitives

// ugly... convert to inline functions?
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
#define asin_approx(x)      (0.5f*M_PIf - acos_approx(x))
#define tan_approx(x)       (sin_approx(x) / cos_approx(x))
float exp_approx(float val);
float log_approx(float val);
float pow_approx(float a, float b);
#else
#define sin_approx(x)       sinf(x)
#define cos_approx(x)       cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define asin_approx(x)      asinf(x)
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

static inline unsigned int constrainu(unsigned int amt, unsigned int low, unsigned int high)
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
