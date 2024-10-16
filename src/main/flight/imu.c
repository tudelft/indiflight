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

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/external_pos.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#include "imu.h"

static pthread_mutex_t imuUpdateLock;

#if defined(SIMULATOR_IMU_SYNC)
static uint32_t imuDeltaT = 0;
static bool imuUpdated = false;
#endif

#define IMU_LOCK pthread_mutex_lock(&imuUpdateLock)
#define IMU_UNLOCK pthread_mutex_unlock(&imuUpdateLock)

#else

#define IMU_LOCK
#define IMU_UNLOCK

#endif

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// https://drive.google.com/file/d/0ByvTkVQo3tqXQUVCVUNyZEgtRGs/view?usp=sharing&resourcekey=0-Mo4254cxdWWx2Y4mGN78Zw

#define SPIN_RATE_LIMIT 20

#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 200        // 200cm/s minimum groundspeed for a gps based IMU heading to be considered valid
                                           // Better to have some update than none for GPS Rescue at slow return speeds

bool canUseGPSHeading = true;

static float throttleAngleScale;
static int throttleAngleValue;
static float smallAngleCosZ = 0;

static imuRuntimeConfig_t imuRuntimeConfig;

#if defined(USE_ACC)
STATIC_UNIT_TESTED bool attitudeIsEstablished = false;
#endif

// quaternion of sensor frame relative to earth frame
STATIC_UNIT_TESTED fp_quaternion_t q = QUATERNION_INITIALIZE;
STATIC_UNIT_TESTED fp_quaternionProducts_t qP = QUATERNION_PRODUCTS_INITIALIZE;
// headfree quaternions
fp_quaternion_t headfree = QUATERNION_INITIALIZE;
fp_quaternion_t offset = QUATERNION_INITIALIZE;

// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
i16_euler_t attitude = EULER_INITIALIZE;
fp_rotationMatrix_t rMat;

PG_REGISTER_WITH_RESET_TEMPLATE(imuConfig_t, imuConfig, PG_IMU_CONFIG, 2);

PG_RESET_TEMPLATE(imuConfig_t, imuConfig,
    .dcm_kp = 2500,                // 1.0 * 10000
    .dcm_ki = 0,                   // 0.003 * 10000
    .small_angle = 25,
    .imu_process_denom = 2
);

STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
    quaternionProducts_of_quaternion(&qP, &q);
    rotationMatrix_of_quaternionProducts(&rMat, &qP);

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    rMat.m[1][0] = -2.0f * (qP.xy - -qP.wz);
    rMat.m[2][0] = -2.0f * (qP.xz + -qP.wy);
#endif
}

static float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    imuRuntimeConfig.dcm_kp = imuConfig()->dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = imuConfig()->dcm_ki / 10000.0f;

    smallAngleCosZ = cos_approx(degreesToRadians(imuConfig()->small_angle));

    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);

    throttleAngleValue = throttle_correction_value;
}

void imuInit(void)
{
#ifdef USE_GPS
    canUseGPSHeading = true;
#else
    canUseGPSHeading = false;
#endif

    imuComputeRotationMatrix();

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&imuUpdateLock, NULL) != 0) {
        printf("Create imuUpdateLock error!\n");
    }
#endif
}

#if defined(USE_ACC)
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, bool useExtPosYaw,
                                float cogYawGain, float courseOverGround, const float dcmKpGain)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
    fp_vector_t zB;
    zB.V.X = rMat.m[2][X];
    zB.V.Y = rMat.m[2][Y];
    zB.V.Z = rMat.m[2][Z];

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (cogYawGain != 0.0f) {
        // Used in a GPS Rescue to boost IMU yaw gain when course over ground and velocity to home differ significantly
        while (courseOverGround >  M_PIf) {
            courseOverGround -= (2.0f * M_PIf);
        }
        while (courseOverGround < -M_PIf) {
            courseOverGround += (2.0f * M_PIf);
        }
        const float ez_ef = cogYawGain * ( sin_approx(courseOverGround) * rMat.m[0][0] - cos_approx(courseOverGround) * rMat.m[1][0] );
        ex = zB.V.X * ez_ef;
        ey = zB.V.Y * ez_ef;
        ez = zB.V.Z * ez_ef;
    }

#ifdef USE_MAG
    // Use measured magnetic field vector
    float mx = mag.magADC[X];
    float my = mag.magADC[Y];
    float mz = mag.magADC[Z];
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        const float hx = rMat.m[0][0] * mx + rMat.m[0][1] * my + rMat.m[0][2] * mz;
        const float hy = rMat.m[1][0] * mx + rMat.m[1][1] * my + rMat.m[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = (hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += zB.V.X * ez_ef;
        ey += zB.V.Y * ez_ef;
        ez += zB.V.Z * ez_ef;
    }
#else
    UNUSED(useMag);
#endif

#ifdef USE_GPS_PI
    // external position transmits psi
    if (useExtPosYaw) {
        float yawI = extPosNed.att.angles.yaw;
        while (yawI >  M_PIf) {
            yawI -= (2.0f * M_PIf);
        }
        while (yawI < -M_PIf) {
            yawI += (2.0f * M_PIf);
        }
        // reduce effect of error with tilt
        const float ez_ef = sin_approx(yawI) * rMat.m[0][0] - cos_approx(yawI) * rMat.m[1][0];
        ex += zB.V.X * ez_ef;
        ey += zB.V.Y * ez_ef;
        ez += zB.V.Z * ez_ef;
    }
#else
    UNUSED(useExtPosYaw);
#endif

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement; useAcc is true when all smoothed acc axes are within 20% of 1G
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex -= (ay * zB.V.Z - az * zB.V.Y);
        ey -= (az * zB.V.X - ax * zB.V.Z);
        ez -= (ax * zB.V.Y - ay * zB.V.X);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    fp_quaternion_t buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}

#if defined(USE_GPS_PI) && (!defined(SIMULATOR_BUILD) || defined(USE_IMU_CALC))
fp_vector_t posEstNed = {0};
fp_vector_t velEstNed = {0};
static bool posHasBeenInvalid = true;

static void imuUpdateDeadReckoning(float dt, float ax, float ay, float az, float Kp, float Ki) {
    UNUSED(Ki);

    if ((dt > 0.05f) || (dt < 0.f)) {
        // should only be momentary conditions, so we dont handle this
        return;
    }

    bool posValid = (extPosState >= EXT_POS_STILL_VALID);
    if (posValid && posHasBeenInvalid) {
        // regained position after longer period on DR: reset position and velocity
        velEstNed = extPosNed.vel;
        posEstNed = extPosNed.pos;
        posHasBeenInvalid = false;
        return;
    }

    posHasBeenInvalid = !posValid;
    Kp *= posValid; // ignore measurements, if too old

    // convert local accel measurement to NED
    fp_vector_t aNed = { .A = { ax, ay, az } };
    rotate_vector_with_rotationMatrix(&aNed, &rMat);
    aNed.V.Z += acc.dev.acc_1G; // remove gravity from accelerometer

    fp_vector_t velErrorNed = extPosNed.vel;
    VEC3_SCALAR_MULT_ADD(velErrorNed, -1.0f, velEstNed);

    VEC3_SCALAR_MULT_ADD(velEstNed, dt*acc.dev.acc_1G_rec*GRAVITYf, aNed);
    VEC3_SCALAR_MULT_ADD(velEstNed, dt*Kp, velErrorNed);

    fp_vector_t posErrorNed = extPosNed.pos;
    VEC3_SCALAR_MULT_ADD(posErrorNed, -1.0f, posEstNed);

    VEC3_SCALAR_MULT_ADD(posEstNed, dt, velEstNed);
    VEC3_SCALAR_MULT_ADD(posEstNed, dt*Kp, posErrorNed);
}
#endif

STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
    fp_quaternionProducts_t buffer;
    fp_euler_t eulerf;

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        quaternionProducts_of_quaternion(&buffer, &headfree);
        fp_euler_of_quaternionProducts(&eulerf, &buffer);
    } else {
        fp_euler_of_rotationMatrix(&eulerf, &rMat);
    }

    i16_euler_of_fp_euler(&attitude, &eulerf);

    if (attitude.angles.yaw < 0) {
        attitude.angles.yaw += 3600;
    }
}

static bool imuIsAccelerometerHealthy(float *accAverage)
{
    float accMagnitudeSq = 0;
    for (int axis = 0; axis < 3; axis++) {
        const float a = accAverage[axis];
        accMagnitudeSq += a * a;
    }

    accMagnitudeSq = accMagnitudeSq * sq(acc.dev.acc_1G_rec);

    // Accept accel readings only in range 0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}

// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
static float imuCalcKpGain(timeUs_t currentTimeUs, bool useAcc, float *gyroAverage)
{
    static bool lastArmState = false;
    static timeUs_t gyroQuietPeriodTimeEnd = 0;
    static timeUs_t attitudeResetTimeEnd = 0;
    static bool attitudeResetCompleted = false;
    float ret;
    bool attitudeResetActive = false;

    const bool armState = ARMING_FLAG(ARMED);

    if (!armState) {
        if (lastArmState) {   // Just disarmed; start the gyro quiet period
            gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
            attitudeResetTimeEnd = 0;
            attitudeResetCompleted = false;
        }

        // If gyro activity exceeds the threshold then restart the quiet period.
        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
        // start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
        if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) || attitudeResetCompleted) {
            if ((fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (!useAcc)) {

                gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
                attitudeResetTimeEnd = 0;
            }
        }
        if (attitudeResetTimeEnd > 0) {        // Resetting the attitude estimation
            if (currentTimeUs >= attitudeResetTimeEnd) {
                gyroQuietPeriodTimeEnd = 0;
                attitudeResetTimeEnd = 0;
                attitudeResetCompleted = true;
            } else {
                attitudeResetActive = true;
            }
        } else if ((gyroQuietPeriodTimeEnd > 0) && (currentTimeUs >= gyroQuietPeriodTimeEnd)) {
            // Start the high gain period to bring the estimation into convergence
            attitudeResetTimeEnd = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
            gyroQuietPeriodTimeEnd = 0;
        }
    }
    lastArmState = armState;

    if (attitudeResetActive) {
        ret = ATTITUDE_RESET_KP_GAIN;
    } else {
        ret = imuRuntimeConfig.dcm_kp;
        if (!armState) {
            ret *= 10.0f; // Scale the kP to generally converge faster when disarmed.
        }
    }

    return ret;
}

#if defined(USE_GPS)
static void imuComputeQuaternionFromRPY(fp_quaternionProducts_t *quatProd, int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    // this function seems broken. It computes fp_quaternionProducts_t, but they may get overriden by imuComputeRotationMatrix()
    if (initialRoll > 1800) {
        initialRoll -= 3600;
    }

    if (initialPitch > 1800) {
        initialPitch -= 3600;
    }

    if (initialYaw > 1800) {
        initialYaw -= 3600;
    }

    const float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    const float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    const float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    const float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    const float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    const float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    quatProd->xx = sq(q1);
    quatProd->yy = sq(q2);
    quatProd->zz = sq(q3);

    quatProd->xy = q1 * q2;
    quatProd->xz = q1 * q3;
    quatProd->yz = q2 * q3;

    quatProd->wx = q0 * q1;
    quatProd->wy = q0 * q2;
    quatProd->wz = q0 * q3;

    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}
#endif

#ifdef USE_EKF
void setPositionState(fp_vector_t posEstNed_set, fp_vector_t velEstNed_set)
{
    posEstNed = posEstNed_set;
    velEstNed = velEstNed_set;
}
#endif // USE_EKF

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    float cogYawGain = 0.0f; // IMU yaw gain to be applied in imuMahonyAHRSupdate from ground course, default to no correction from CoG
    float courseOverGround = 0; // To be used when cogYawGain is non-zero, in radians

    timeDelta_t deltaT = cmpTimeUs(currentTimeUs, previousIMUUpdateTime);
    previousIMUUpdateTime = currentTimeUs;

#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy()
#ifdef USE_GPS_RESCUE
        && !gpsRescueDisableMag()
#endif
        ) {
        useMag = true;
    }
#endif
#if defined(USE_GPS)
    if (!useMag && sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat > GPS_MIN_SAT_COUNT && gpsSol.groundSpeed >= GPS_COG_MIN_GROUNDSPEED) {
        // Use GPS course over ground to correct attitude.angles.yaw
        courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
        cogYawGain = (FLIGHT_MODE(GPS_RESCUE_MODE)) ? gpsRescueGetImuYawCogGain() : 1.0f;
        // normally update yaw heading with GPS data, but when in a Rescue, modify the IMU yaw gain dynamically
        if (shouldInitializeGPSHeading()) {
            // Reset our reference and reinitialize quaternion.
            // shouldInitializeGPSHeading() returns true only once.
            imuComputeQuaternionFromRPY(&qP, attitude.angles.roll, attitude.angles.pitch, gpsSol.groundCourse);
            cogYawGain = 0.0f; // Don't use the COG when we first initialize
        }
    }
#endif

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC)
    UNUSED(imuMahonyAHRSupdate);
    UNUSED(imuIsAccelerometerHealthy);
    UNUSED(useAcc);
    UNUSED(useMag);
    UNUSED(cogYawGain);
    UNUSED(canUseGPSHeading);
    UNUSED(courseOverGround);
    UNUSED(deltaT);
    UNUSED(imuCalcKpGain);
#else

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
//  printf("[imu]deltaT = %u, imuDeltaT = %u, currentTimeUs = %u, micros64_real = %lu\n", deltaT, imuDeltaT, currentTimeUs, micros64_real());
    deltaT = imuDeltaT;
#endif
    float gyroAverage[XYZ_AXIS_COUNT];
    for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        gyroAverage[axis] = gyroGetFilteredDownsampled(axis);
    }

    useAcc = imuIsAccelerometerHealthy(acc.accADCf); // all smoothed accADCf values are within 20% of 1G
#ifdef USE_GPS_PI
    bool useExtPosYaw = (extPosState >= EXT_POS_STILL_VALID);
#else
    bool useExtPosYaw = false;
#endif

    float Kp = imuCalcKpGain(currentTimeUs, useAcc, gyroAverage);
    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, acc.accADCf[X], acc.accADCf[Y], acc.accADCf[Z],
                        useMag, useExtPosYaw,
                        cogYawGain, courseOverGround, Kp);

    imuUpdateEulerAngles();

#ifdef USE_GPS_PI
    float KpPos = 8.f*Kp;
    imuUpdateDeadReckoning(((float) deltaT) * 1e-6f,
        acc.accADCf[X], acc.accADCf[Y], acc.accADCf[Z], KpPos, 0.f);
#endif

#endif
}

static int calculateThrottleAngleCorrection(void)
{
    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (getCosTiltAngle() <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acos_approx(getCosTiltAngle()) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttleAngleValue * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

void imuUpdateAttitude(timeUs_t currentTimeUs)
{
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
        IMU_LOCK;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
        if (imuUpdated == false) {
            IMU_UNLOCK;
            return;
        }
        imuUpdated = false;
#endif
        imuCalculateEstimatedAttitude(currentTimeUs);
        IMU_UNLOCK;

        // Update the throttle correction for angle and supply it to the mixer
        int throttleAngleCorrection = 0;
        if (throttleAngleValue && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && ARMING_FLAG(ARMED)) {
            throttleAngleCorrection = calculateThrottleAngleCorrection();
        }
        mixerSetThrottleAngleCorrection(throttleAngleCorrection);

    } else {
        acc.accADCf[X] = 0;
        acc.accADCf[Y] = 0;
        acc.accADCf[Z] = 0;
        schedulerIgnoreTaskStateTime();
    }

    DEBUG_SET(DEBUG_ATTITUDE, X, acc.accADCf[X]);
    DEBUG_SET(DEBUG_ATTITUDE, Y, acc.accADCf[Y]);
}
#endif // USE_ACC

bool shouldInitializeGPSHeading(void)
{
    static bool initialized = false;

    if (!initialized) {
        initialized = true;

        return true;
    }

    return false;
}

float getCosTiltAngle(void)
{
    return rMat.m[2][2];
}

void getAttitudeQuaternion(fp_quaternion_t *quat)
{
   quat->w = q.w;
   quat->x = q.x;
   quat->y = q.y;
   quat->z = q.z;
}

void setAttitudeWithQuaternion(const fp_quaternion_t *quat)
{
    q.w = quat->w;
    q.x = quat->x;
    q.y = quat->y;
    q.z = quat->z;

    imuComputeRotationMatrix(); // sets rotation matrix and quatenrion products
    imuUpdateEulerAngles();
}

#ifdef USE_EKF
void setAttitudeWithEuler(float roll, float pitch, float yaw)
{
    // expecting roll pitch yaw in radians
    while (roll > M_PIf) roll -= 2*M_PIf;
    while (roll < -M_PIf) roll += 2*M_PIf;

    while (pitch > M_PIf) pitch -= 2*M_PIf;
    while (pitch < -M_PIf) pitch += 2*M_PIf;

    while (yaw > M_PIf) yaw -= 2*M_PIf;
    while (yaw < -M_PIf) yaw += 2*M_PIf;

    attitude.angles.roll = (int16_t) RADIANS_TO_DECIDEGREES(roll);
    attitude.angles.pitch = (int16_t) RADIANS_TO_DECIDEGREES(pitch);
    attitude.angles.yaw = (int16_t) RADIANS_TO_DECIDEGREES(yaw);

    fp_euler_t eulerf;
    fp_euler_of_i16_euler(&eulerf, &attitude);
    quaternion_of_fp_euler(&q, &eulerf);
    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}
#endif

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw)
{
    IMU_LOCK;

    attitude.angles.roll = roll * 10;
    attitude.angles.pitch = pitch * 10;
    attitude.angles.yaw = yaw * 10;

    IMU_UNLOCK;
}

void imuSetAttitudeQuat(float w, float x, float y, float z)
{
    IMU_LOCK;

    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;

    imuComputeRotationMatrix();

    attitudeIsEstablished = true;

    imuUpdateEulerAngles();

    IMU_UNLOCK;
}
#endif
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt)
{
    IMU_LOCK;

    imuUpdated = true;
    imuDeltaT = dt;

    IMU_UNLOCK;
}
#endif

bool imuQuaternionHeadfreeOffsetSet(void)
{
    if ((abs(attitude.angles.roll) < 450)  && (abs(attitude.angles.pitch) < 450)) {
        const float yaw = atan2_approx((+2.0f * (qP.wz + qP.xy)), (+1.0f - 2.0f * (qP.yy + qP.zz)));

        offset.w = cos_approx(yaw/2);
        offset.x = 0;
        offset.y = 0;
        offset.z = sin_approx(yaw/2);

        return true;
    } else {
        return false;
    }
}

void imuQuaternionHeadfreeTransformVectorEarthToBody(fp_vector_t *v)
{
    headfree = chain_quaternion(&offset, &q);
    headfree.w *= -1.0f; // we need inverse!
    rotate_vector_with_quaternion(v, &headfree);
    headfree.w *= -1.0f; // return
}

bool isUpright(void)
{
#ifdef USE_ACC
    return !sensors(SENSOR_ACC) || (attitudeIsEstablished && getCosTiltAngle() > smallAngleCosZ);
#else
    return true;
#endif
}
