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

// Attitude and Heading Reference System (AHRS)

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

#include "flight/ekf.h"
#include "flight/learner.h"
#include "flight/gps_rescue.h"
#include "flight/ahrs.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/local_pos.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#include "ahrs.h"

static pthread_mutex_t ahrsUpdateLock;

#if defined(SIMULATOR_AHRS_SYNC)
static uint32_t ahrsDeltaT = 0;
static bool ahrsUpdated = false;
#endif

#define AHRS_LOCK pthread_mutex_lock(&ahrsUpdateLock)
#define AHRS_UNLOCK pthread_mutex_unlock(&ahrsUpdateLock)

#else

#define AHRS_LOCK
#define AHRS_UNLOCK

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

static float throttleAngleScale;
static int throttleAngleValue;
static float smallAngleCosZ = 0;

static ahrsRuntimeConfig_t ahrsRuntimeConfig;

#if defined(USE_ACC)
STATIC_UNIT_TESTED bool attitudeIsEstablished = false;
#endif

// quaternion of sensor frame relative to earth frame
STATIC_UNIT_TESTED fp_quaternion_t q = QUATERNION_INITIALIZE;
STATIC_UNIT_TESTED fp_quaternionProducts_t qP = QUATERNION_PRODUCTS_INITIALIZE;
static fp_quaternion_t qHover = QUATERNION_INITIALIZE;

// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
i16_euler_t attitude = EULER_INITIALIZE;
fp_rotationMatrix_t rMat = ROTATION_MATRIX_INITIALIZE;

static fp_quaternion_t qMahony = QUATERNION_INITIALIZE;
static fp_quaternionProducts_t qPMahony = QUATERNION_PRODUCTS_INITIALIZE;
static fp_rotationMatrix_t rMatMahony = ROTATION_MATRIX_INITIALIZE;

PG_REGISTER_WITH_RESET_TEMPLATE(ahrsConfig_t, ahrsConfig, PG_AHRS_CONFIG, 2);

PG_RESET_TEMPLATE(ahrsConfig_t, ahrsConfig,
    .dcm_kp = 2500,                // 1.0 * 10000
    .dcm_ki = 0,                   // 0.003 * 10000
    .small_angle = 25,
    .ahrs_process_denom = 16
);

static float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void ahrsConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    ahrsRuntimeConfig.dcm_kp = ahrsConfig()->dcm_kp / 10000.0f;
    ahrsRuntimeConfig.dcm_ki = ahrsConfig()->dcm_ki / 10000.0f;

    smallAngleCosZ = cos_approx(degreesToRadians(ahrsConfig()->small_angle));

    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
    throttleAngleValue = throttle_correction_value;
}

void ahrsInit(void)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&ahrsUpdateLock, NULL) != 0) {
        printf("Create ahrsUpdateLock error!\n");
    }
#endif
}

#if defined(USE_ACC)
static void mahonyUpdate(float dt, fp_vector_t* rate, bool useAcc, fp_vector_t* acc, bool useMag,
                                const float dcmKpGain)
{
    const float rate_norm = VEC3_LENGTH((*rate)); // rad/s
    const float acc_norm  = VEC3_LENGTH((*acc));

    fp_vector_t e           = { 0 }; // rotation error
    static fp_vector_t eint = { 0 }; // rotation error integral

    // global z in body frame
    fp_vector_t zB = { 0 };
    zB.V.X = rMatMahony.m[2][X];
    zB.V.X = rMatMahony.m[2][Y];
    zB.V.Z = rMatMahony.m[2][Z];

#ifdef USE_MAG
    // Use measured magnetic field vector
    fp_vector_t m = { .V.X = mag.magADC[X], .V.Y = mag.magADC[Y], .V.Z = mag.magADC[Z] };
    float mag_norm = VEC3_LENGTH(m);
    if (useMag && mag_norm > 0.01f) {
        // Normalise magnetometer measurement
        VEC3_NORMALIZE(m);

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        rotate_vector_with_rotationMatrix(&m, &rMatMahony); // measured mag field vector in EF 
        const float bx = VEC3_XY_LENGTH(m);                 // reference mag field vector heading due North in EF (assuming Z-component is zero)

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = (m.V.Y * bx); // assuming mz = 0

        // Rotate mag error vector back to BF and accumulate
        VEC3_SCALAR_MULT_ADD(e, ez_ef, zB); // e += ez_ef * zB
    }
#else
    UNUSED(useMag);
#endif

    // Use measured acceleration vector
    if (useAcc && acc_norm > 0.01f) {
        // Normalise accelerometer measurement; useAcc is true when all smoothed acc axes are within 20% of 1G
        VEC3_NORMALIZE((*acc));

        // Error is sum of cross product between estimated direction and measured direction of gravity
        fp_vector_t xprod;
        VEC3_CROSS(xprod, zB, (*acc));       // xprod = zB x a
        VEC3_SCALAR_MULT_ADD(e, 1.f, xprod); // e += zB x a
    }

    // Compute and apply integral feedback if enabled
    if (ahrsRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (rate_norm < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            VEC3_SCALAR_MULT_ADD(eint, dt, e);
        }
    } else {
        eint.V.X = 0.f; eint.V.Y = 0.f; eint.V.Z = 0.f; // prevent integral windup
    }

    // Apply proportional and integral feedback
    VEC3_SCALAR_MULT_ADD((*rate), dcmKpGain, e);
    VEC3_SCALAR_MULT_ADD((*rate), ahrsRuntimeConfig.dcm_ki, eint);

    // Integrate rate of change of quaternion
    quaternion_integrate_body_rates(&qMahony, rate, dt);
}

static bool ahrsIsAccelerometerHealthy(fp_vector_t *accAverage)
{
    // Accept accel readings only in range 0.85g - 1.15g
    float accMagnitude = VEC3_LENGTH((*accAverage));
    return (0.85f < accMagnitude) && (accMagnitude < 1.15f);
}

// Calculate the dcmKpGain to use. When armed, the gain is ahrsRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
static float ahrsCalcKpGain(timeUs_t currentTimeUs, bool useAcc, fp_vector_t *gyroAverage)
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
            if ((fabsf(gyroAverage->V.X) > DEGREES_TO_RADIANS(ATTITUDE_RESET_GYRO_LIMIT))
                || (fabsf(gyroAverage->V.Y) > DEGREES_TO_RADIANS(ATTITUDE_RESET_GYRO_LIMIT))
                || (fabsf(gyroAverage->V.Z) > DEGREES_TO_RADIANS(ATTITUDE_RESET_GYRO_LIMIT))
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
        ret = ahrsRuntimeConfig.dcm_kp;
        if (!armState) {
            ret *= 10.0f; // Scale the kP to generally converge faster when disarmed.
        }
    }

    return ret;
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

FAST_CODE void ahrsDecider(void) {
#ifdef USE_EKF
    if (isInitializedEkf() 
#ifdef USE_AHRS_FALLBACK_LOGIC
    && (FLIGHT_MODE(POSITION_MODE) || FLIGHT_MODE(VELOCITY_MODE) || FLIGHT_MODE(NN_MODE) || FLIGHT_MODE(CATAPULT_MODE))
#endif
        ) {
        // we should only have POSITION_MODE when ekf is intialized, but just to be safe we check it

        q.w = qEkf.w;
        q.x = qEkf.x;
        q.y = qEkf.y;
        q.z = qEkf.z;
        quaternionProducts_of_quaternion(&qP, &q);
        rotationMatrix_of_quaternionProducts(&rMat, &qP);
    } else
#endif // USE_EKF
    {
        q = qMahony;
        qP = qPMahony;
        rMat = rMatMahony;
    }

#ifdef USE_LEARNER
    qHover = chain_quaternion(&q, &hoverAttitude);
#else
    qHover = q;
#endif

    fp_euler_t euler_fp;
    fp_euler_of_quaternionProducts(&euler_fp, &qP);
    i16_euler_of_fp_euler(&attitude, &euler_fp);

    // correct yaw to not be negative
    if (attitude.angles.yaw < 0) { attitude.angles.yaw += 3600; }

    attitudeIsEstablished = true;
}

void ahrsUpdate(timeUs_t currentTimeUs)
{
    // ----- check if we even need to calculate anything
#if defined(SIMULATOR_BUILD) && !defined(USE_AHRS_CALC) // no AHRS calcs necessary, return early
    UNUSED(gyroGetFilteredDownsampled);
    UNUSED(ahrsIsAccelerometerHealthy);
    UNUSED(compassIsHealthy);
    UNUSED(ahrsCalcKpGain);
    UNUSED(mahonyUpdate);
    UNUSED(ahrsDecider);
    return;
#endif // we need to calculate AHRS

    static timeUs_t previousAHRSUpdateTime = 0;
    timeDelta_t deltaT = cmpTimeUs(currentTimeUs, previousAHRSUpdateTime);
    previousAHRSUpdateTime = currentTimeUs;

    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce && deltaT < 100000) {
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_AHRS_SYNC)
        if (ahrsUpdated == false) {
            AHRS_UNLOCK;
            return;
        }
        ahrsUpdated = false;

//        printf("[ahrs]deltaT = %u, ahrsDeltaT = %u, currentTimeUs = %u, micros64_real = %lu\n", deltaT, ahrsDeltaT, currentTimeUs, micros64_real());
        deltaT = ahrsDeltaT;
#endif

        // ----- perpare input data
        AHRS_LOCK;
        fp_vector_t gyroAverage = {0};
        for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
            gyroAverage.A[axis] = DEGREES_TO_RADIANS(gyroGetFilteredDownsampled(axis));
        }

        fp_vector_t accAverage = { .V.X = acc.accADCf[X], .V.Y = acc.accADCf[Y], .V.Z = acc.accADCf[Z] };
        bool useAcc = ahrsIsAccelerometerHealthy(&accAverage);

#ifdef USE_MAG
        bool useMag = (sensors(SENSOR_MAG) && compassIsHealthy());
#else
        bool useMag = false;
#endif

        float Kp = ahrsCalcKpGain(currentTimeUs, useAcc, &gyroAverage);

        // ----- calculate gain and perform attitude update
        mahonyUpdate(1e-6f*deltaT, &gyroAverage, useAcc, &accAverage, useMag, Kp);
        AHRS_UNLOCK;

        // ----- Pre-compute rotation matrix from quaternion
        quaternionProducts_of_quaternion(&qPMahony, &qMahony);
        rotationMatrix_of_quaternionProducts(&rMatMahony, &qPMahony);

        // ----- Update the throttle correction for angle and supply it to the mixer
        int throttleAngleCorrection = 0;
        if (throttleAngleValue && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && ARMING_FLAG(ARMED)) {
            throttleAngleCorrection = calculateThrottleAngleCorrection();
        }
        mixerSetThrottleAngleCorrection(throttleAngleCorrection);

        // ----- assign, if used. this also calculates euler angles, and rMat
        ahrsDecider();
    } else {
        if (!sensors(SENSOR_ACC) || !acc.isAccelUpdatedAtLeastOnce) {
            acc.accADCf[X] = 0;
            acc.accADCf[Y] = 0;
            acc.accADCf[Z] = 0;
        }
        schedulerIgnoreTaskStateTime();
    }

    DEBUG_SET(DEBUG_ATTITUDE, X, acc.accADCf[X]);
    DEBUG_SET(DEBUG_ATTITUDE, Y, acc.accADCf[Y]);
}
#endif // USE_ACC

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

void getHoverAttitudeQuaternion(fp_quaternion_t *quat)
{
   quat->w = qHover.w;
   quat->x = qHover.x;
   quat->y = qHover.y;
   quat->z = qHover.z;
}

#ifdef USE_LEARNER
void overrideAttitudeQuaternion(fp_quaternion_t *quat)
{
   q.w = quat->w;
   q.x = quat->x;
   q.y = quat->y;
   q.z = quat->z;
}
#endif

#ifdef SIMULATOR_BUILD
void ahrsSetAttitudeRPY(float roll, float pitch, float yaw)
{
    AHRS_LOCK;

    attitude.angles.roll = roll * 10;
    attitude.angles.pitch = pitch * 10;
    attitude.angles.yaw = yaw * 10;

    AHRS_UNLOCK;
}

void ahrsSetAttitudeQuat(float w, float x, float y, float z)
{
    AHRS_LOCK;

    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;

    quaternionProducts_of_quaternion(&qP, &q);
    rotationMatrix_of_quaternionProducts(&rMat, &qP);

    fp_euler_t euler_fp;
    fp_euler_of_quaternionProducts(&euler_fp, &qP);
    i16_euler_of_fp_euler(&attitude, &euler_fp);

    // correct yaw to not be negative
    if (attitude.angles.yaw < 0) { attitude.angles.yaw += 3600; }

    attitudeIsEstablished = true;

    AHRS_UNLOCK;
}
#endif
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_AHRS_SYNC)
void ahrsSetHasNewData(uint32_t dt)
{
    AHRS_LOCK;

    ahrsUpdated = true;
    ahrsDeltaT = dt;

    AHRS_UNLOCK;
}
#endif

bool isUpright(void)
{
#ifdef USE_ACC
    return !sensors(SENSOR_ACC) || (attitudeIsEstablished && getCosTiltAngle() > smallAngleCosZ);
#else
    return true;
#endif
}
