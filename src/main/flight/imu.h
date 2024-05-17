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

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "pg/pg.h"

// Exported symbols
extern bool canUseGPSHeading;

extern i16_euler_t attitude;
extern fp_rotationMatrix_t rMat;

typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

#ifdef USE_EKF

#ifdef USE_EKF_ATTITUDE
void setAttitudeWithEuler(float roll, float pitch, float yaw);
#endif
#ifdef USE_EKF_POSITION
void setPositionState(fp_vector_t posEstNed_set, fp_vector_t velEstNed_set);
#endif

#endif //USE_EKF

float getCosTiltAngle(void);
void getAttitudeQuaternion(fp_quaternion_t * q);
void setAttitudeWithQuaternion(const fp_quaternion_t * q);
void imuUpdateAttitude(timeUs_t currentTimeUs);

void imuInit(void);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(fp_vector_t * v);
bool shouldInitializeGPSHeading(void);
bool isUpright(void);

#ifdef USE_GPS_PI
extern fp_vector_t posEstNed;
extern fp_vector_t velEstNed;
//void imuUpdateDeadReckoning(float dt, float ax, float ay, float az, const float Kp);
#endif
