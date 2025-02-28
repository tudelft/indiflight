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
extern i16_euler_t attitude;
extern fp_rotationMatrix_t rMat;

typedef struct ahrsConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t ahrs_process_denom;
} ahrsConfig_t;

PG_DECLARE(ahrsConfig_t, ahrsConfig);

typedef struct ahrsRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} ahrsRuntimeConfig_t;

void ahrsConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

float getCosTiltAngle(void);
void getAttitudeQuaternion(fp_quaternion_t * q);
void getHoverAttitudeQuaternion(fp_quaternion_t *q);
#ifdef USE_LEARNER
void overrideAttitudeQuaternion(fp_quaternion_t *quat);
#endif
void ahrsDecider(void);
void ahrsUpdate(timeUs_t currentTimeUs);

void ahrsInit(void);

#ifdef SIMULATOR_BUILD
void ahrsSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void ahrsSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_AHRS_SYNC)
void ahrsSetHasNewData(uint32_t dt);
#endif
#endif

bool isUpright(void);
