/*
 * 
 *
 * Copyright 2023 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Improved integration with legacy estimator, added parameters
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


#ifndef EKF_H
#define EKF_H

#include "common/time.h"			// for timeUs_t
#include "common/maths.h"			// for timeUs_t
#include "ekf_calc.h"

#include "pg/pg.h"

typedef struct ekfConfig_s {
    uint8_t use_attitude_estimate;     // use in INS
    uint8_t use_position_estimate;     // use in INS
    uint8_t use_quat_measurement;      // yes or no for now
    uint32_t proc_noise_acc[3];        // noise covariance * 1e6
    uint32_t proc_noise_gyro[3];       // noise covariance * 1e6
    uint32_t proc_noise_acc_bias[3];   // noise covariance * 1e6
    uint32_t proc_noise_gyro_bias[3];  // noise covariance * 1e6
    uint32_t meas_noise_position[3];   // noise covariance * 1e6
    uint32_t meas_noise_quat[4];       // noise covariance * 1e6
    uint8_t meas_delay;                // ms
} ekfConfig_t;

PG_DECLARE(ekfConfig_t, ekfConfig);

extern fp_quaternion_t qEkf;

bool isInitializedEkf(void);
void initEkf(timeUs_t currentTimeUs);
void updateEkf(timeUs_t currentTimeUs);

#endif // EKF_H
