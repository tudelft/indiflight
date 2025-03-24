/*
 * 
 *
 * Copyright 2023 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Improved integration with legacy estimator, added parameters
 * Copyright 2025 Till Blaha (Delft University of Technology)
 *     Upstream quaternion EKF, remove legacy estimator
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
#include "common/maths.h"
#include "ekf_calc.h"

#include "pg/pg.h"

typedef struct ekfConfig_s {
    uint8_t use_quat_measurement;      // bool
    uint8_t use_for_manual_flight;
    uint8_t meas_source;
    int32_t global_home_lat;
    int32_t global_home_lon;
    uint32_t proc_noise_acc[3];        // noise covariance * 1e6
    uint32_t proc_noise_gyro[3];       // noise covariance * 1e6
    uint32_t proc_noise_acc_bias[3];   // noise covariance * 1e6
    uint32_t proc_noise_gyro_bias[3];  // noise covariance * 1e6
    uint32_t meas_noise_position[3];   // noise covariance * 1e6
    uint32_t meas_noise_quat[4];       // noise covariance * 1e6
    uint8_t meas_delay;                // ms
} ekfConfig_t;

PG_DECLARE(ekfConfig_t, ekfConfig);

#define EKF_MAX_MEAS_AGE_US 100000   // 0.1 seconds
#define EKF_DEINIT_TIMEOUT 500000    // 0.5 seconds
#define EKF_CONVERGE_TIME_US 2000000 // 2 seconds

bool shouldBeUsedEkf(void);
bool isConvergedEkf(void);
void initEkf(timeUs_t currentTimeUs);
void forceDeinitEkf(void);
void updateEkf(timeUs_t currentTimeUs);

extern fp_vector_t posEstNed;
extern fp_vector_t velEstNed;
extern fp_quaternion_t qEkf;

#endif // EKF_H
