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
#include "ekf_calc.h"

#include "pg/pg.h"

// useful macros
// #define getEkfPosNed() (ekf_get_X()[0])
// #define getEkfVelNed() (ekf_get_X()[3])
// #define getEkfAtt() (ekf_get_X()[6])
// #define getEkfAccBias() (ekf_get_X()[9])
// #define getEkfGyroBias() (ekf_get_X()[12])

typedef struct ekfConfig_s {
    uint8_t use_attitude_estimate;     // use in INS
    uint8_t use_position_estimate;     // use in INS
    uint8_t use_angle_measurements[3]; // booleans for phi, theta, psi
    uint32_t proc_noise_acc[3];        // noise covariance * 1e4
    uint32_t proc_noise_gyro[3];       // noise covariance * 1e4
    uint32_t meas_noise_position[3];   // noise covariance * 1e4
    uint32_t meas_noise_angles[3];     // noise covariance * 1e4
    uint8_t meas_delay;                // ms
} ekfConfig_t;

PG_DECLARE(ekfConfig_t, ekfConfig);

void updateEkf(timeUs_t currentTimeUs);

#endif // EKF_H
