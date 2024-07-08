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
