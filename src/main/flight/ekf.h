

#ifndef EKF_H
#define EKF_H

#include "common/time.h"			// for timeUs_t
#include "ekf_calc.h"

// useful macros
// #define getEkfPosNed() (ekf_get_X()[0])
// #define getEkfVelNed() (ekf_get_X()[3])
// #define getEkfAtt() (ekf_get_X()[6])
// #define getEkfAccBias() (ekf_get_X()[9])
// #define getEkfGyroBias() (ekf_get_X()[12])


void updateEkf(timeUs_t currentTimeUs);

#endif // EKF_H
