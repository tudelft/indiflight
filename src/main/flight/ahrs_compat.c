/*
 * This file is part of Indiflight.
 *
 * See ahrs_compat.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include "platform.h"

#ifdef USE_INDI

#include "flight/ahrs_compat.h"
#ifdef USE_EKF
#include "flight/ekf.h"
#endif

// imu.c (and the EKF) hold the FRD body -> NED earth quaternion, the same
// convention as indiflight's ahrs.c, so this is a plain copy.
void getAttitudeQuaternion(fp_quaternion_t *q)
{
#ifdef USE_EKF
    // imu.c only runs at the attitude task rate; INDI needs the EKF at loop rate
    if (shouldBeUsedEkf() && isConvergedEkf()) {
        *q = qEkf;
        return;
    }
#endif
    quaternion_t qImu;
    getQuaternion(&qImu);
    q->w = qImu.w;
    q->x = qImu.x;
    q->y = qImu.y;
    q->z = qImu.z;
}

void getHoverAttitudeQuaternion(fp_quaternion_t *q)
{
    // without USE_LEARNER, indiflight's hover attitude equals the attitude
    getAttitudeQuaternion(q);
}

#endif // USE_INDI
