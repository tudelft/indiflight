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

// TODO(spike): verify frame conventions against indiflight's ahrs.c on
// hardware. Indiflight's Mahony uses the same q layout as Betaflight's imu.c,
// so this is a plain copy for now. EKF attitude should be fed into imu.c
// instead (see report), so all upstream consumers see it too.
void getAttitudeQuaternion(fp_quaternion_t *q)
{
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
