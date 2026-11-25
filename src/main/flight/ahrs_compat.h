/*
 * This file is part of Indiflight.
 *
 * Maps the attitude interface used by indiflight controllers (formerly
 * flight/ahrs.h) onto upstream flight/imu.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#pragma once

#include "common/maths_indi.h"
#include "flight/imu.h"

// attitude of the body frame relative to the earth frame
void getAttitudeQuaternion(fp_quaternion_t *q);
// same, but rotated by the hover attitude offset (tailsitters / learner)
void getHoverAttitudeQuaternion(fp_quaternion_t *q);
