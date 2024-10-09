/*
 * Interface to run a subset of Indiflight without real-time scheduler
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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

// TODO: integrate in target.c, and remove this file

#include "platform.h"
#include "target.h"

#include <stdio.h>
#include "common/maths.h"
#include "common/time.h"
#include "flight/learner.h"
#include "flight/throw.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "fc/init.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "io/external_pos.h"
#include "fc/tasks.h"
#include "scheduler/scheduler.h"

#ifndef USE_TELEMETRY_PI
#error "MOCKUP target requires USE_TELEMETRY_PI"
#endif

#ifndef USE_POS_CTL
#error "MOCKUP target requires USE_POS_CTL"
#endif

#ifndef USE_ACC
#error "MOCKUP target requires USE_ACC"
#endif

void setImu(const float *g, const float *a) {
    // g in rad/s, a in N/kg

    for (int axis = 0; axis < 3; axis++) {
        gyro.gyroADC[axis]   = RADIANS_TO_DEGREES(g[axis]); // shouldnt be used
        acc.dev.ADCRaw[axis] = a[axis] / 9.81f * acc.dev.acc_1G; // input is in g
    }
    gyroUpdate(); // rotate and pretend to downsample
    getTask(TASK_FILTER)->attribute->taskFunc( micros() ); // filter gyro

    accUpdate(micros());
#ifdef USE_THROW_TO_ARM
    updateThrowFallStateMachine(micros());
#endif
}

void setMotorSpeed(const float *omega, const int n) {
    // in rad/s
    int lim = MIN(n, getMotorCount());
    for (int motor = 0; motor < lim; motor++) {
        motorOmegaValues[motor] = omega[motor];
    }
}

void setMocap(const float *pos, const float *vel, const float *q) {
    extPosState = EXT_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    extLatestMsgTimeReceived = micros();
    for (int axis = 0; axis < 3; axis++) {
        extPosNed.pos.A[axis] = pos[axis];
        extPosNed.vel.A[axis] = vel[axis];
    }
    extPosNed.quat.w = q[0];
    extPosNed.quat.x = q[1];
    extPosNed.quat.y = q[2];
    extPosNed.quat.z = q[3];
}

void setPosSetpoint(const float *pos, const float yaw) {
    posSetpointState = EXT_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    extLatestMsgTimeReceived = micros();
    // meters, NED. rad
    for (int axis = 0; axis < 3; axis++)
        posSpNed.pos.A[axis] = pos[axis];

    posSpNed.psi = yaw;
    posSpNed.trackPsi = true;
}

void getMotorOutputCommands(float *cmd, int n) {
    int lim = MIN(n, getMotorCount());
    for (int m = 0; m < lim; m++) {
        if (ARMING_FLAG(ARMED)) {
            cmd[m] = motor_normalized[m];
        } else {
            cmd[m] = motor_disarmed[m];
        }
    }
}

void tick(const timeDelta_t dtUs)
{
    clock_tick( dtUs );
    timeUs_t currentTimeUs = micros();

    unsetArmingDisabled(0xffffffff); // disable all, always

    static uint8_t counter = 0;
    if (++counter % 2 == 1) {
        // todo: make this demon nicer
        counter = 1;
        if (throwState == THROW_STATE_THROWN) {
            // because we bypass updateArmingStatus, we need to do this here manually
            armingFlags = 1;
        }
        getTask(TASK_ATTITUDE)->attribute->taskFunc( currentTimeUs );
#ifdef USE_EKF
        getTask(TASK_EKF)->attribute->taskFunc( currentTimeUs );
#endif
        getTask(TASK_POS_CTL)->attribute->taskFunc( currentTimeUs );
    } 

    getTask(TASK_INNER_LOOP)->attribute->taskFunc( currentTimeUs );

    if (cmpTimeUs(currentTimeUs, extLatestMsgTimeReceived) > EXT_POS_TIMEOUT_US) {
        extPosState = EXT_POS_NO_SIGNAL;
    } else {
        extPosState = EXT_POS_STILL_VALID;
    }
}
