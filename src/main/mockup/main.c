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
#include "fc/core.h"
#include "flight/learner.h"
#include "flight/throw.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "fc/init.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"

#include "mockup/main.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "io/local_pos.h"
#include "fc/tasks.h"
#include "scheduler/scheduler.h"

#ifndef USE_TELEMETRY_PI
#error "MOCKUP target requires USE_TELEMETRY_PI"
#endif

#ifndef USE_LOCAL_POSITION
#error "MOCKUP target requires USE_LOCAL_POSITION"
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
}

void setMotorSpeed(const float *omega, const int n) {
    // in rad/s
    int lim = MIN(n, getMotorCount());
    for (int motor = 0; motor < lim; motor++) {
        motorOmegaValues[motor] = omega[motor];
    }
}

void setMocap(const float *pos, const float *vel, const float *q) {
    posMeasState = LOCAL_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    posLatestMsgTime = micros();
    //posLatestMsgTimeReceived = micros();
    for (int axis = 0; axis < 3; axis++) {
        posMeasNed.pos.A[axis] = pos[axis];
        posMeasNed.vel.A[axis] = vel[axis];
    }
    posMeasNed.quat.w = q[0];
    posMeasNed.quat.x = q[1];
    posMeasNed.quat.y = q[2];
    posMeasNed.quat.z = q[3];
}

void setMocapT(const float *pos, const float *vel, const float *q, const uint32_t time_us) {
    posMeasState = LOCAL_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    posLatestMsgTime = micros();
    //posLatestMsgTimeReceived = micros();
    for (int axis = 0; axis < 3; axis++) {
        posMeasNed.pos.A[axis] = pos[axis];
        posMeasNed.vel.A[axis] = vel[axis];
    }
    posMeasNed.quat.w = q[0];
    posMeasNed.quat.x = q[1];
    posMeasNed.quat.y = q[2];
    posMeasNed.quat.z = q[3];
    posMeasNed.time_us = time_us;
}

void setPosSetpoint(const float *pos, const float yaw) {
    posSpState = LOCAL_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    posLatestMsgTime = micros();
    //posLatestMsgTimeReceived = micros();
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

#define MOCKUP_TICK_DT_US 125

void tick(void)
{
    clock_tick( MOCKUP_TICK_DT_US );
    timeUs_t currentTimeUs = micros();

    unsetArmingDisabled(0xffffffff); // disable all, always

    if (throwState == THROW_STATE_THROWN) {
        // because we bypass updateArmingStatus, we need to do this here manually
        armingFlags = 1;
    }

    /* copied from scheduler */
    getTask(TASK_IMU)->attribute->taskFunc( currentTimeUs );

    bool filterInnerLoopShouldRun = filterReady(); // save running this function twice
    if (filterInnerLoopShouldRun) {
        getTask(TASK_FILTER)->attribute->taskFunc( currentTimeUs );
    }

    if (stateEstimationReady()) {
#ifdef USE_EKF
        // if no position measurement available at all, then EKF runs 
        // the fallback TASK_ATTITUDE itself
        getTask(TASK_EKF)->attribute->taskFunc( currentTimeUs );
#else
        getTask(TASK_ATTITUDE)->attribute->taskFunc( currentTimeUs );
#endif
        // addition in mockup, just run POS_CTL right after EKF
        getTask(TASK_POS_CTL)->attribute->taskFunc( currentTimeUs );
    }

    if (filterInnerLoopShouldRun) {
        getTask(TASK_INNER_LOOP)->attribute->taskFunc( currentTimeUs );
    }

    if ((posMeasState == LOCAL_POS_NO_SIGNAL) || (cmpTimeUs(currentTimeUs, posLatestMsgTime) > LOCAL_POS_TIMEOUT_US)) { // or received?
        posMeasState = LOCAL_POS_NO_SIGNAL;
    } else {
        posMeasState = LOCAL_POS_STILL_VALID;
    }
}
