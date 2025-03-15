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
#include <stdint.h>
#include "common/maths.h"
#include "common/time.h"
#include "fc/core.h"
#include "flight/learner.h"
#include "flight/throw.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "flight/servos.h"
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

#include "telemetry/uros.h"

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
        motorOmegaValues[motor] = omega[motor]; // rad/s
    }
}

void setServoAngle(const float *angles, const int n) {
    int lim = MIN(n, MAX_SUPPORTED_SERVOS);
    for (int servo = 0; servo < lim; servo++) {
        servo_feedback[servo] = (int16_t) 100*RADIANS_TO_DEGREES(angles[servo]); // rad to centidegrees
    }
}

void setMocap(const float *pos, const float *vel, const float *q) {
    setMocapT(pos, vel, q, micros());
}

void setMocapT(const float *pos, const float *vel, const float *q, const uint32_t time_us) {
    local_pos_ned_t new_pos;

    new_pos.source = LOCAL_POS_SOURCE_MOCKUP;
    new_pos.new = true; // just always set this.. don't know how to handle it better
    new_pos.time_us = time_us;
    for (int axis = 0; axis < 3; axis++) {
        new_pos.pos.A[axis] = pos[axis];
        new_pos.vel.A[axis] = vel[axis];
    }
    new_pos.quat.w = q[0];
    new_pos.quat.x = q[1];
    new_pos.quat.y = q[2];
    new_pos.quat.z = q[3];

    setLocalPosMeas(&new_pos);
}

void setPosSetpoint(const float *pos, const float yaw) {
    local_pos_sp_ned_t sp;

    sp.source = LOCAL_POS_SOURCE_MOCKUP;
    sp.new = true; // just always set this.. don't know how to handle it better
    sp.time_us = micros();
    //posLatestMsgTimeReceived = micros();
    // meters, NED. rad
    for (int axis = 0; axis < 3; axis++) {
        sp.pos.A[axis] = pos[axis];
    }

    sp.psi = yaw;
    sp.trackPsi = true;

    setLocalPosSp(&sp);
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

void getServoOutputCommands(float *cmd, int n) {
    int lim = MIN(n, MAX_SUPPORTED_SERVOS);
    for (int m = 0; m < lim; m++) {
        cmd[m] = servo_normalized[m];
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
#ifdef USE_TELEMETRY_UROS
        static int i = 0;
        if (++i % 16 == 0) {
            urosUpdate(currentTimeUs);
        }
#endif
    }

    if (stateEstimationReady()) {
#ifdef USE_EKF
        getTask(TASK_EKF)->attribute->taskFunc( currentTimeUs );
#endif
        // run fallback attitude estimator (which also runs the decider)
        getTask(TASK_AHRS)->attribute->taskFunc( currentTimeUs );

        // addition in mockup, just run POS_CTL right after EKF
        getTask(TASK_POS_CTL)->attribute->taskFunc( currentTimeUs );
    }

    if (filterInnerLoopShouldRun) {
        getTask(TASK_INNER_LOOP)->attribute->taskFunc( currentTimeUs );
    }
}
