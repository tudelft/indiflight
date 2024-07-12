
#include "platform.h"
#include "target.h"

#include <stdio.h>
#include "common/maths.h"
#include "common/time.h"
#include "flight/learner.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "fc/init.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "io/external_pos.h"
#include "fc/tasks.h"
#include "scheduler/scheduler.h"

void setImu(const float *g, const float *a) {
    // g in rad/s, a in N/kg

    for (int axis = 0; axis < 3; axis++) {
        gyro.gyroADC[axis]   = RADIANS_TO_DEGREES(g[axis]); // shouldnt be used
        acc.dev.ADCRaw[axis] = a[axis] / 9.81f * acc.dev.acc_1G; // input is in g
    }
}

void setMotorSpeed(const float *omega) {
    // in rad/s
    for (int motor = 0; motor < getMotorCount(); motor++) {
        motorOmegaValues[motor] = omega[motor];
    }
}

void setMocap(const float *pos, const float *vel, const float *q) {
    extPosState = EXT_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    extLatestMsgTime = micros();
    for (int axis = 0; axis < 3; axis++) {
        extPosNed.pos.A[axis] = pos[axis];
        extPosNed.vel.A[axis] = vel[axis];
    }
    fp_euler_t eulers;
    fp_quaternion_t quat;
    // the quaternion x,y,z are in ENU, we convert them to NED
    quat.w = q[0];
    quat.x = q[1];
    quat.y = q[2];
    quat.z = q[3];
    fp_quaternionProducts_t qP;
    quaternionProducts_of_quaternion(&qP, &quat);
    fp_euler_of_quaternionProducts (&eulers, &qP);
    extPosNed.att.angles.roll = eulers.angles.roll;
    extPosNed.att.angles.pitch = eulers.angles.pitch;
    extPosNed.att.angles.yaw = eulers.angles.yaw;
}

void setPosSetpoint(const float *pos, const float yaw) {
    posSetpointState = EXT_POS_NEW_MESSAGE; // just always set this.. don't know how to handle it better
    // meters, NED. rad
    for (int axis = 0; axis < 3; axis++)
        posSpNed.pos.A[axis] = pos[axis];

    posSpNed.psi = yaw;
}

void getMotorOutputCommands(float *cmd, int n) {
    int lim = MIN(n, getMotorCount());
    for (int m = 0; m < lim; m++) {
        cmd[m] = motor[m];
    }
}

void tick(const timeDelta_t dtUs)
{
    clock_tick( dtUs );
    timeUs_t currentTimeUs = micros();

    static uint8_t counter = 0;
    gyroUpdate(); // rotate and pretend to downsample
    getTask(TASK_FILTER)->attribute->taskFunc( currentTimeUs );
    if (++counter % 2 == 1) {
        // todo: make this demon nicer
        counter = 1;
        getTask(TASK_ACCEL)->attribute->taskFunc( currentTimeUs );
        getTask(TASK_ATTITUDE)->attribute->taskFunc( currentTimeUs );
        getTask(TASK_EKF)->attribute->taskFunc( currentTimeUs );
        getTask(TASK_POS_CTL)->attribute->taskFunc( currentTimeUs );
    } 

    getTask(TASK_INNER_LOOP)->attribute->taskFunc( currentTimeUs );
}
