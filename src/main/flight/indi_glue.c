/*
 * This file is part of Indiflight.
 *
 * See indi_glue.h.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include "platform.h"

#ifdef USE_INDI

#include "common/maths.h"
#include "common/filter.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/mixer_init.h"

#include "flight/indi.h"
#include "flight/indi_init.h"
#include "flight/indi_glue.h"
#include "flight/rpm_filter.h"
#include "sensors/acceleration.h"
#ifdef USE_EKF
#include "flight/ekf.h"
#endif
#ifdef USE_LOCAL_POSITION
#include "flight/pos_ctl.h"
#endif

static FAST_DATA_ZERO_INIT float motor_normalized[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT float servo_normalized[MAX_SUPPORTED_MOTORS];

void indiflightInit(void)
{
    initIndiRuntime();
#ifdef USE_LOCAL_POSITION
    posCtlInit();
#endif
}

// runs in the PID task before the controller, so INDI sees this loop's estimate.
// TODO(spike): indiflight runs the EKF as its own REALTIME task (IMU->FILTER->EKF->AHRS->INNER LOOP)
FAST_DATA_ZERO_INIT float indiAccAfterRpm[3];

// acc pipeline for INDI at PID rate. The fork read acc in the gyro task and
// averaged it over the PID period; upstream reads acc in TASK_ACCEL, so this
// holds the latest sample (TODO: acc sampling in the gyro task needs sensors/).
static FAST_CODE void indiAccFiltering(void)
{
    for (int axis = 0; axis < 3; axis++) {
        indiAccAfterRpm[axis] = acc.accADC.v[axis];
    }
#ifdef USE_RPM_FILTER
    rpmFilterRunAcc(indiAccAfterRpm);
#endif
}

FAST_CODE void indiflightPreController(timeUs_t currentTimeUs)
{
    indiAccFiltering();
#ifdef USE_EKF
    updateEkf(currentTimeUs);
#else
    UNUSED(currentTimeUs);
#endif
}

// stick activity detection (was in indiflight fc/rc.c)
static float sticksReference[4];

void setSticksReference(void)
{
    for (int i = 0; i < 3; i++) {
        sticksReference[i] = getRcDeflection(i);
    }
    sticksReference[THROTTLE] = 1e-3f * (rcCommand[THROTTLE] - 1000.f); // [0, 1]
}

bool haveSticksMoved(void)
{
    for (int i = 0; i < 3; i++) {
        if (fabsf(getRcDeflection(i) - sticksReference[i]) > RC_STICK_MOVEMENT_THRESHOLD) {
            return true;
        }
    }
    return fabsf(1e-3f * (rcCommand[THROTTLE] - 1000.f) - sticksReference[THROTTLE]) > RC_STICK_MOVEMENT_THRESHOLD;
}

bool indiIsActive(void)
{
    return FLIGHT_MODE(INDI_MODE);
}

// INDI output -> upstream motor[] (motor protocol units). Replaces mixTable().
// TODO(spike): servos (servo_normalized -> upstream servo[]), throw/learner
// overrides, failsafe/motor-stop handling that mixTable() does for PID.
FAST_CODE void indiMixTable(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    const int numMotors = mixerRuntime.motorCount;

    int m = 0;
    int s = 0;
    for (int i = 0; i < indiRun.actNum; i++) {
        if ((indiRun.actType[i] == INDI_ACT_TYPE_MOTOR) && (m < numMotors)) {
            motor_normalized[m++] = constrainf(indiRun.d[i], 0.f, 1.f);
        }
        if ((indiRun.actType[i] == INDI_ACT_TYPE_SERVO) && (s < MAX_SUPPORTED_MOTORS)) {
            servo_normalized[s++] = constrainf(indiRun.d[i], -1.f, 1.f);
        }
    }

    // TODO(spike): servo feedback instead of commands once a feedback driver exists
    indiUpdateActuatorState(motor_normalized, servo_normalized);

    if (indiRun.useMotorLeadLag) {
        for (int i = 0; i < numMotors; i++) {
            motor_normalized[i] = constrainf(phaseCompApply(&indiRun.motorLeadLagFilter[i], motor_normalized[i]), 0.f, 1.f);
        }
    }

    // INDI rpm loop must see the final commands that go to the ESCs
    indiUpdateRpmCtlState(motor_normalized);

    for (int i = 0; i < numMotors; i++) {
        motor[i] = ARMING_FLAG(ARMED)
            ? scaleRangef(motor_normalized[i], 0.f, 1.f, mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh)
            : motor_disarmed[i];
    }
}

// PID path: feed the upstream motor outputs back into INDI's actuator state
FAST_CODE void indiTrackMotorOutput(void)
{
    const int numMotors = mixerRuntime.motorCount;
    for (int i = 0; i < numMotors; i++) {
        motor_normalized[i] = constrainf(scaleRangef(motor[i], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f);
    }
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        servo_normalized[i] = 0.f; // TODO(spike): upstream servo[] -> normalized
    }
    indiUpdateActuatorState(motor_normalized, servo_normalized);
    indiUpdateRpmCtlState(motor_normalized);
}

#endif // USE_INDI
