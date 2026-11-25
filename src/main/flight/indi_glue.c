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
#include "flight/servos.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#ifdef USE_EKF
#include "flight/ekf.h"
#endif
#ifdef USE_LOCAL_POSITION
#include "flight/pos_ctl.h"
#endif
#ifdef USE_GEOFENCE
#include "flight/geofence.h"
#endif
#ifdef USE_PITOT
#include "sensors/pitotmeter.h"
#endif

static FAST_DATA_ZERO_INIT float motor_normalized[MAX_SUPPORTED_MOTORS];

// normalized [0, 1] motor command INDI sends to the ESCs (for MOCKUP / HIL tooling)
float indiGetMotorNormalized(int i)
{
    return (i >= 0 && i < MAX_SUPPORTED_MOTORS) ? motor_normalized[i] : 0.f;
}

void indiflightInit(void)
{
#ifdef USE_ACC_IN_GYRO_TASK
    // acc is processed once per PID loop: acc_lpf and jerk run at that rate
    if (gyro.targetLooptime) {
        acc.sampleRateHz = 1e6f / gyro.targetLooptime;
        accInitFilters();
    }
#endif
    initIndiRuntime();
#ifdef USE_LOCAL_POSITION
    posCtlInit();
#endif
#ifdef USE_GEOFENCE
    geofenceInit();
#endif
#ifdef USE_PITOT
    pitotInit();
    pitotStartCalibration();
#endif
}

// runs in the PID task before the controller, so INDI sees this loop's estimate.
// TODO(spike): indiflight runs the EKF as its own REALTIME task (IMU->FILTER->EKF->AHRS->INNER LOOP)
FAST_DATA_ZERO_INIT float indiAccAfterRpm[3];

static float *indiServoState(void)
{
#ifdef USE_SERVO_FEEDBACK
    static FAST_DATA_ZERO_INIT float servoNormalizedFeedback[MAX_SUPPORTED_SERVOS];
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoNormalizedFeedback[i] = constrainf(((float)servo_feedback[i]) * 1e-4f, -1.f, 1.f);
    }
    return servoNormalizedFeedback;
#else
    return servo_normalized;
#endif
}

// acc input of INDI at PID rate: acc sampled in the gyro task and averaged over
// the PID period (USE_ACC_IN_GYRO_TASK), before acc_lpf, then the gyro's rpm notches
static FAST_CODE void indiAccFiltering(void)
{
    for (int axis = 0; axis < 3; axis++) {
#ifdef USE_ACC_IN_GYRO_TASK
        indiAccAfterRpm[axis] = acc.accADCunfiltered.v[axis];
#else
        indiAccAfterRpm[axis] = acc.accADC.v[axis];
#endif
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
    return INDIFLIGHT_MODE(INDI_MODE);
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
        if ((indiRun.actType[i] == INDI_ACT_TYPE_SERVO) && (s < MAX_SUPPORTED_SERVOS)) {
            servo_normalized[s++] = constrainf(indiRun.d[i], -1.f, 1.f);
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo_normalized[i] = 0.f;
        }
    }

    // INDI's actuator state uses the measured servo angles when a driver reports them
    indiUpdateActuatorState(motor_normalized, indiServoState());

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
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo_normalized[i] = 0.f; // PID path does not command the indiflight servo drivers
    }
    indiUpdateActuatorState(motor_normalized, indiServoState());
    indiUpdateRpmCtlState(motor_normalized);
}

#endif // USE_INDI
