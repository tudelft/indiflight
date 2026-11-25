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

static FAST_DATA_ZERO_INIT float motor_normalized[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT float servo_normalized[MAX_SUPPORTED_MOTORS];

void indiflightInit(void)
{
    initIndiRuntime();
}

void indiflightProcessRxModes(void)
{
    // legacy PIDs. motor_output_limit only applies to PID
    if (IS_RC_MODE_ACTIVE(BOXPIDCTL)) {
        if (!FLIGHT_MODE(PID_MODE)) {
            ENABLE_FLIGHT_MODE(PID_MODE);
            initEscEndpoints();
        }
    } else if (FLIGHT_MODE(PID_MODE)) {
        DISABLE_FLIGHT_MODE(PID_MODE);
        initEscEndpoints();
    }

#ifdef USE_LOCAL_POSITION
    // TODO(spike): port indiflight's edge handling (EKF convergence gate,
    // ARMING_DISABLED_EKF_OR_SETPOINT, waypoint rising edge/auto-off)
    if (IS_RC_MODE_ACTIVE(BOXPOSCTL)) {
        ENABLE_FLIGHT_MODE(POSITION_MODE);
    } else {
        DISABLE_FLIGHT_MODE(POSITION_MODE);
    }
    if (IS_RC_MODE_ACTIVE(BOXWAYPOINTS) && FLIGHT_MODE(POSITION_MODE)) {
        ENABLE_FLIGHT_MODE(WAYPOINT_MODE);
    } else {
        DISABLE_FLIGHT_MODE(WAYPOINT_MODE);
    }
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
    return !FLIGHT_MODE(PID_MODE);
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
            motor_normalized[i] = constrainf(biquadFilterApply(&indiRun.motorLeadLagFilter[i], motor_normalized[i]), 0.f, 1.f);
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

#endif // USE_INDI
