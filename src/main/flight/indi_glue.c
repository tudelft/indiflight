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
#include "io/beeper.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "flight/imu.h"
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

// normalized [0, 1] motor command INDI sends to the ESCs (for SIL / HIL tooling)
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
// the PID period (USE_ACC_IN_GYRO_TASK), rpm-notched in the filter task if
// acc_rpm_filter is set, before acc_lpf
static FAST_CODE void indiAccFiltering(void)
{
    for (int axis = 0; axis < 3; axis++) {
#ifdef USE_ACC_IN_GYRO_TASK
        indiAccAfterRpm[axis] = acc.accADCunfiltered.v[axis];
#else
        indiAccAfterRpm[axis] = acc.accADC.v[axis];
#endif
    }
#if !defined(USE_ACC_IN_GYRO_TASK) && defined(USE_RPM_FILTER)
    if (indiConfig()->acc_rpm_filter) {
        rpmFilterRunAcc(indiAccAfterRpm);
    }
#endif
}

// attitude estimation in the PID loop, every imu_process_denom gyro samples, after
// the EKF (as the 4.4 fork's TASK_AHRS in the realtime loop); TASK_ATTITUDE is off
static FAST_CODE void indiImuUpdate(timeUs_t currentTimeUs)
{
    static FAST_DATA_ZERO_INIT uint32_t gyroSamplesSinceImu;
    const uint32_t gyroSamplesPerPidLoop = (gyro.sampleLooptime > 0) ? MAX(1, lrintf((float)gyro.targetLooptime / gyro.sampleLooptime)) : 1;
    gyroSamplesSinceImu += gyroSamplesPerPidLoop;
    if (gyroSamplesSinceImu >= imuConfig()->imu_process_denom) {
        gyroSamplesSinceImu = 0;
        imuUpdateAttitude(currentTimeUs);
    }
}

FAST_CODE void indiflightPreController(timeUs_t currentTimeUs)
{
    indiAccFiltering();
#ifdef USE_EKF
    updateEkf(currentTimeUs);
#endif
    indiImuUpdate(currentTimeUs);
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

#ifdef USE_ACTUATOR_TEST_DANGEROUS
#pragma message "WARNING: compiling with dangerous code (USE_ACTUATOR_TEST_DANGEROUS)."
// indiflight motor test (as in the 4.4 fork): after arming and a grace period,
// motor 0 runs a fixed sequence of sines and steps, the other motors idle. Used
// by the CineRatMotorTest profile on a test stand. Overrides PID and INDI.
#define ACTUATOR_TEST_GRACE_PERIOD_US 2000000

// Sine input settings
#define ACTUATOR_TEST_N_AMPLITUDES 3
#define ACTUATOR_TEST_N_FREQUENCIES 3
#define ACTUATOR_TEST_SIN_US 3000000 // 3 sec
static const float actuatorTestAmplitudes[ACTUATOR_TEST_N_AMPLITUDES] = { 0.1f, 0.2f, 0.3f }; // [-]
static const float actuatorTestFrequencies[ACTUATOR_TEST_N_FREQUENCIES] = { 0.5f, 1.f, 2.f }; // [Hz]

// Step input settings
#define ACTUATOR_TEST_N_STEPS 3
#define ACTUATOR_TEST_STEP_US 2000000 // 2 sec
static const float actuatorTestStepAmplitudes[ACTUATOR_TEST_N_STEPS] = { 0.25f, 0.5f, 0.75f };

// Other settings
#define ACTUATOR_TEST_BREAK_US 5000000 // 5 sec
#define ACTUATOR_TEST_IDLE_OUTPUT 0.1f // 10% throttle

static float actuatorTestSignal(timeDelta_t timeSinceStartUs)
{
    timeDelta_t totalUs = 0;

    for (int i = 0; i < ACTUATOR_TEST_N_AMPLITUDES; i++) {
        for (int j = 0; j < ACTUATOR_TEST_N_FREQUENCIES; j++) {
            if (totalUs <= timeSinceStartUs && timeSinceStartUs < totalUs + ACTUATOR_TEST_BREAK_US) {
                return ACTUATOR_TEST_IDLE_OUTPUT;
            }
            totalUs += ACTUATOR_TEST_BREAK_US;

            if (totalUs <= timeSinceStartUs && timeSinceStartUs < totalUs + ACTUATOR_TEST_SIN_US) {
                const float periodUs = 1000000.f / actuatorTestFrequencies[j];
                const float phase = 2.0f * M_PIf * ((float)(timeSinceStartUs - totalUs)) / periodUs;
                return 0.5f + actuatorTestAmplitudes[i] * sinf(phase);
            }
            totalUs += ACTUATOR_TEST_SIN_US;
        }
    }

    for (int i = 0; i < ACTUATOR_TEST_N_STEPS; i++) {
        if (totalUs <= timeSinceStartUs && timeSinceStartUs < totalUs + ACTUATOR_TEST_BREAK_US) {
            return ACTUATOR_TEST_IDLE_OUTPUT;
        }
        totalUs += ACTUATOR_TEST_BREAK_US;

        if (totalUs <= timeSinceStartUs && timeSinceStartUs < totalUs + ACTUATOR_TEST_STEP_US) {
            return actuatorTestStepAmplitudes[i];
        }
        totalUs += ACTUATOR_TEST_STEP_US;
    }
    return ACTUATOR_TEST_IDLE_OUTPUT;
}

// overwrite motor_normalized[] with the test sequence
static void actuatorTestOverride(timeUs_t currentTimeUs, int numMotors)
{
    for (int i = 0; i < numMotors; i++) {
        motor_normalized[i] = 0.f;
    }

    static timeUs_t lastArmedUs = 0;
    static bool lastArmed = false;
    if (!lastArmed && ARMING_FLAG(ARMED)) {
        lastArmedUs = currentTimeUs;
        beeperWarningBeeps(15); // warn user
    }
    lastArmed = ARMING_FLAG(ARMED);
    const timeDelta_t timeSinceArmUs = cmpTimeUs(currentTimeUs, lastArmedUs);

    if (timeSinceArmUs >= ACTUATOR_TEST_GRACE_PERIOD_US) {
        motor_normalized[0] = constrainf(actuatorTestSignal(timeSinceArmUs - ACTUATOR_TEST_GRACE_PERIOD_US), 0.f, 1.f);
    }
}
#endif // USE_ACTUATOR_TEST_DANGEROUS

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

#ifdef USE_ACTUATOR_TEST_DANGEROUS
    actuatorTestOverride(currentTimeUs, numMotors);
#endif

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
FAST_CODE void indiTrackMotorOutput(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    const int numMotors = mixerRuntime.motorCount;
    for (int i = 0; i < numMotors; i++) {
        motor_normalized[i] = constrainf(scaleRangef(motor[i], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f);
    }
#ifdef USE_ACTUATOR_TEST_DANGEROUS
    // the test sequence also overrides the PID outputs
    actuatorTestOverride(currentTimeUs, numMotors);
    for (int i = 0; i < numMotors; i++) {
        motor[i] = ARMING_FLAG(ARMED)
            ? scaleRangef(motor_normalized[i], 0.f, 1.f, mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh)
            : motor_disarmed[i];
    }
#endif
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo_normalized[i] = 0.f; // PID path does not command the indiflight servo drivers
    }
    indiUpdateActuatorState(motor_normalized, indiServoState());
    indiUpdateRpmCtlState(motor_normalized);
}

#endif // USE_INDI
