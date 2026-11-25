/*
 * This file is part of Indiflight.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Indiflight is distributed in the hope that they will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// indiflight MOCKUP: the SIMULATOR platform (sitl.c with -DMOCKUP) built as a
// shared library and stepped by the caller, see src/utils/indiflight_mockup_interface.py
// and support/simulation/PyNDIflight. All vectors are body FRD / earth NED, SI units.
// Ported from the 4.4 fork's src/main/mockup/main.c.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef MOCKUP

#if !defined(USE_LOCAL_POSITION) || !defined(USE_ACC) || !defined(USE_INDI)
#error "MOCKUP requires USE_INDI, USE_LOCAL_POSITION and USE_ACC"
#endif

#include "common/maths.h"
#include "common/bitarray.h"
#include "common/utils.h"

#include "blackbox/blackbox.h"
#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/time.h"
#include "fc/core.h"
#include "fc/init.h"
#include "drivers/system.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/tasks.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "io/hil.h"
#include "io/local_pos.h"
#include "flight/ekf.h"
#include "flight/indi.h"
#include "flight/indi_glue.h"
#include "msp/msp_box.h"
#include <string.h>
#include "pg/motor.h"
#include "scheduler/scheduler.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define MOCKUP_TICK_DT_US 125   // one tick = one gyro/PID loop at 8 kHz
#define MOCKUP_GRAVITY 9.80665f

void mockupClockTick(uint32_t us); // sitl.c

static float motorOmega[MAX_SUPPORTED_MOTORS]; // rad/s, from the simulation

static int16_t toInt16(float x)
{
    return (int16_t)constrain(lrintf(x), INT16_MIN, INT16_MAX);
}

// Gyro in rad/s, specific force in N/kg, in the FRD *sensor* frame, as in the
// 4.4 fork: the firmware applies the sensor and board alignment to them, so the
// sim's IMU(qBody=...) must match the alignment settings (identity by default).
// The virtual sensor reports FLU counts (Y, Z negated), which the FLU -> FRD
// step after the alignment (sensors/boardalignment.c) turns back into FRD.
void setImu(const float *g, const float *a)
{
    static const float fluFromFrd[3] = { 1.f, -1.f, -1.f };
    if (virtualGyroDev) {
        const float s = 1.0f / virtualGyroDev->scale;
        virtualGyroSet(virtualGyroDev,
            toInt16(fluFromFrd[0] * RADIANS_TO_DEGREES(g[0]) * s),
            toInt16(fluFromFrd[1] * RADIANS_TO_DEGREES(g[1]) * s),
            toInt16(fluFromFrd[2] * RADIANS_TO_DEGREES(g[2]) * s));
    }
    if (virtualAccDev) {
        const float s = virtualAccDev->acc_1G / MOCKUP_GRAVITY;
        virtualAccSet(virtualAccDev, toInt16(fluFromFrd[0] * a[0] * s), toInt16(fluFromFrd[1] * a[1] * s), toInt16(fluFromFrd[2] * a[2] * s));
    }
}

void setMotorSpeed(const float *omega, const int n)
{
    for (int m = 0; m < MIN(n, MAX_SUPPORTED_MOTORS); m++) {
        motorOmega[m] = omega[m];
    }
}

void setServoAngle(const float *angles, const int n)
{
#ifdef USE_SERVOS
    for (int s = 0; s < MIN(n, MAX_SUPPORTED_SERVOS); s++) {
        servo_feedback[s] = (int16_t)(100.f * RADIANS_TO_DEGREES(angles[s])); // centidegrees
    }
#else
    UNUSED(angles);
    UNUSED(n);
#endif
}

// position (m), velocity (m/s), NED; attitude quaternion body FRD -> NED (w, x, y, z)
void setMocapT(const float *pos, const float *vel, const float *q, const uint32_t time_us)
{
    local_pos_ned_t p;
    p.source = LOCAL_POS_SOURCE_MOCKUP;
    p.time_us = time_us;
    for (int i = 0; i < 3; i++) {
        p.pos.A[i] = pos[i];
        p.vel.A[i] = vel[i];
    }
    p.quat.w = q[0];
    p.quat.x = q[1];
    p.quat.y = q[2];
    p.quat.z = q[3];
    p.vel_valid = true;
    p.quat_valid = true;
    setLocalPosMeas(&p);
}

void setMocap(const float *pos, const float *vel, const float *q)
{
    setMocapT(pos, vel, q, micros());
}

// position setpoint (m, NED) and heading (rad)
void setPosSetpoint(const float *pos, const float yaw)
{
    local_pos_sp_ned_t sp;
    sp.source = LOCAL_POS_SOURCE_MOCKUP;
    sp.time_us = micros();
    for (int i = 0; i < 3; i++) {
        sp.pos.A[i] = pos[i];
    }
    sp.psi = yaw;
    sp.trackPsi = true;
    setLocalPosSp(&sp);
}

// normalized motor commands [0, 1] (0 while disarmed)
void getMotorOutputCommands(float *cmd, int n)
{
    for (int m = 0; m < n; m++) {
        cmd[m] = 0.f;
        if (m < getMotorCount() && ARMING_FLAG(ARMED)) {
            // INDI writes motor_normalized[] (as in the 4.4 fork), the PID mixer only motor[]
            cmd[m] = FLIGHT_MODE(INDI_MODE) ? indiGetMotorNormalized(m)
                : constrainf(scaleRangef(motor[m], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f);
        }
    }
}

void getServoOutputCommands(float *cmd, int n)
{
    for (int s = 0; s < n; s++) {
#ifdef USE_SERVOS
        cmd[s] = (s < MAX_SUPPORTED_SERVOS) ? servo_normalized[s] : 0.f;
#else
        cmd[s] = 0.f;
#endif
    }
}

// Flight modes and RX boxes. All mode access goes through these two helpers
// (and the matching one in indiflight_mockup_interface.py), so moving the
// indiflight flags into their own word only needs a change here.
void mockupSetFlightModes(uint32_t mask, bool enable)
{
    if (enable) {
        ENABLE_FLIGHT_MODE((flightModeFlags_e)mask);
    } else {
        DISABLE_FLIGHT_MODE((flightModeFlags_e)mask);
    }
}

uint32_t mockupGetFlightModes(void)
{
    return flightModeFlags;
}

extern boxBitmask_t rcModeActivationMask; // fc/rc_modes.c

void mockupSetRxBox(uint32_t boxId, bool enable)
{
    if (boxId >= CHECKBOX_ITEM_COUNT) {
        return;
    }
    if (enable) {
        bitArraySet(&rcModeActivationMask, boxId);
    } else {
        bitArrayClr(&rcModeActivationMask, boxId);
    }
}

// RX box by its configurator name ("ARM", "PREARM", "INDI", ...), returns false if unknown
bool mockupSetRxBoxByName(const char *name, bool enable)
{
    for (unsigned i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        const box_t *box = findBoxByBoxId(i);
        if (box && strcmp(box->boxName, name) == 0) {
            mockupSetRxBox(i, enable);
            return true;
        }
    }
    return false;
}

void mockupArm(bool arm)
{
    if (arm) {
        ENABLE_ARMING_FLAG(ARMED);
    } else {
        disarm(DISARM_REASON_SWITCH);
    }
}

void mockupBlackboxUpdate(void)
{
#ifdef USE_BLACKBOX
    blackboxUpdate(micros());
#endif
}

// state for inspection from the caller: EKF position (3) and velocity (3, NED),
// position setpoint (3, NED), INDI specific force setpoint (3, body FRD),
// EKF in use / converged (2), INDI d[0], acc.accADC.z (counts), INDI measured specific force z (N/kg),
// INDI u[0] and omega[0] (rad/s) (5), filtered gyro (3, deg/s)
int mockupGetState(float *out, int n)
{
    float v[22];
    int k = 0;
    for (int i = 0; i < 3; i++) { v[k++] = posEstNed.A[i]; }
    for (int i = 0; i < 3; i++) { v[k++] = velEstNed.A[i]; }
    for (int i = 0; i < 3; i++) { v[k++] = posSpNed.pos.A[i]; }
    for (int i = 0; i < 3; i++) { v[k++] = indiRun.spfSpBody.A[i]; }
    v[k++] = shouldBeUsedEkf();
    v[k++] = isConvergedEkf();
    v[k++] = indiRun.d[0];
    v[k++] = acc.accADC.z;
    v[k++] = indiRun.spfIMU.V.Z;
    v[k++] = indiRun.u[0];
    v[k++] = indiRun.omega[0];
    for (int i = 0; i < 3; i++) { v[k++] = gyro.gyroADCf[i]; } // deg/s, body FRD
    for (int i = 0; i < MIN(n, k); i++) {
        out[i] = v[i];
    }
    return k;
}

void mockupShutdown(void)
{
    // nothing to release: no threads or sockets in MOCKUP (sitl.c)
}

#if !defined(USE_DSHOT)
// motor rpm "telemetry" for INDI from the simulation (no dshot in the SIMULATOR platform)
bool isDshotTelemetryActive(void)
{
    return true;
}

uint16_t getDshotErpm(uint8_t motorIndex)
{
    if (motorIndex >= MAX_SUPPORTED_MOTORS) {
        return 0;
    }
    const float rpm = fmaxf(motorOmega[motorIndex], 0.f) * 60.f / (2.f * M_PIf);
    return (uint16_t)lrintf(rpm * (motorConfig()->motorPoleCount / 2.0f) / 100.0f); // ERPM_PER_LSB
}
#endif

// what main() does before run() (src/main/main.c), without the scheduler loop
void init(void)
{
    systemInit();
    initPhase1();
    initPhase2();
    initPhase3();
}

static void runTask(taskId_e id, timeUs_t now)
{
    getTask(id)->attribute->taskFunc(now);
}

// one 8 kHz gyro loop: gyro, filter, acc, and by division attitude, position control and
// the PID/INDI loop (incl. EKF)
void tick(void)
{
    static uint32_t n = 0;
    mockupClockTick(MOCKUP_TICK_DT_US);
    const timeUs_t now = micros();

    // the caller arms/disarms and sets the modes directly; no RX, no arming checks
    unsetArmingDisabled(0xffffffff);

    runTask(TASK_GYRO, now);
    runTask(TASK_FILTER, now);
    runTask(TASK_ACCEL, now);
    if (n % 8 == 0) {   // 1 kHz
        runTask(TASK_ATTITUDE, now);
    }
#ifdef USE_POS_CTL
    if (n % 16 == 0) {  // 500 Hz
        runTask(TASK_POS_CTL, now);
    }
#endif
    const unsigned pidDenom = MAX(1, pidConfig()->pid_process_denom);
    if (n % pidDenom == 0) { // the PID/INDI loop runs at gyro rate / pid_process_denom
        runTask(TASK_PID, now);
    }
    n++;
}

#endif // MOCKUP
