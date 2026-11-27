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

// Receive IMU, motor RPM and servo angles, send motor and servo commands, using
// pi-protocol. (C) tblaha 2023, ported to Betaflight 2026 (FRD/NED).

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#if defined(HIL_BUILD) || defined(SIL_BUILD)

#if !defined(USE_FRD_NED)
#error "HIL_BUILD and SIL_BUILD exchange FRD body-frame data: build with USE_INDIFLIGHT"
#endif

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/servos.h"

#include "io/serial.h"
#include "pg/motor.h"
#include "rx/rx.h"
#include "common/sensor_alignment.h"
#include "sensors/boardalignment.h"
#include "telemetry/telemetry.h"

#include "io/hil.h"

#include "pi-protocol.h"
#include "pi-messages.h"

// The simulator sends FRD vectors in the board frame, as PyNDIflight's
// IMU(qBody=...) models it for both HIL and SIL. The IMU is assumed
// mounted on the board without rotation (USE_SIM_IDENTITY_SENSOR_ALIGN replaces the
// board config's GYRO_n_ALIGN by CW0_DEG), so the profile's align_board_* must
// match qBody, identically in SIL and HIL. The raw counts are FLU (the FLU -> FRD
// step follows the alignment, sensors/boardalignment.c).
void hilRawFromSensorFrd(const float frd[3], float raw[3])
{
    raw[0] = frd[0];
    raw[1] = -frd[1];
    raw[2] = -frd[2];
}

#endif // HIL_BUILD || SIL_BUILD

#if defined(HIL_BUILD)

#define HIL_INITIAL_PORT_MODE MODE_RXTX

// HIL_IN / HIL_OUT scaling (pi-protocol int16 fields)
#define HIL_TO_DEGS 0.1f
#define HIL_TO_G 0.001f
#define HIL_TO_RPM 10.f
#define HIL_TO_SERVO_CENTIDEGREES 1.f
#define MOTOR_TO_HIL 32767
#define SERVO_TO_HIL 32767

static serialPort_t *hilPort = NULL;
static const serialPortConfig_t *portConfig;
static bool hilEnabled = false;
static portSharing_e hilPortSharing;
static pi_parse_states_t p_hil;

// input from the simulation in natural units (deg/s, g, rpm, cdeg), body FRD
hilInput_t hilInput;

static void freeHilPort(void)
{
    closeSerialPort(hilPort);
    hilPort = NULL;
    hilEnabled = false;
}

static void configureHilPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }
    hilPort = openSerialPort(portConfig->identifier, FUNCTION_HIL, NULL, NULL, baudRates[baudRateIndex], HIL_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);
    hilEnabled = (hilPort != NULL);
}

void initHil(void)
{
    portConfig = findSerialPortConfig(FUNCTION_HIL);
    hilPortSharing = determinePortSharing(portConfig, FUNCTION_HIL);
}

void checkHilState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!hilEnabled && telemetrySharedPort != NULL) {
            hilPort = telemetrySharedPort;
            hilEnabled = true;
        }
    } else {
        const bool enable = telemetryDetermineEnabledState(hilPortSharing);
        if (enable == hilEnabled) {
            return;
        }
        if (enable) {
            configureHilPort();
        } else {
            freeHilPort();
        }
    }
}

static void serialWriter(uint8_t byte) { serialWrite(hilPort, byte); }

static int16_t motorToHil(int i)
{
    if (i >= getMotorCount()) {
        return 0;
    }
    return (int16_t)(MOTOR_TO_HIL * constrainf(scaleRangef(motor[i], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f));
}

static int16_t servoToHil(int i)
{
#ifdef USE_SERVOS
    return (i < MAX_SUPPORTED_SERVOS) ? (int16_t)(SERVO_TO_HIL * constrainf(servo_normalized[i], -1.f, 1.f)) : 0;
#else
    UNUSED(i);
    return 0;
#endif
}

void hilSendActuators(void)
{
#ifdef USE_SERVOS
    // measured servo angles come back from the simulation
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo_feedback[i] = lrintf(hilInput.servo_angle[i]);
    }
#endif

    if (!hilPort || !hilEnabled) {
        return;
    }

    piMsgHilOutTx.time_us = micros();
    piMsgHilOutTx.set_1 = motorToHil(0);
    piMsgHilOutTx.set_2 = motorToHil(1);
    piMsgHilOutTx.set_3 = motorToHil(2);
    piMsgHilOutTx.set_4 = motorToHil(3);
    piMsgHilOutTx.set_5 = motorToHil(4);
    piMsgHilOutTx.set_6 = motorToHil(5);
    piMsgHilOutTx.set_7 = motorToHil(6);
    piMsgHilOutTx.set_8 = motorToHil(7);
    piMsgHilOutTx.servo_1 = servoToHil(0);
    piMsgHilOutTx.servo_2 = servoToHil(1);
    piMsgHilOutTx.servo_3 = servoToHil(2);
    piMsgHilOutTx.servo_4 = servoToHil(3);
    piMsgHilOutTx.servo_5 = servoToHil(4);
    piMsgHilOutTx.servo_6 = servoToHil(5);
    piMsgHilOutTx.servo_7 = servoToHil(6);
    piMsgHilOutTx.servo_8 = servoToHil(7);
    piSendMsg(&piMsgHilOutTx, &serialWriter);
}

void handleHil(void)
{
    if (!hilEnabled || !hilPort) {
        return;
    }

    while (serialRxBytesWaiting(hilPort)) {
        piParse(&p_hil, serialRead(hilPort));
    }

    if (piMsgHilInRxState == PI_MSG_RX_STATE_NONE) {
        return;
    }

    hilInput.gyro[0] = HIL_TO_DEGS * piMsgHilInRx->gyro_x;
    hilInput.gyro[1] = HIL_TO_DEGS * piMsgHilInRx->gyro_y;
    hilInput.gyro[2] = HIL_TO_DEGS * piMsgHilInRx->gyro_z;
    hilInput.acc[0]  = HIL_TO_G * piMsgHilInRx->acc_x;
    hilInput.acc[1]  = HIL_TO_G * piMsgHilInRx->acc_y;
    hilInput.acc[2]  = HIL_TO_G * piMsgHilInRx->acc_z;

    const int16_t rpm[8] = { piMsgHilInRx->rpm_1, piMsgHilInRx->rpm_2, piMsgHilInRx->rpm_3, piMsgHilInRx->rpm_4,
                             piMsgHilInRx->rpm_5, piMsgHilInRx->rpm_6, piMsgHilInRx->rpm_7, piMsgHilInRx->rpm_8 };
    for (int i = 0; i < MIN(8, MAX_SUPPORTED_MOTORS); i++) {
        hilInput.rpm[i] = HIL_TO_RPM * rpm[i];
    }
    const int16_t servo[8] = { piMsgHilInRx->servo_1, piMsgHilInRx->servo_2, piMsgHilInRx->servo_3, piMsgHilInRx->servo_4,
                               piMsgHilInRx->servo_5, piMsgHilInRx->servo_6, piMsgHilInRx->servo_7, piMsgHilInRx->servo_8 };
    for (int i = 0; i < MIN(8, MAX_SUPPORTED_SERVOS); i++) {
        hilInput.servo_angle[i] = HIL_TO_SERVO_CENTIDEGREES * servo[i];
    }
}

static int16_t toInt16(float x)
{
    return (int16_t)constrain(lrintf(x), INT16_MIN, INT16_MAX);
}

void hilInjectGyroRaw(gyroDev_t *gyroDev)
{
    float raw[3];
    hilRawFromSensorFrd(hilInput.gyro, raw);
    for (int i = 0; i < 3; i++) {
        gyroDev->gyroADCRaw[i] = toInt16(raw[i] / gyroDev->scale);
    }
}

void hilInjectAccRaw(accDev_t *accDev)
{
    float raw[3];
    hilRawFromSensorFrd(hilInput.acc, raw);
    for (int i = 0; i < 3; i++) {
        accDev->ADCRaw[i] = toInt16(raw[i] * accDev->acc_1G);
    }
}

void hilGetDshotTelemetry(uint8_t motorIndex, uint32_t *value, dshotTelemetryType_t *type)
{
    // eRPM in units of ERPM_PER_LSB (100), from mechanical rpm and the pole count
    const float rpm = (motorIndex < MAX_SUPPORTED_MOTORS) ? fmaxf(hilInput.rpm[motorIndex], 0.f) : 0.f;
    *value = lrintf(rpm * (motorConfig()->motorPoleCount / 2.0f) / 100.0f); // ERPM_PER_LSB
    *type = DSHOT_TELEMETRY_TYPE_eRPM;
}

#endif // HIL_BUILD
