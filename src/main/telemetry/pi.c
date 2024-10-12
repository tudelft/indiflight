/*
 * Configure serial port to parse pi-messages and provide facilities to send
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY_PI)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/external_pos.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/pi.h"
#include "pi-protocol.h"
#include "pi-messages.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#define TELEMETRY_PI_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_PI_MAXRATE 50
#define TELEMETRY_PI_DELAY ((1000 * 1000) / TELEMETRY_PI_MAXRATE)

static serialPort_t *piPort = NULL;
static const serialPortConfig_t *portConfig;

static bool piTelemetryEnabled =  false;
static portSharing_e piPortSharing;

// wrapper for serialWrite
static void serialWriter(uint8_t byte) { serialWrite(piPort, byte); }

void freePiTelemetryPort(void)
{
    closeSerialPort(piPort);
    piPort = NULL;
    piTelemetryEnabled = false;
}

void initPiTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_PI);
    piPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_PI);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configurePiTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }

    piPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_PI, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_PI_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!piPort) {
        return;
    }

    piTelemetryEnabled = true;
}

void checkPiTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!piTelemetryEnabled && telemetrySharedPort != NULL) {
            piPort = telemetrySharedPort;
            piTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(piPortSharing);

        if (newTelemetryEnabledValue == piTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configurePiTelemetryPort();
        else
            freePiTelemetryPort();
    }
}

void piSendIMU(void)
{
    piMsgImuTx.time_us = micros();
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);
    piMsgImuTx.x = GRAVITYf * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.y = GRAVITYf * ((float)acc.accADC[1]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.z = GRAVITYf * ((float)acc.accADC[2]) / ((float)acc.dev.acc_1G);
    
    piSendMsg(&piMsgImuTx, &serialWriter);
}

#include "flight/indi.h"

void piSendEkfInputs(void)
{
    piMsgEkfInputsTx.time_us = (uint32_t) micros();
    piMsgEkfInputsTx.x = acc.accADC[0];
    piMsgEkfInputsTx.y = acc.accADC[1];
    piMsgEkfInputsTx.z = acc.accADC[2];
    piMsgEkfInputsTx.p = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCf[0] * 0.0005f );
    piMsgEkfInputsTx.q = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCf[1] * 0.0005f );
    piMsgEkfInputsTx.r = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCf[2] * 0.0005f );
    piMsgEkfInputsTx.omega1 = (int16_t) indiRun.omega[0];
    piMsgEkfInputsTx.omega2 = (int16_t) indiRun.omega[1];
    piMsgEkfInputsTx.omega3 = (int16_t) indiRun.omega[2];
    piMsgEkfInputsTx.omega4 = (int16_t) indiRun.omega[3];

    piSendMsg(&piMsgEkfInputsTx, &serialWriter);
}


void processPiTelemetry(void)
{
    static unsigned int senderCount = 0;
    if (senderCount++ % 2 == 0) {
        // rate limiting
        senderCount = 1;
        //piSendIMU();
        piSendEkfInputs();
    }
}

pi_parse_states_t p_telem;

void processPiUplink(void)
{
#ifdef PI_BETAFLIGHT_DEBUG
    static unsigned int i = 0;
    if (++i > 3) {
        i = 0;
        LED1_TOGGLE;
        cliPrintLinef("%10d", piStats[PI_PARSE_INVOKE]);
    }
#endif
    if (piPort) {
        while (serialRxBytesWaiting(piPort)) {
            uint8_t msgId = piParse(&p_telem, serialRead(piPort));
#if defined(USE_GPS_PI)
            if (msgId == PI_MSG_EXTERNAL_POSE_ID) {
                getExternalPos(0);
            }
#else
            UNUSED(msgId);
#endif
        }
    }
}

void handlePiTelemetry(void)
{
    if (!piTelemetryEnabled) {
        return;
    }

    if (!piPort) {
        return;
    }

    processPiTelemetry();
    processPiUplink();
}

#endif
