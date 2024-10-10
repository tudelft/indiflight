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

/*
void piSendIMU(void)
{
    // is never run now
    piMsgImuTx.time_us = micros();
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);
    piMsgImuTx.x = GRAVITYf * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.y = GRAVITYf * ((float)acc.accADC[1]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.z = GRAVITYf * ((float)acc.accADC[2]) / ((float)acc.dev.acc_1G);
    piSendMsg(&piMsgImuTx, &serialWriter);
}
*/

void piSendEkfInputs(timeUs_t currentTimeUs, fp_vector_t* g, fp_vector_t* a, float* omega)
{
    piMsgEkfInputsTx.time_us = (uint32_t) currentTimeUs;
    piMsgEkfInputsTx.p = (int16_t) ( ((float) ((1 << 15) - 1)) * RADIANS_TO_DEGREES(g->V.X) * 0.0005f);
    piMsgEkfInputsTx.q = (int16_t) ( ((float) ((1 << 15) - 1)) * RADIANS_TO_DEGREES(g->V.Y) * 0.0005f);
    piMsgEkfInputsTx.r = (int16_t) ( ((float) ((1 << 15) - 1)) * RADIANS_TO_DEGREES(g->V.Z) * 0.0005f);
    piMsgEkfInputsTx.x = (int16_t) ( ((float) acc.dev.acc_1G) / GRAVITYf * a->V.X );
    piMsgEkfInputsTx.y = (int16_t) ( ((float) acc.dev.acc_1G) / GRAVITYf * a->V.Y );
    piMsgEkfInputsTx.z = (int16_t) ( ((float) acc.dev.acc_1G) / GRAVITYf * a->V.Z );
    piMsgEkfInputsTx.omega1 = (int16_t) omega[0];
    piMsgEkfInputsTx.omega2 = (int16_t) omega[1];
    piMsgEkfInputsTx.omega3 = (int16_t) omega[2];
    piMsgEkfInputsTx.omega4 = (int16_t) omega[3];

    piSendMsg(&piMsgEkfInputsTx, &serialWriter);
}

void processPiTelemetry(void)
{
    // could do rate limiting with the stream stuffs above
    //piSendIMU();

    // dont send here, send when we actually receive/use the data
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
#if (!defined(USE_GPS_PI) && !defined(USE_EKF))
            UNUSED(msgId);
#endif

#ifdef USE_GPS_PI
            if (msgId == PI_MSG_EXTERNAL_POSE_ID) {
                getExternalPos(0);
            }
#endif
#ifdef USE_EKF
            if (msgId == PI_MSG_OFFBOARD_POSE_ID) {
                // todo: make this more robust to signal loss
                offboardPosNed.time_us = piMsgOffboardPoseRx->time_us;
                offboardPosNed.pos.V.X = piMsgOffboardPoseRx->x;
                offboardPosNed.pos.V.Y = piMsgOffboardPoseRx->y;
                offboardPosNed.pos.V.Z = piMsgOffboardPoseRx->z;
                offboardPosNed.quat.w = piMsgOffboardPoseRx->qw;
                offboardPosNed.quat.x = piMsgOffboardPoseRx->qx;
                offboardPosNed.quat.y = piMsgOffboardPoseRx->qy;
                offboardPosNed.quat.z = piMsgOffboardPoseRx->qz;
                offboardLatestMsgTime = offboardPosNed.time_us;
            }
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
