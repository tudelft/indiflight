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
#include "drivers/dshot.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/ahrs.h"
#include "flight/indi.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/local_pos.h"

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
    piMsgImuTx.time_us = (uint32_t) gyro.rawSensorDev->gyroLastEXTIUs;
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[2]);
    piMsgImuTx.x = GRAVITYf * acc.accADCafterRpm[0] * acc.dev.acc_1G_rec;
    piMsgImuTx.y = GRAVITYf * acc.accADCafterRpm[1] * acc.dev.acc_1G_rec;
    piMsgImuTx.z = GRAVITYf * acc.accADCafterRpm[2] * acc.dev.acc_1G_rec;

    if (piPort) {
        piSendMsg(&piMsgImuTx, &serialWriter);
    }
}

void piSendEkfInputs(void)
{
    piMsgEkfInputsTx.time_us = (uint32_t) gyro.rawSensorDev->gyroLastEXTIUs;
    piMsgEkfInputsTx.x = 2048.f * acc.accADCafterRpm[0] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.y = 2048.f * acc.accADCafterRpm[1] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.z = 2048.f * acc.accADCafterRpm[2] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.p = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[0] * 0.0005f );
    piMsgEkfInputsTx.q = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[1] * 0.0005f );
    piMsgEkfInputsTx.r = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[2] * 0.0005f );
#ifdef USE_DSHOT_TELEMETRY
    piMsgEkfInputsTx.omega1 = (int16_t) indiRun.omega[0];
    piMsgEkfInputsTx.omega2 = (int16_t) indiRun.omega[1];
    piMsgEkfInputsTx.omega3 = (int16_t) indiRun.omega[2];
    piMsgEkfInputsTx.omega4 = (int16_t) indiRun.omega[3];
#else
    piMsgEkfInputsTx.omega1 = 0;
    piMsgEkfInputsTx.omega2 = 0;
    piMsgEkfInputsTx.omega3 = 0;
    piMsgEkfInputsTx.omega4 = 0;
#endif

    if (piPort) {
        piSendMsg(&piMsgEkfInputsTx, &serialWriter);
    }
}

void piSendAux(void)
{
    piMsgAuxTx.time_us = micros();
    piMsgAuxTx.aux_1 = (int16_t) rcData[AUX1 + 0];
    piMsgAuxTx.aux_2 = (int16_t) rcData[AUX1 + 1];
    piMsgAuxTx.aux_3 = (int16_t) rcData[AUX1 + 2];
    piMsgAuxTx.aux_4 = (int16_t) rcData[AUX1 + 3];
    piMsgAuxTx.aux_5 = (int16_t) rcData[AUX1 + 4];
    piMsgAuxTx.aux_6 = (int16_t) rcData[AUX1 + 5];
    piMsgAuxTx.aux_7 = (int16_t) rcData[AUX1 + 6];
    piMsgAuxTx.aux_8 = (int16_t) rcData[AUX1 + 7];
    piMsgAuxTx.aux_9 = (int16_t) rcData[AUX1 + 8];
    piMsgAuxTx.aux_10 = (int16_t) rcData[AUX1 + 9];
    piMsgAuxTx.aux_11 = (int16_t) rcData[AUX1 + 10];
    piMsgAuxTx.aux_12 = (int16_t) rcData[AUX1 + 11];
    piMsgAuxTx.aux_13 = (int16_t) rcData[AUX1 + 12];
    piMsgAuxTx.aux_14 = (int16_t) rcData[AUX1 + 13];

    if (piPort) {
        piSendMsg(&piMsgAuxTx, &serialWriter);
    }
}

void processPiTelemetry(void)
{
    // handled event based now, whenever there is stuff to be send, those functions
    // call piSendEkfInputs, or similar. More boilerplate, but lower latency
    piSendAux();
}

static void processNewMessage(uint8_t msgId) {
    switch (msgId) {
#ifdef USE_LOCAL_POSITION
        case PI_MSG_EXTERNAL_POSE_ID: {
            local_pos_ned_t pos;
            pos.time_us = piMsgExternalPoseRx->time_us;
            pos.source = LOCAL_POS_SOURCE_PI;
            // process new message (should be NED)
            pos.pos.V.X = piMsgExternalPoseRx->ned_x;
            pos.pos.V.Y = piMsgExternalPoseRx->ned_y;
            pos.pos.V.Z = piMsgExternalPoseRx->ned_z;
            pos.vel.V.X = piMsgExternalPoseRx->ned_xd;
            pos.vel.V.Y = piMsgExternalPoseRx->ned_yd;
            pos.vel.V.Z = piMsgExternalPoseRx->ned_zd;
            // the quaternion x,y,z should be NED
            pos.quat.w = piMsgExternalPoseRx->body_qi;
            pos.quat.x = piMsgExternalPoseRx->body_qx;
            pos.quat.y = piMsgExternalPoseRx->body_qy;
            pos.quat.z = piMsgExternalPoseRx->body_qz;

            pos.vel_valid = true;
            pos.quat_valid = true;

            setLocalPosMeas(&pos);
            break;
        }
        case PI_MSG_POS_SETPOINT_ID: {
            local_pos_sp_ned_t sp;
            sp.time_us = piMsgPosSetpointRx->time_us;
            sp.source = LOCAL_POS_SOURCE_PI;
            sp.pos.V.X = piMsgPosSetpointRx->ned_x;
            sp.pos.V.Y = piMsgPosSetpointRx->ned_y;
            sp.pos.V.Z = piMsgPosSetpointRx->ned_z;
            sp.vel.V.X = piMsgPosSetpointRx->ned_xd;
            sp.vel.V.Y = piMsgPosSetpointRx->ned_yd;
            sp.vel.V.Z = piMsgPosSetpointRx->ned_zd;
            sp.psi = DEGREES_TO_RADIANS(piMsgPosSetpointRx->yaw);
            sp.trackPsi = true;
            setLocalPosSp(&sp);
            break;
        }
#endif
    }
}

pi_parse_states_t p_telem;

void processPiUplink(void)
{
    if (!piPort) {
        return;
    }

    while (serialRxBytesWaiting(piPort)) {
        uint8_t msgId = piParse(&p_telem, serialRead(piPort));
        if (msgId != PI_MSG_NONE_ID) {
            processNewMessage(msgId);
        }
    }
}

void handlePiTelemetry(void)
{
    if (!piTelemetryEnabled || !piPort) {
        return;
    }

    processPiTelemetry();
    processPiUplink();
}

#endif
