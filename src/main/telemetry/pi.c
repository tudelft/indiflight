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

#include "flight/neural_controllers/nn_controller.h"
#include "flight/neural_controllers/nn_debug.h"

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

void processPiTelemetry(void)
{
    // handled event based now, whenever there is stuff to be send, those functions
    // call piSendEkfInputs, or similar. More boilerplate, but lower latency
}

// Only the vision-conditioned controllers define NN_FEATURE_DIM (in their
// neural_network.h); the hover controller takes no companion features, so the
// NN_INPUT_CHUNK reassembly compiles out entirely.
#ifdef NN_FEATURE_DIM
// NN_INPUT_CHUNK carries NN_FEATURE_DIM floats split across 2 chunks of 32
// (see lib/main/pi-protocol/msgs/NN_INPUT_CHUNK.yaml for why: a single
// message can't fit all 64 floats under the 255 byte payload cap).
_Static_assert(NN_FEATURE_DIM == 64,
    "NN_INPUT_CHUNK reassembly assumes NN_FEATURE_DIM == 64 (2 chunks of 32 floats)");
static float nnInputChunkVec[NN_FEATURE_DIM];
static uint8_t nnInputChunkMask = 0;
#endif

static void processNewMessage(uint8_t msgId) {
    switch (msgId) {
#ifdef NN_FEATURE_DIM
        case PI_MSG_NN_INPUT_CHUNK_ID: {
            const pi_NN_INPUT_CHUNK_t *m = piMsgNnInputChunkRx;
            if (m->chunk_index < 2) {
                // copy by value: taking &m->f0 would be a misaligned packed-member
                // address (f0 sits at a non-4-byte-aligned offset), which the
                // firmware build's -Werror -Wextra rejects.
                float *dst = &nnInputChunkVec[m->chunk_index * 32];
                dst[0]  = m->f0;  dst[1]  = m->f1;  dst[2]  = m->f2;  dst[3]  = m->f3;
                dst[4]  = m->f4;  dst[5]  = m->f5;  dst[6]  = m->f6;  dst[7]  = m->f7;
                dst[8]  = m->f8;  dst[9]  = m->f9;  dst[10] = m->f10; dst[11] = m->f11;
                dst[12] = m->f12; dst[13] = m->f13; dst[14] = m->f14; dst[15] = m->f15;
                dst[16] = m->f16; dst[17] = m->f17; dst[18] = m->f18; dst[19] = m->f19;
                dst[20] = m->f20; dst[21] = m->f21; dst[22] = m->f22; dst[23] = m->f23;
                dst[24] = m->f24; dst[25] = m->f25; dst[26] = m->f26; dst[27] = m->f27;
                dst[28] = m->f28; dst[29] = m->f29; dst[30] = m->f30; dst[31] = m->f31;

                nnInputChunkMask = (uint8_t)(nnInputChunkMask | (1u << m->chunk_index));
                if (nnInputChunkMask == 0x3) {
                    nn_set_features(nnInputChunkVec);
                    nn_debug_on_features(nnInputChunkVec);
                    nnInputChunkMask = 0;
                }
            }
            break;
        }
#endif
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
