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
#include <string.h>

#include "platform.h"

#if defined(USE_ACTUATORS_T4)

#ifndef USE_SERVOS
#error "USE_ACTUATORS_T4 requires USE_SERVOS"
#endif

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
#include "flight/mixer_init.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"

#include "t4.h"
#include "io/t4_protocol.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

static serialPort_t *t4Port = NULL;
static const serialPortConfig_t *portConfig;

void freeActuatorsT4Port(void)
{
    closeSerialPort(t4Port);
    t4Port = NULL;
}

void initActuatorsT4(void)
{
    portConfig = findSerialPortConfig(FUNCTION_ACTUATORS_T4);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configureActuatorsT4Port(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }

    t4Port = openSerialPort(portConfig->identifier, FUNCTION_ACTUATORS_T4, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!t4Port) {
        return;
    }
}

void sendActuatorsT4(void)
{
    if (!t4Port) {
        return;
    }

    struct ActuatorsT4In in = { 0 }; 
    in.esc_arm = 0x00; // disarm all escs
    in.servo_arm = 0xFFFF; // arm all servos

    // todo: fix this hardcoding
    in.servo_1_cmd = (uint16_t) (motor_normalized[2] * 180.f * 100.f);
    in.servo_2_cmd = (uint16_t) (motor_normalized[3] * 180.f * 100.f);

    // write to serial port with checksum
    serialWrite(t4Port, START_BYTE_ACTUATORS_T4);
    uint8_t* stream = (uint8_t *) &in; // movable pointer into the packged struct
    in.checksum_in = 0;
    while ((uint8_t *) &in + sizeof(struct ActuatorsT4In) - 1 - stream) {
        in.checksum_in += *stream;
        serialWrite(t4Port, *stream);
        stream++;
    }
    serialWrite(t4Port, in.checksum_in);
}

// extern
struct ActuatorsT4Out t4_out = { 0 };

static struct ActuatorsT4Out t4_out_buf = { 0 };
static uint8_t* t4_out_buf_u8view;

void handleActuatorsT4(void)
{
    if (!t4Port) {
        return;
    }

    static uint8_t checksum;
    static t4_parse_state_t parser = T4_IDLE;
    while (serialRxBytesWaiting(t4Port)) {
        // read next byte
        uint8_t byte = serialRead(t4Port);
        switch (parser) {
            case T4_IDLE:
                if (byte == START_BYTE_ACTUATORS_T4) {
                    // likely start of frame, reset checksum and buffer pointer
                    checksum = 0;
                    t4_out_buf_u8view = (uint8_t*) &t4_out_buf;
                    // put parser in next state
                    parser = T4_STX_FOUND;
                }
                break;
            case T4_STX_FOUND:
                *(t4_out_buf_u8view++) = byte; // insert byte into the buffer
                checksum += byte;
                if ((uint8_t *) &t4_out_buf + sizeof(struct ActuatorsT4Out) - 1 - t4_out_buf_u8view == 0) {
                    // next byte is checksum
                    parser = T4_WAITING_FOR_CHECKSUM;
                }
                break;
            case T4_WAITING_FOR_CHECKSUM:
                *t4_out_buf_u8view = byte;
                if (byte == checksum) {
                    // success
                    memcpy(&t4_out, &t4_out_buf, sizeof(struct ActuatorsT4Out));

                    // hardcode for now
                    servo_feedback[0] = t4_out.servo_1_angle;
                    servo_feedback[1] = t4_out.servo_2_angle;
#ifdef USE_CLI_DEBUG_PRINT
                    static unsigned printCounter = 1;
                    if (printCounter++ % 100 == 0) {
                        printCounter = 1;
                        cliDebugPrintLinef("Servo position %d cdeg %d cdeg", servo_feedback[0], servo_feedback[2]);
                    }
#endif
                }
                parser = T4_IDLE;
                break;
        }
    }
}

#endif
