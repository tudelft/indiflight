/*
 * Configure serial port to interface with teensy actuator board https://github.com/tudelft/t4_actuators_board/
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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

#if defined(USE_ACTUATORS_SERVO)

#ifndef USE_SERVOS
#error "USE_ACTUATORS_SERVO requires USE_SERVOS"
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
#include "flight/ahrs.h"
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

#include "servo.h"
#include "io/servo_protocol.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

static serialPort_t *ServoPort = NULL;
static const serialPortConfig_t *portConfig;

void freeActuatorsServoPort(void)
{
    closeSerialPort(ServoPort);
    ServoPort = NULL;
}

void initActuatorsServo(void)
{
    portConfig = findSerialPortConfig(FUNCTION_ACTUATORS_SERVO);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configureActuatorsServoPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }

    ServoPort = openSerialPort(portConfig->identifier, FUNCTION_ACTUATORS_SERVO, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!ServoPort) {
        return;
    }
}

void sendActuatorsServo(void)
{
    if (!ServoPort) {
        return;
    }

    struct ActuatorsServoIn in = { 0 }; 
    in.esc_arm = 0x00; // disarm all escs
    in.servo_arm = 0xFFFF; // arm all servos

    // todo: fix this hardcoding
    in.servo_1_cmd = (int16_t) (servo_normalized[0] * 100.f * 100.f);
    in.servo_2_cmd = (int16_t) (-servo_normalized[1] * 100.f * 100.f);
    in.servo_3_cmd = (int16_t) (servo_normalized[2] * 100.f * 100.f);
    in.servo_4_cmd = (int16_t) (servo_normalized[3] * 100.f * 100.f);

    // write to serial port with checksum
    serialWrite(ServoPort, START_BYTE_ACTUATORS_SERVO);
    uint8_t* stream = (uint8_t *) &in; // movable pointer into the packged struct
    in.checksum_in = 0;
    while ((uint8_t *) &in + sizeof(struct ActuatorsServoIn) - 1 - stream) {
        in.checksum_in += *stream;
        serialWrite(ServoPort, *stream);
        stream++;
    }
    serialWrite(ServoPort, in.checksum_in);
}

// extern
struct ActuatorsServoOut Servo_out = { 0 };

static struct ActuatorsServoOut Servo_out_buf = { 0 };
static uint8_t* Servo_out_buf_u8view;

void handleActuatorsServo(void)
{
    if (!ServoPort) {
        return;
    }

    static uint8_t checksum;
    static Servo_parse_state_t parser = SERVO_IDLE;
    while (serialRxBytesWaiting(ServoPort)) {
        // read next byte
        uint8_t byte = serialRead(ServoPort);
        switch (parser) {
            case SERVO_IDLE:
                if (byte == START_BYTE_ACTUATORS_SERVO) {
                    // likely start of frame, reset checksum and buffer pointer
                    checksum = 0;
                    Servo_out_buf_u8view = (uint8_t*) &Servo_out_buf;
                    // put parser in next state
                    parser = SERVO_STX_FOUND;
                }
                break;
            case SERVO_STX_FOUND:
                *(Servo_out_buf_u8view++) = byte; // insert byte into the buffer
                checksum += byte;
                if ((uint8_t *) &Servo_out_buf + sizeof(struct ActuatorsServoOut) - 1 - Servo_out_buf_u8view == 0) {
                    // next byte is checksum
                    parser = SERVO_WAITING_FOR_CHECKSUM;
                }
                break;
            case SERVO_WAITING_FOR_CHECKSUM:
                *Servo_out_buf_u8view = byte;
                if (byte == checksum) {
                    // success
                    memcpy(&Servo_out, &Servo_out_buf, sizeof(struct ActuatorsServoOut));

                    // todo: hardcode for now
                    servo_feedback[0] = Servo_out.servo_1_angle; // centidegree
                    servo_feedback[1] = -Servo_out.servo_2_angle; // flipped in tailsitter
                    servo_feedback[2] = Servo_out.servo_3_angle;
                    servo_feedback[3] = Servo_out.servo_4_angle;
#ifdef USE_CLI_DEBUG_PRINT
                    static unsigned printCounter = 1;
                    if (printCounter++ % 100 == 0) {
                        printCounter = 1;
                        cliDebugPrintLinef("Servo position %d cdeg %d cdeg", servo_feedback[0], servo_feedback[1]);
                    }
#endif
                }
                parser = SERVO_IDLE;
                break;
        }
    }
}

#endif

