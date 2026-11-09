/*
 * Driver for Feetech STS serial bus servos (e.g. STS3032) on a half-duplex UART.
 *
 * All servos share one bus. Every FEETECH_SLOT_US one frame is sent, cycling
 * through: write goal position to servo 1, read telemetry of servo 1, write
 * servo 2, read servo 2, ... Replies are parsed asynchronously.
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
#include <math.h>

#include "platform.h"

#if defined(USE_SERVOS_FEETECH)

#ifndef USE_SERVOS
#error "USE_SERVOS_FEETECH requires USE_SERVOS"
#endif

#ifdef USE_ACTUATORS_T4
#error "USE_SERVOS_FEETECH and USE_ACTUATORS_T4 both write servo_feedback[]; enable only one"
#endif

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "io/serial.h"

#include "flight/servos.h"

#include "io/feetech.h"
#include "io/feetech_protocol.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

STATIC_ASSERT(FEETECH_SERVO_COUNT <= MAX_SUPPORTED_SERVOS, feetech_too_many_servos);

#define FEETECH_BAUDRATE            1000000 // servo factory default

// STS3032: 12-bit position, 4096 steps per revolution
#define FEETECH_STEPS_PER_REV       4096
#define FEETECH_CENTER_STEP         2048
#define FEETECH_MAX_STEP            4095

// servo_normalized = +-1 maps to +-100 deg. Feedback is in centidegrees, so
// servo_feedback * 1e-4 is again normalized (see servo_normalized_feedback).
#define FEETECH_DEG_PER_NORMALIZED  100.0f

#define FEETECH_SLOT_US             1000 // one frame per slot, so the bus never saturates
#define FEETECH_RX_TIMEOUT_US       2000 // drop partial frames after this much silence

// Telemetry read: position, speed, load (2 bytes each), voltage, temperature
#define FEETECH_TELEMETRY_REG_START FEETECH_REG_PRESENT_POSITION_L
#define FEETECH_TELEMETRY_REG_COUNT 8

#define FEETECH_FRAME_OVERHEAD      6 // 2x header, ID, LEN, INSTR/ERROR, CHECKSUM
#define FEETECH_MAX_TX_PARAMS       7 // goal position write
#define FEETECH_RX_BUFFER_SIZE      (FEETECH_FRAME_OVERHEAD + FEETECH_TELEMETRY_REG_COUNT)

// Frame byte offsets
#define FEETECH_IDX_ID              2
#define FEETECH_IDX_LEN             3
#define FEETECH_IDX_INSTR           4 // ERROR in replies
#define FEETECH_IDX_PARAMS          5

feetechTelemetry_t feetechTelemetry[FEETECH_SERVO_COUNT];

static serialPort_t *feetechPort = NULL;
static const serialPortConfig_t *portConfig = NULL;

static struct {
    uint8_t buf[FEETECH_RX_BUFFER_SIZE];
    uint8_t idx;
    timeUs_t lastByteUs;
} rx;

static inline uint8_t servoIdFromIndex(uint8_t servoIndex)
{
    return servoIndex + 1;
}

// ~(sum of bytes), over ID ... last param
static uint8_t feetechChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return ~sum;
}

// STS speed/load/position use sign-magnitude with the sign at bit signBit
static int16_t decodeSignMagnitude(uint8_t lo, uint8_t hi, uint8_t signBit)
{
    const uint16_t raw = lo | (hi << 8);
    const uint16_t signMask = 1u << signBit;
    const int16_t magnitude = raw & (signMask - 1);
    return (raw & signMask) ? -magnitude : magnitude;
}

static int16_t stepsToCentidegrees(int16_t steps)
{
    return (int16_t)(((int32_t)steps - FEETECH_CENTER_STEP) * 36000 / FEETECH_STEPS_PER_REV);
}

static uint16_t normalizedToSteps(float normalized)
{
    const float deg = normalized * FEETECH_DEG_PER_NORMALIZED;
    const long steps = lrintf(deg * (FEETECH_STEPS_PER_REV / 360.0f)) + FEETECH_CENTER_STEP;
    return (uint16_t)constrain(steps, 0, FEETECH_MAX_STEP);
}

static void feetechSendInstruction(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t paramCount)
{
    uint8_t frame[FEETECH_FRAME_OVERHEAD + FEETECH_MAX_TX_PARAMS];
    if (paramCount > FEETECH_MAX_TX_PARAMS) {
        return;
    }

    uint8_t len = 0;
    frame[len++] = FEETECH_HEADER_BYTE;
    frame[len++] = FEETECH_HEADER_BYTE;
    frame[len++] = id;
    frame[len++] = paramCount + 2;
    frame[len++] = instruction;
    memcpy(&frame[len], params, paramCount);
    len += paramCount;
    frame[len] = feetechChecksum(&frame[FEETECH_IDX_ID], len - FEETECH_IDX_ID);
    len++;

    serialWriteBuf(feetechPort, frame, len);
}

static void feetechWriteGoalPosition(uint8_t id, uint16_t steps)
{
    const uint8_t params[FEETECH_MAX_TX_PARAMS] = {
        FEETECH_REG_GOAL_POSITION_L,
        steps & 0xFF, steps >> 8,
        0, 0, // goal time: 0 = as fast as possible
        0, 0, // goal speed: 0 = no limit
    };
    feetechSendInstruction(id, FEETECH_INST_WRITE, params, sizeof(params));
}

static void feetechRequestTelemetry(uint8_t id)
{
    const uint8_t params[2] = { FEETECH_TELEMETRY_REG_START, FEETECH_TELEMETRY_REG_COUNT };
    feetechSendInstruction(id, FEETECH_INST_READ, params, sizeof(params));
}

static void feetechProcessFrame(void)
{
    const uint8_t *f = rx.buf;
    const uint8_t frameLen = f[FEETECH_IDX_LEN] + 4;

    if (feetechChecksum(&f[FEETECH_IDX_ID], frameLen - 3) != f[frameLen - 1]) {
        return;
    }

    // Only telemetry replies have this length. This also rejects the echoes
    // of our own requests (half duplex: READ has LEN 4, WRITE has LEN 9) and
    // the status replies to WRITE (LEN 2).
    if (f[FEETECH_IDX_LEN] != FEETECH_TELEMETRY_REG_COUNT + 2) {
        return;
    }

    const uint8_t id = f[FEETECH_IDX_ID];
    if (id < servoIdFromIndex(0) || id > servoIdFromIndex(FEETECH_SERVO_COUNT - 1)) {
        return;
    }
    const uint8_t servoIndex = id - servoIdFromIndex(0);

    const uint8_t *p = &f[FEETECH_IDX_PARAMS];
    feetechTelemetry_t *t = &feetechTelemetry[servoIndex];
    t->error       = f[FEETECH_IDX_INSTR];
    t->position    = stepsToCentidegrees(decodeSignMagnitude(p[0], p[1], 15));
    t->speed       = decodeSignMagnitude(p[2], p[3], 15);
    t->load        = decodeSignMagnitude(p[4], p[5], 10);
    t->voltage     = p[6];
    t->temperature = p[7];

    servo_feedback[servoIndex] = t->position;

#ifdef USE_CLI_DEBUG_PRINT
    static unsigned printCounter = 0;
    if (++printCounter >= 100) {
        printCounter = 0;
        cliDebugPrintLinef("Servo position %d cdeg %d cdeg", servo_feedback[0], servo_feedback[1]);
    }
#endif
}

static void feetechParseByte(uint8_t c)
{
    switch (rx.idx) {
    case 0:
    case 1:
        // sync on the 0xFF 0xFF preamble
        if (c == FEETECH_HEADER_BYTE) {
            rx.buf[rx.idx++] = c;
        } else {
            rx.idx = 0;
        }
        return;
    case FEETECH_IDX_ID:
        if (c == FEETECH_HEADER_BYTE) {
            rx.idx = 0;
            return; // extra preamble byte; 0xFF is not a valid ID, reset parser
        }
        break;
    case FEETECH_IDX_LEN:
        if (c < 2 || c + 4 > FEETECH_RX_BUFFER_SIZE) {
            rx.idx = 0; // invalid or longer than any frame we care about
            return;
        }
        break;
    default:
        break;
    }

    rx.buf[rx.idx++] = c;

    if (rx.idx > FEETECH_IDX_LEN && rx.idx == rx.buf[FEETECH_IDX_LEN] + 4) {
        feetechProcessFrame();
        rx.idx = 0;
    }
}

void initServosFeetech(void)
{
    portConfig = findSerialPortConfig(FUNCTION_SERVOS_FEETECH);
}

void configureServosFeetechPort(void)
{
    if (!portConfig) {
        return;
    }

    // half duplex on the TX pin, push-pull, no pull resistor
    feetechPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_SERVOS_FEETECH,
        NULL,
        NULL,
        FEETECH_BAUDRATE,
        MODE_RXTX,
        SERIAL_BIDIR | SERIAL_BIDIR_PP | SERIAL_BIDIR_NOPULL
    );
}

void freeServosFeetechPort(void)
{
    if (feetechPort) {
        closeSerialPort(feetechPort);
        feetechPort = NULL;
    }
}

void handleServosFeetech(timeUs_t currentTimeUs)
{
    if (!feetechPort) {
        return;
    }

    if (rx.idx > 0 && cmpTimeUs(currentTimeUs, rx.lastByteUs) > FEETECH_RX_TIMEOUT_US) {
        rx.idx = 0;
    }

    while (serialRxBytesWaiting(feetechPort)) {
        rx.lastByteUs = currentTimeUs;
        feetechParseByte(serialRead(feetechPort));
    }
}

void sendServosFeetech(timeUs_t currentTimeUs)
{
    static timeUs_t lastSlotUs = 0;
    static uint8_t slot = 0;

    if (!feetechPort || cmpTimeUs(currentTimeUs, lastSlotUs) < FEETECH_SLOT_US) {
        return;
    }
    lastSlotUs = currentTimeUs;

    // even slots write, odd slots read, two slots per servo
    const uint8_t servoIndex = slot / 2;
    const uint8_t id = servoIdFromIndex(servoIndex);
    if (slot % 2 == 0) {
        feetechWriteGoalPosition(id, normalizedToSteps(servo_normalized[servoIndex]));
    } else {
        feetechRequestTelemetry(id);
    }

    slot = (slot + 1) % (2 * FEETECH_SERVO_COUNT);
}

#endif // USE_SERVOS_FEETECH
