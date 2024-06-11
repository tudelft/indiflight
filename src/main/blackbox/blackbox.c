/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "blackbox.h"
#include "blackbox_encoding.h"
#include "blackbox_fielddefs.h"
#include "blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/encoding.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/board_info.h"
#include "fc/controlrate_profile.h"
#include "fc/parameter_names.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/gps_rescue.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/indi.h"
#include "flight/pos_ctl.h"
#include "flight/ekf.h"
#include "flight/learner.h"
#include "flight/nn_control.h"
#include "flight/catapult.h"
#include "flight/throw.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/local_pos.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif

#if defined(MOCKUP) && (MAX_SUPPORTED_MOTORS == 8)
#define BLACKBOX_LEARNER_LOG_8_MOTORS
#define BLACKBOX_LEARNER_N 8
#else
#define BLACKBOX_LEARNER_N 4
#endif

#define BLACKBOX_LEARNER_2N (BLACKBOX_LEARNER_N << 1)
#define BLACKBOX_LEARNER_MOTOR_RLS_N 4

PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 3);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .fields_disabled_mask = 0, // default log all fields
    .sample_rate = BLACKBOX_RATE_QUARTER,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .mode = BLACKBOX_MODE_NORMAL,
    .high_resolution = false
);

STATIC_ASSERT((sizeof(blackboxConfig()->fields_disabled_mask) * 8) >= FLIGHT_LOG_FIELD_SELECT_COUNT, too_many_flight_log_fields_selections);

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define FIELD_SELECT(x) CONCAT(FLIGHT_LOG_FIELD_SELECT_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n";

static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAYLEN(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),     .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    /* I terms get special packed encoding in P frames: */
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    {"axisF",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    /* rcCommands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   3, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},

    // setpoint - define 4 fields like rcCommand to use the same encoding. setpoint[4] contains the mixer throttle
    {"setpoint",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},

    {"vbatLatest",    -1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), CONDITION(VBAT)},
    {"amperageLatest",-1, SIGNED,   .Ipredict = PREDICT(0),        .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), CONDITION(AMPERAGE_ADC)},

#ifdef USE_MAG
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
#endif
#ifdef USE_BARO
    {"baroAlt",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(BARO)},
#endif
#ifdef USE_RANGEFINDER
    {"surfaceRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(RANGEFINDER)},
#endif
    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(RSSI)},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADCafterRpm",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADCafterRpm",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADCafterRpm",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"accSmooth",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accADCafterRpm",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accADCafterRpm",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accADCafterRpm",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",       0, UNSIGNED, .Ipredict = PREDICT(MINMOTOR), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",       1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",       2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",       3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor",       4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",       5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",       6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",       7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},

    /* Tricopter tail servo */
    {"servo",       5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)},

    /* INDI stuff*/
#ifdef USE_INDI
    {"quat",        0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quat",        1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quat",        2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quat",        3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"alpha",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"alpha",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"alpha",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"quatSp",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quatSp",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quatSp",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"quatSp",      3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"gyroSp",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"gyroSp",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"gyroSp",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"alphaSp",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"alphaSp",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"alphaSp",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"spfSp",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"spfSp",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"spfSp",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"dv",          0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"dv",          1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"dv",          2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"dv",          3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"dv",          4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},
    {"dv",          5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(INDI)},

    {"u",           0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u",           7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},

    {"u_state",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"u_state",     7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},

    {"omega",       0, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       2, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       3, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       4, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       5, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       6, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega",       7, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},

    {"omegaUnfiltered",       0, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       2, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       3, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       4, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       5, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       6, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omegaUnfiltered",       7, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},

    {"omega_dot",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
    {"omega_dot",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(INDI)},
#endif

#if USE_LOCAL_POSITION
    {"pos",         0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"pos",         1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"pos",         2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"extTime",     -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(UNSIGNED_VB), CONDITION(POS)},

    {"extPos",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extPos",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extPos",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"posSp",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"posSp",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"posSp",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"vel",         0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vel",         1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vel",         2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"extVel",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extVel",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extVel",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"velSp",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"velSp",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"velSp",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"accSp",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"accSp",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"accSp",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"extAtt",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extAtt",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"extAtt",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
#endif
#ifdef USE_VIO_POSE
    {"vioTime",    -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(UNSIGNED_VB), CONDITION(POS)},
    
    {"vioPos",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioPos",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioPos",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"vioVel",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioVel",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioVel",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"vioQuat",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioQuat",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioQuat",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioQuat",      3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},

    {"vioRate",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioRate",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
    {"vioRate",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(POS)},
#endif
#ifdef USE_EKF
    {"ekf_pos",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_pos",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_pos",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},

    {"ekf_vel",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_vel",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_vel",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},

    {"ekf_att",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_att",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_att",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},

    {"ekf_acc_b",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_acc_b",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_acc_b",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},

    {"ekf_gyro_b",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_gyro_b",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
    {"ekf_gyro_b",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),     .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(EKF)},
#endif

#ifdef USE_LEARNER
    {"motor_0_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_0_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_0_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_0_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_0_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_0_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_1_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_1_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_1_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_1_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_1_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_1_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_2_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_2_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_2_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_2_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_2_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_2_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_3_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_3_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_3_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_3_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_3_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_3_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
    {"motor_4_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_4_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_4_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_4_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_4_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_4_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_5_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_5_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_5_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_5_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_5_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_5_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_6_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_6_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_6_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_6_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_6_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_6_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"motor_7_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_7_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_7_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_7_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_7_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"motor_7_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
#endif

    {"imu_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"imu_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"imu_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
    {"fx_x_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"fx_y_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"fx_z_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
#else
    {"fx_x_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"fx_y_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"fx_z_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
#endif

    {"fx_p_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
    // todo: check very well if this actually works
    {"fx_p_rls_x",   8, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",   9, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  10, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  11, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  12, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  13, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  14, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_p_rls_x",  15, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#endif

    {"fx_q_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
    // todo: check very well if this actually works
    {"fx_q_rls_x",   8, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",   9, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  10, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  11, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  12, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  13, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  14, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_q_rls_x",  15, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#endif

    {"fx_r_rls_x",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
    // todo: check very well if this actually works
    {"fx_r_rls_x",   8, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",   9, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  10, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  11, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  12, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  13, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  14, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
    {"fx_r_rls_x",  15, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(LEARNER)},
#endif

    {"fx_x_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_x_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_y_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_z_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_p_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_p_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_q_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_q_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_r_rls_e_var",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"fx_r_rls_lambda",   -1, UNSIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"learnerGains",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerGains",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerGains",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerGains",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"hoverAttitude",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"hoverAttitude",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"hoverAttitude",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"hoverAttitude",      3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},

    {"learnerTimings",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
    {"learnerTimings",      6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(LEARNER)},
#endif
};

#ifdef USE_GPS
// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
};
#endif

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
};

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_CACHE_FLUSH,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN,
    BLACKBOX_STATE_START_ERASE,
    BLACKBOX_STATE_ERASING,
    BLACKBOX_STATE_ERASED
} BlackboxState;


typedef struct blackboxMainState_s {
    uint32_t time;

    int32_t axisPID_P[XYZ_AXIS_COUNT];
    int32_t axisPID_I[XYZ_AXIS_COUNT];
    int32_t axisPID_D[XYZ_AXIS_COUNT];
    int32_t axisPID_F[XYZ_AXIS_COUNT];

    int16_t rcCommand[4];
    int16_t setpoint[4];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t gyroADCafterRpm[XYZ_AXIS_COUNT];
    int16_t accADCf[XYZ_AXIS_COUNT];
    int16_t accADCafterRpm[XYZ_AXIS_COUNT];
    int16_t debug[DEBUG16_VALUE_COUNT];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    uint16_t vbatLatest;
    int32_t amperageLatest;

#ifdef USE_BARO
    int32_t baroAlt;
#endif
#ifdef USE_MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef USE_RANGEFINDER
    int32_t surfaceRaw;
#endif
    uint16_t rssi;
#ifdef USE_INDI
    int16_t quat[4];
    int16_t alpha[XYZ_AXIS_COUNT];
    int16_t quatSp[4];
    int16_t gyroSp[XYZ_AXIS_COUNT];
    int16_t alphaSp[XYZ_AXIS_COUNT];
    int16_t spfSp[XYZ_AXIS_COUNT];
    int16_t dv[MAXV];
    int16_t u[MAXU];
    int16_t u_state[MAXU];
    uint16_t omega[MAXU];
    uint16_t omegaUnfiltered[MAXU];
    int16_t omega_dot[MAXU];
#endif
#ifdef USE_LOCAL_POSITION
    int32_t pos[XYZ_AXIS_COUNT]; // will be mm, so must be more than 16bit
    uint32_t extTime; // will be in ms, so must be more than 32bit
    int32_t extPos[XYZ_AXIS_COUNT]; // will be mm, so must be more than 16bit
    int32_t posSp[XYZ_AXIS_COUNT];
    int16_t vel[XYZ_AXIS_COUNT]; // will be cm/s, so this is fine
    int16_t extVel[XYZ_AXIS_COUNT]; // will be cm/s, so this is fine
    int16_t velSp[XYZ_AXIS_COUNT]; 
    int16_t accSp[XYZ_AXIS_COUNT]; // will be cm/s/s, so this is fine
    int16_t extAtt[XYZ_AXIS_COUNT]; // will be degrees/1000
#ifdef USE_VIO_POSE
    uint32_t vioTime;           // will be in ms, so must be more than 32bit
    int32_t vioPos[XYZ_AXIS_COUNT]; // will be mm, so must be more than 16bit
    int16_t vioVel[XYZ_AXIS_COUNT]; // will be cm/s, so this is fine
    int16_t vioQuat[4];
    int16_t vioRate[XYZ_AXIS_COUNT]; // will be degrees/1000
#endif
#endif
#ifdef USE_EKF
    int16_t ekf_pos[XYZ_AXIS_COUNT]; // will be mm, so must be more than 16bit
    int16_t ekf_vel[XYZ_AXIS_COUNT]; // will be cm/s, so this is fine
    int16_t ekf_att[XYZ_AXIS_COUNT]; // will be degrees/1000
    int16_t ekf_acc_b[XYZ_AXIS_COUNT];
    int16_t ekf_gyro_b[XYZ_AXIS_COUNT];
#endif
#ifdef USE_LEARNER
    int16_t motor_rls_x[BLACKBOX_LEARNER_N][BLACKBOX_LEARNER_MOTOR_RLS_N];
    int16_t imu_rls_x[XYZ_AXIS_COUNT];
    int16_t fx_x_rls_x[BLACKBOX_LEARNER_N];
    int16_t fx_y_rls_x[BLACKBOX_LEARNER_N];
    int16_t fx_z_rls_x[BLACKBOX_LEARNER_N];
    int16_t fx_p_rls_x[BLACKBOX_LEARNER_2N];
    int16_t fx_q_rls_x[BLACKBOX_LEARNER_2N];
    int16_t fx_r_rls_x[BLACKBOX_LEARNER_2N];
    uint16_t eVarsMotor[BLACKBOX_LEARNER_N];
    uint16_t eVarsFx[6];
    uint16_t lambdasMotor[BLACKBOX_LEARNER_N];
    uint16_t lambdasFx[6]; // 6 degrees of freedom
    uint16_t learnerGains[LEARNER_LOOP_COUNT];
    int16_t hoverAttitude[4];
    int32_t learnerTimings[LEARNER_TIMINGS_NUM];
#endif
} blackboxMainState_t;

typedef struct blackboxGpsState_s {
    int32_t GPS_home[2];
    int32_t GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

//From rc_controls.c
extern boxBitmask_t rcModeActivationMask;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes

static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        uint32_t startTime;
    } u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_LAST, too_many_flight_log_conditions);

static uint32_t blackboxIteration;
static uint16_t blackboxLoopIndex;
static uint16_t blackboxPFrameIndex;
static uint16_t blackboxIFrameIndex;
// number of flight loop iterations before logging I-frame
// typically 32 for 1kHz loop, 64 for 2kHz loop etc
STATIC_UNIT_TESTED int16_t blackboxIInterval = 0;
// number of flight loop iterations before logging P-frame
STATIC_UNIT_TESTED int8_t blackboxPInterval = 0;
STATIC_UNIT_TESTED int32_t blackboxSInterval = 0;
STATIC_UNIT_TESTED int32_t blackboxSlowFrameIterationTimer;
static bool blackboxLoggedAnyFrames;
static float blackboxHighResolutionScale;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];

static bool blackboxModeActivationConditionPresent = false;

/**
 * Return true if it is safe to edit the Blackbox configuration.
 */
bool blackboxMayEditConfig(void)
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsOnlyLoggingIntraframes(void)
{
    return blackboxPInterval == 0;
}

static bool isFieldEnabled(FlightLogFieldSelect_e field)
{
    return (blackboxConfig()->fields_disabled_mask & (1 << field)) == 0;
}

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
    case CONDITION(ALWAYS):
        return true;

    case CONDITION(AT_LEAST_MOTORS_1):
    case CONDITION(AT_LEAST_MOTORS_2):
    case CONDITION(AT_LEAST_MOTORS_3):
    case CONDITION(AT_LEAST_MOTORS_4):
    case CONDITION(AT_LEAST_MOTORS_5):
    case CONDITION(AT_LEAST_MOTORS_6):
    case CONDITION(AT_LEAST_MOTORS_7):
    case CONDITION(AT_LEAST_MOTORS_8):
        return (getMotorCount() >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(TRICOPTER):
        return (mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(PID):
        return isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(NONZERO_PID_D_0):
    case CONDITION(NONZERO_PID_D_1):
    case CONDITION(NONZERO_PID_D_2):
        return (currentPidProfile->pid[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0].D != 0) && isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(RC_COMMANDS):
        return isFieldEnabled(FIELD_SELECT(RC_COMMANDS));

    case CONDITION(SETPOINT):
        return isFieldEnabled(FIELD_SELECT(SETPOINT));

    case CONDITION(MAG):
#ifdef USE_MAG
        return sensors(SENSOR_MAG) && isFieldEnabled(FIELD_SELECT(MAG));
#else
        return false;
#endif

    case CONDITION(BARO):
#ifdef USE_BARO
        return sensors(SENSOR_BARO) && isFieldEnabled(FIELD_SELECT(ALTITUDE));
#else
        return false;
#endif

    case CONDITION(VBAT):
        return (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(AMPERAGE_ADC):
        return (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) && (batteryConfig()->currentMeterSource != CURRENT_METER_VIRTUAL) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(RANGEFINDER):
#ifdef USE_RANGEFINDER
        return sensors(SENSOR_RANGEFINDER) && isFieldEnabled(FIELD_SELECT(ALTITUDE));
#else
        return false;
#endif

    case CONDITION(RSSI):
        return isRssiConfigured() && isFieldEnabled(FIELD_SELECT(RSSI));

    case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
        return blackboxPInterval != blackboxIInterval;

    case CONDITION(GYRO):
        return isFieldEnabled(FIELD_SELECT(GYRO));

    case CONDITION(ACC):
        return sensors(SENSOR_ACC) && isFieldEnabled(FIELD_SELECT(ACC));

    case CONDITION(DEBUG_LOG):
        return (debugMode != DEBUG_NONE) && isFieldEnabled(FIELD_SELECT(DEBUG_LOG));

    case CONDITION(INDI):
#ifdef USE_INDI
        return isFieldEnabled(FIELD_SELECT(INDI));
#else
        return false;
#endif

    case CONDITION(POS):
#ifdef USE_LOCAL_POSITION
        return isFieldEnabled(FIELD_SELECT(POS));
#else
        return false;
#endif
    
    case CONDITION(EKF):
#ifdef USE_EKF
        return isFieldEnabled(FIELD_SELECT(EKF));
#else
        return false;
#endif

    case CONDITION(LEARNER):
#ifdef USE_LEARNER
        return isFieldEnabled(FIELD_SELECT(LEARNER));
#else
        return false;
#endif

    case CONDITION(NEVER):
        return false;

    default:
        return false;
    }
}

static void blackboxBuildConditionCache(void)
{
    blackboxConditionCache = 0;
    for (FlightLogFieldCondition cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
        if (testBlackboxConditionUncached(cond)) {
            blackboxConditionCache |= 1 << cond;
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}

static void blackboxSetState(BlackboxState newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        blackboxLoggedAnyFrames = false;
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxHeaderBudget = 0;
        xmitState.headerIndex = 0;
        xmitState.u.startTime = millis();
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        xmitState.headerIndex = 0;
        break;
    case BLACKBOX_STATE_RUNNING:
        blackboxSlowFrameIterationTimer = blackboxSInterval; //Force a slow frame to be written on the first iteration
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        xmitState.u.startTime = millis();
        break;
    default:
        ;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    if (testBlackboxCondition(CONDITION(PID))) {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);

        // Don't bother writing the current D term if the corresponding PID setting is zero
        for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
            if (testBlackboxCondition(CONDITION(NONZERO_PID_D_0) + x)) {
                blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
            }
        }

        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_F, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(RC_COMMANDS))) {
        // Write roll, pitch and yaw first:
        blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);

        /*
         * Write the throttle separately from the rest of the RC data as it's unsigned.
         * Throttle lies in range [PWM_RANGE_MIN..PWM_RANGE_MAX]:
         */
        blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[THROTTLE]);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        // Write setpoint roll, pitch, yaw, and throttle
        blackboxWriteSigned16VBArray(blackboxCurrent->setpoint, 4);
    }

    if (testBlackboxCondition(CONDITION(VBAT))) {
        /*
         * Our voltage is expected to decrease over the course of the flight, so store our difference from
         * the reference:
         *
         * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
         */
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }

    if (testBlackboxCondition(CONDITION(AMPERAGE_ADC))) {
        // 12bit value directly from ADC
        blackboxWriteSignedVB(blackboxCurrent->amperageLatest);
    }

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
    }
#endif

#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO))) {
        blackboxWriteSignedVB(blackboxCurrent->baroAlt);
    }
#endif

#ifdef USE_RANGEFINDER
    if (testBlackboxCondition(CONDITION(RANGEFINDER))) {
        blackboxWriteSignedVB(blackboxCurrent->surfaceRaw);
    }
#endif

    if (testBlackboxCondition(CONDITION(RSSI))) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroADCafterRpm, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->accADCf, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->accADCafterRpm, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(DEBUG_LOG))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->debug, DEBUG16_VALUE_COUNT);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        //Motors can be below minimum output when disarmed, but that doesn't happen much
        blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - getMotorOutputLow());

        //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
        const int motorCount = getMotorCount();
        for (int x = 1; x < motorCount; x++) {
            blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
        }

        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
            //Assume the tail spends most of its time around the center
            blackboxWriteSignedVB(blackboxCurrent->servo[5] - 1500);
        }
    }

#ifdef USE_INDI
    if (testBlackboxCondition(CONDITION(INDI))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->quat, 4);
        blackboxWriteSigned16VBArray(blackboxCurrent->alpha, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->quatSp, 4);
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->alphaSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->spfSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->dv, MAXV);
        blackboxWriteSigned16VBArray(blackboxCurrent->u, MAXU);
        blackboxWriteSigned16VBArray(blackboxCurrent->u_state, MAXU);
        blackboxWriteUnsigned16VBArray(blackboxCurrent->omega, MAXU);
        blackboxWriteUnsigned16VBArray(blackboxCurrent->omegaUnfiltered, MAXU);
        blackboxWriteSigned16VBArray(blackboxCurrent->omega_dot, MAXU);
    }
#endif
#ifdef USE_LOCAL_POSITION
    if (testBlackboxCondition(CONDITION(POS))) {
        blackboxWriteSignedVBArray(blackboxCurrent->pos, XYZ_AXIS_COUNT);
        blackboxWriteUnsignedVB(blackboxCurrent->extTime);
        blackboxWriteSignedVBArray(blackboxCurrent->extPos, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->posSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->vel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->extVel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->velSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->accSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->extAtt, XYZ_AXIS_COUNT);
#ifdef USE_VIO_POSE
        blackboxWriteUnsignedVB(blackboxCurrent->vioTime);
        blackboxWriteSignedVBArray(blackboxCurrent->vioPos, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->vioVel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->vioQuat, 4);
        blackboxWriteSigned16VBArray(blackboxCurrent->vioRate, XYZ_AXIS_COUNT);
#endif
    }
#endif
#ifdef USE_EKF
    if (testBlackboxCondition(CONDITION(EKF))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->ekf_pos, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->ekf_vel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->ekf_att, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->ekf_acc_b, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->ekf_gyro_b, XYZ_AXIS_COUNT);
    }
#endif
#ifdef USE_LEARNER
    if (testBlackboxCondition(CONDITION(LEARNER))) {
        for (int i = 0; i < BLACKBOX_LEARNER_N; i++ ) {
            blackboxWriteSigned16VBArray(blackboxCurrent->motor_rls_x[i], BLACKBOX_LEARNER_MOTOR_RLS_N);
            blackboxWriteUnsignedVB(blackboxCurrent->eVarsMotor[i]);
            blackboxWriteUnsignedVB(blackboxCurrent->lambdasMotor[i]);
        }
        blackboxWriteSigned16VBArray(blackboxCurrent->imu_rls_x, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_x_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_y_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_z_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_p_rls_x, BLACKBOX_LEARNER_2N);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_q_rls_x, BLACKBOX_LEARNER_2N);
        blackboxWriteSigned16VBArray(blackboxCurrent->fx_r_rls_x, BLACKBOX_LEARNER_2N);
        for (int i = 0; i < 6; i++ ) {
            blackboxWriteUnsignedVB(blackboxCurrent->eVarsFx[i]);
            blackboxWriteUnsignedVB(blackboxCurrent->lambdasFx[i]);
        }
        blackboxWriteUnsigned16VBArray(blackboxCurrent->learnerGains, LEARNER_LOOP_COUNT);
        blackboxWriteSigned16VBArray(blackboxCurrent->hoverAttitude, 4);
        blackboxWriteSignedVBArray(blackboxCurrent->learnerTimings, LEARNER_TIMINGS_NUM);
    }
#endif

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr  = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++) {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}

static void writeInterframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    //No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    int32_t deltas[MAX(MAX_SUPPORTED_MOTORS, BLACKBOX_LEARNER_2N)];
    int16_t deltas16[MAX(MAX_SUPPORTED_MOTORS, BLACKBOX_LEARNER_2N)];
    int32_t setpointDeltas[4];

    if (testBlackboxCondition(CONDITION(PID))) {
        arraySubInt32(deltas, blackboxCurrent->axisPID_P, blackboxLast->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

        /*
         * The PID I field changes very slowly, most of the time +-2, so use an encoding
         * that can pack all three fields into one byte in that situation.
         */
        arraySubInt32(deltas, blackboxCurrent->axisPID_I, blackboxLast->axisPID_I, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        /*
         * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
         * always zero. So don't bother recording D results when PID D terms are zero.
         */
        for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
                blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
            }
        }

        arraySubInt32(deltas, blackboxCurrent->axisPID_F, blackboxLast->axisPID_F, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);
    }

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    for (int x = 0; x < 4; x++) {
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
        setpointDeltas[x] = blackboxCurrent->setpoint[x] - blackboxLast->setpoint[x];
    }

    if (testBlackboxCondition(CONDITION(RC_COMMANDS))) {
        blackboxWriteTag8_4S16(deltas);
    }
    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        blackboxWriteTag8_4S16(setpointDeltas);
    }

    //Check for sensors that are updated periodically (so deltas are normally zero)
    int optionalFieldCount = 0;

    if (testBlackboxCondition(CONDITION(VBAT))) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
    }

    if (testBlackboxCondition(CONDITION(AMPERAGE_ADC))) {
        deltas[optionalFieldCount++] = blackboxCurrent->amperageLatest - blackboxLast->amperageLatest;
    }

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
        }
    }
#endif

#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO))) {
        deltas[optionalFieldCount++] = blackboxCurrent->baroAlt - blackboxLast->baroAlt;
    }
#endif

#ifdef USE_RANGEFINDER
    if (testBlackboxCondition(CONDITION(RANGEFINDER))) {
        deltas[optionalFieldCount++] = blackboxCurrent->surfaceRaw - blackboxLast->surfaceRaw;
    }
#endif

    if (testBlackboxCondition(CONDITION(RSSI))) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxLast->rssi;
    }

    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

    //Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC),   XYZ_AXIS_COUNT);
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADCafterRpm),   XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADCf), XYZ_AXIS_COUNT);
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADCafterRpm), XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(DEBUG_LOG))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), DEBUG16_VALUE_COUNT);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor),     getMotorCount());

        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
            blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
        }
    }

#ifdef USE_INDI
    if (testBlackboxCondition(CONDITION(INDI))) {
        // quat attitude
        arraySubInt16(deltas16, blackboxCurrent->quat, blackboxLast->quat, 4);
        //for (int i=0; i < 4; i++)
        //    deltas[i] = deltas16[i];
        //blackboxWriteTag8_4S16(deltas);
        blackboxWriteSigned16VBArray(deltas16, 4);

        // rotational accel
        arraySubInt16(deltas16, blackboxCurrent->alpha, blackboxLast->alpha, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        // quat setpoints
        arraySubInt16(deltas16, blackboxCurrent->quatSp, blackboxLast->quatSp, 4);
        //for (int i=0; i < 4; i++)
        //    deltas[i] = deltas16[i];
        //blackboxWriteTag8_4S16(deltas);
        blackboxWriteSigned16VBArray(deltas16, 4);

        // rotational rate setpoint
        arraySubInt16(deltas16, blackboxCurrent->gyroSp, blackboxLast->gyroSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        // rotational accel setpoint
        arraySubInt16(deltas16, blackboxCurrent->alphaSp, blackboxLast->alphaSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        // rotational accel setpoint
        arraySubInt16(deltas16, blackboxCurrent->spfSp, blackboxLast->spfSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        // dv
        arraySubInt16(deltas16, blackboxCurrent->dv, blackboxLast->dv, MAXV);
        blackboxWriteSigned16VBArray(deltas16, MAXV);

        // u
        arraySubInt16(deltas16, blackboxCurrent->u, blackboxLast->u, MAXU);
        for (int i=0; i < MAXU; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, MAXU);

        // u_state
        arraySubInt16(deltas16, blackboxCurrent->u_state, blackboxLast->u_state, MAXU);
        for (int i=0; i < MAXU; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, MAXU);

        // omega
        arraySubUint16(deltas16, blackboxCurrent->omega, blackboxLast->omega, MAXU);
        for (int i=0; i < MAXU; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, MAXU);

        // omega
        arraySubUint16(deltas16, blackboxCurrent->omegaUnfiltered, blackboxLast->omegaUnfiltered, MAXU);
        for (int i=0; i < MAXU; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, MAXU);

        // omega_dot
        arraySubInt16(deltas16, blackboxCurrent->omega_dot, blackboxLast->omega_dot, MAXU);
        for (int i=0; i < MAXU; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, MAXU);
    }
#else
    UNUSED(deltas16);
#endif // USE_INDI

#ifdef USE_LOCAL_POSITION
    if (testBlackboxCondition(CONDITION(POS))) {
        arraySubInt32(deltas, blackboxCurrent->pos, blackboxLast->pos, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

        blackboxWriteUnsignedVB(blackboxCurrent->extTime - blackboxLast->extTime);

        arraySubInt32(deltas, blackboxCurrent->extPos, blackboxLast->extPos, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

        arraySubInt32(deltas, blackboxCurrent->posSp, blackboxLast->posSp, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->vel, blackboxLast->vel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->extVel, blackboxLast->extVel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->velSp, blackboxLast->velSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->accSp, blackboxLast->accSp, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->extAtt, blackboxLast->extAtt, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);
#ifdef USE_VIO_POSE
        blackboxWriteUnsignedVB(blackboxCurrent->vioTime - blackboxLast->vioTime);

        arraySubInt32(deltas, blackboxCurrent->vioPos, blackboxLast->vioPos, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->vioVel, blackboxLast->vioVel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->vioQuat, blackboxLast->vioQuat, 4);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->vioRate, blackboxLast->vioRate, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);
#endif
    }
#else
    UNUSED(deltas16);
#endif

#ifdef USE_EKF
    if (testBlackboxCondition(CONDITION(EKF))) {
        arraySubInt16(deltas16, blackboxCurrent->ekf_pos, blackboxLast->ekf_pos, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->ekf_vel, blackboxLast->ekf_vel, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->ekf_att, blackboxLast->ekf_att, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->ekf_acc_b, blackboxLast->ekf_acc_b, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->ekf_gyro_b, blackboxLast->ekf_gyro_b, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, XYZ_AXIS_COUNT);
    }
#else
    UNUSED(deltas16);
#endif

#ifdef USE_LEARNER
    if (testBlackboxCondition(CONDITION(LEARNER))) {
        int16_t dlambdasMotor[BLACKBOX_LEARNER_N], deVarsMotor[BLACKBOX_LEARNER_N];
        int16_t dlambdasFx[6], deVarsFx[6];
        arraySubUint16(dlambdasMotor, blackboxCurrent->lambdasMotor, blackboxLast->lambdasMotor, BLACKBOX_LEARNER_N);
        arraySubUint16(deVarsMotor, blackboxCurrent->eVarsMotor, blackboxLast->eVarsMotor, BLACKBOX_LEARNER_N);
        arraySubUint16(dlambdasFx, blackboxCurrent->lambdasFx, blackboxLast->lambdasFx, 6);
        arraySubUint16(deVarsFx, blackboxCurrent->eVarsFx, blackboxLast->eVarsFx, 6);

        for (int i = 0; i < BLACKBOX_LEARNER_N; i++ ) {
            arraySubInt16(deltas16, blackboxCurrent->motor_rls_x[i], blackboxLast->motor_rls_x[i], BLACKBOX_LEARNER_MOTOR_RLS_N);
            blackboxWriteSigned16VBArray(deltas16, BLACKBOX_LEARNER_MOTOR_RLS_N);
            blackboxWriteSignedVB(deVarsMotor[i]);
            blackboxWriteSignedVB(dlambdasMotor[i]);
        }

        arraySubInt16(deltas16, blackboxCurrent->imu_rls_x, blackboxLast->imu_rls_x, XYZ_AXIS_COUNT);
        blackboxWriteSigned16VBArray(deltas16, 3);

        arraySubInt16(deltas16, blackboxCurrent->fx_x_rls_x, blackboxLast->fx_x_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(deltas16, BLACKBOX_LEARNER_N);

        arraySubInt16(deltas16, blackboxCurrent->fx_y_rls_x, blackboxLast->fx_y_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(deltas16, BLACKBOX_LEARNER_N);

        arraySubInt16(deltas16, blackboxCurrent->fx_z_rls_x, blackboxLast->fx_z_rls_x, BLACKBOX_LEARNER_N);
        blackboxWriteSigned16VBArray(deltas16, BLACKBOX_LEARNER_N);

        arraySubInt16(deltas16, blackboxCurrent->fx_p_rls_x, blackboxLast->fx_p_rls_x, BLACKBOX_LEARNER_2N);
        for (int i=0; i < BLACKBOX_LEARNER_2N; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, BLACKBOX_LEARNER_N);
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
        blackboxWriteTag8_8SVB(deltas+8, BLACKBOX_LEARNER_N);
#endif

        arraySubInt16(deltas16, blackboxCurrent->fx_q_rls_x, blackboxLast->fx_q_rls_x, BLACKBOX_LEARNER_2N);
        for (int i=0; i < BLACKBOX_LEARNER_2N; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, BLACKBOX_LEARNER_N);
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
        blackboxWriteTag8_8SVB(deltas+8, BLACKBOX_LEARNER_N);
#endif

        arraySubInt16(deltas16, blackboxCurrent->fx_r_rls_x, blackboxLast->fx_r_rls_x, BLACKBOX_LEARNER_2N);
        for (int i=0; i < BLACKBOX_LEARNER_2N; i++)
            deltas[i] = deltas16[i];
        blackboxWriteTag8_8SVB(deltas, BLACKBOX_LEARNER_N);
#ifdef BLACKBOX_LEARNER_LOG_8_MOTORS
        blackboxWriteTag8_8SVB(deltas+8, BLACKBOX_LEARNER_N);
#endif

        for (int i=0; i < 6; i++) {
            blackboxWriteSignedVB(deVarsFx[i]);
            blackboxWriteSignedVB(dlambdasFx[i]);
        }

        arraySubUint16(deltas16, blackboxCurrent->learnerGains, blackboxLast->learnerGains, LEARNER_LOOP_COUNT);
        blackboxWriteSigned16VBArray(deltas16, LEARNER_LOOP_COUNT);

        arraySubInt16(deltas16, blackboxCurrent->hoverAttitude, blackboxLast->hoverAttitude, 4);
        blackboxWriteSigned16VBArray(deltas16, 4);

        arraySubInt32(deltas, blackboxCurrent->learnerTimings, blackboxLast->learnerTimings, LEARNER_TIMINGS_NUM);
        blackboxWriteSignedVBArray(deltas, LEARNER_TIMINGS_NUM);
    }
#else
    UNUSED(deltas16);
#endif // USE_LEARNER

    //Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    int32_t values[3];

    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);

    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);

    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t *slow)
{
    memcpy(&slow->flightModeFlags, &rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than blackboxSInterval logging iterations
 * since the field was last logged.
 */
STATIC_UNIT_TESTED bool writeSlowFrameIfNeeded(void)
{
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    bool shouldWrite = blackboxSlowFrameIterationTimer >= blackboxSInterval;

    if (shouldWrite) {
        loadSlowState(&slowHistory);
    } else {
        blackboxSlowState_t newSlowState;

        loadSlowState(&newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
            // Use the new state as our new history
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            shouldWrite = true;
        }
    }

    if (shouldWrite) {
        writeSlowFrame();
    }
    return shouldWrite;
}

void blackboxValidateConfig(void)
{
    // If we've chosen an unsupported device, change the device to serial
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#endif
#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
#endif
    case BLACKBOX_DEVICE_SERIAL:
        // Device supported, leave the setting alone
        break;

    default:
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
    }
}

static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    blackboxIFrameIndex = 0;
    blackboxPFrameIndex = 0;
    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
static void blackboxStart(void)
{
    blackboxValidateConfig();

    if (!blackboxDeviceOpen()) {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
        return;
    }

    memset(&gpsHistory, 0, sizeof(gpsHistory));

    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];

    vbatReference = getBatteryVoltageLatest();

    //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

    /*
     * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
     * must always agree with the logged data, the results of these tests must not change during logging. So
     * cache those now.
     */
    blackboxBuildConditionCache();

    blackboxModeActivationConditionPresent = isModeActivationConditionPresent(BOXBLACKBOX);

    blackboxResetIterationTimers();

    /*
     * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
     * it finally plays the beep for this arming event.
     */
    blackboxLastArmingBeep = getArmingBeepTimeMicros();
    memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags)); // record startup status

    blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}

/**
 * Begin Blackbox shutdown.
 */
void blackboxFinish(void)
{
    switch (blackboxState) {
    case BLACKBOX_STATE_DISABLED:
    case BLACKBOX_STATE_STOPPED:
    case BLACKBOX_STATE_SHUTTING_DOWN:
        // We're already stopped/shutting down
        break;
    case BLACKBOX_STATE_RUNNING:
    case BLACKBOX_STATE_PAUSED:
        blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
        FALLTHROUGH;
    default:
        blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
    }
}

/**
 * Test Motors Blackbox Logging
 */
static bool startedLoggingInTestMode = false;

static void startInTestMode(void)
{
    if (!startedLoggingInTestMode) {
        if (blackboxConfig()->device == BLACKBOX_DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return; // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
        blackboxStart();
        startedLoggingInTestMode = true;
    }
}

static void stopInTestMode(void)
{
    if (startedLoggingInTestMode) {
        blackboxFinish();
        startedLoggingInTestMode = false;
    }
}
/**
 * We are going to monitor the MSP_SET_MOTOR target variables motor_disarmed[] for values other than minthrottle
 * on reading a value (i.e. the user is testing the motors), then we enable test mode logging;
 * we monitor when the values return to minthrottle and start a delay timer (5 seconds); if
 * the test motors are left at minimum throttle for this delay timer, then we assume we are done testing and
 * shutdown the logger.
 *
 * Of course, after the 5 seconds and shutdown of the logger, the system will be re-enabled to allow the
 * test mode to trigger again; its just that the data will be in a second, third, fourth etc log file.
 */
static bool inMotorTestMode(void)
{
    static uint32_t resetTime = 0;

    if (!ARMING_FLAG(ARMED) && areMotorsRunning()) {
        resetTime = millis() + 5000; // add 5 seconds
        return true;
    } else {
        // Monitor the duration at minimum
        return (millis() < resetTime);
    }
    return false;
}

#ifdef USE_GPS
static void writeGPSHomeFrame(void)
{
    blackboxWrite('H');

    blackboxWriteSignedVB(GPS_home[0]);
    blackboxWriteSignedVB(GPS_home[1]);
    //TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame(timeUs_t currentTimeUs)
{
    blackboxWrite('G');

    /*
     * If we're logging every frame, then a GPS frame always appears just after a frame with the
     * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
     *
     * If we're not logging every frame, we need to store the time of this GPS frame.
     */
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        // Predict the time of the last frame in the main log
        blackboxWriteUnsignedVB(currentTimeUs - blackboxHistory[1]->time);
    }

    blackboxWriteUnsignedVB(gpsSol.numSat);
    blackboxWriteSignedVB(gpsSol.llh.lat - gpsHistory.GPS_home[GPS_LATITUDE]);
    blackboxWriteSignedVB(gpsSol.llh.lon - gpsHistory.GPS_home[GPS_LONGITUDE]);
    blackboxWriteUnsignedVB(gpsSol.llh.altCm / 10); // was originally designed to transport meters in int16, but +-3276.7m is a good compromise
    blackboxWriteUnsignedVB(gpsSol.groundSpeed);
    blackboxWriteUnsignedVB(gpsSol.groundCourse);

    gpsHistory.GPS_numSat = gpsSol.numSat;
    gpsHistory.GPS_coord[GPS_LATITUDE] = gpsSol.llh.lat;
    gpsHistory.GPS_coord[GPS_LONGITUDE] = gpsSol.llh.lon;
}
#endif

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs)
{
#ifndef UNIT_TEST
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = lrintf(pidData[i].P);
        blackboxCurrent->axisPID_I[i] = lrintf(pidData[i].I);
        blackboxCurrent->axisPID_D[i] = lrintf(pidData[i].D);
        blackboxCurrent->axisPID_F[i] = lrintf(pidData[i].F);
        blackboxCurrent->gyroADC[i] = lrintf(gyro.gyroADCf[i] * blackboxHighResolutionScale);
        blackboxCurrent->gyroADCafterRpm[i] = lrintf(gyro.gyroADCafterRpm[i] * blackboxHighResolutionScale);
#if defined(USE_ACC)
        blackboxCurrent->accADCf[i] = lrintf(acc.accADCf[i]);
        blackboxCurrent->accADCafterRpm[i] = lrintf(acc.accADCafterRpm[i]);
#endif
#ifdef USE_MAG
        blackboxCurrent->magADC[i] = lrintf(mag.magADC[i]);
#endif
    }

    for (int i = 0; i < 4; i++) {
        blackboxCurrent->rcCommand[i] = lrintf(rcCommand[i] * blackboxHighResolutionScale);
    }

    // log the currentPidSetpoint values applied to the PID controller
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->setpoint[i] = lrintf(pidGetPreviousSetpoint(i) * blackboxHighResolutionScale);
    }
    // log the final throttle value used in the mixer
    blackboxCurrent->setpoint[3] = lrintf(mixerGetThrottle() * 1000);

    for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }

    const int motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = lrintf(motor[i]);
    }

    blackboxCurrent->vbatLatest = getBatteryVoltageLatest();
    blackboxCurrent->amperageLatest = getAmperageLatest();

#ifdef USE_BARO
    blackboxCurrent->baroAlt = baro.altitude;
#endif

#ifdef USE_RANGEFINDER
    // Store the raw sonar value without applying tilt correction
    blackboxCurrent->surfaceRaw = rangefinderGetLatestAltitude();
#endif

    blackboxCurrent->rssi = getRssi();

#ifdef USE_SERVOS
    //Tail servo for tricopters
    blackboxCurrent->servo[5] = servo[5];
#endif

#ifdef USE_INDI
    fp_quaternion_t attitude_q;
    getHoverAttitudeQuaternion(&attitude_q);
    blackboxCurrent->quat[0] = lrintf(attitude_q.w * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quat[1] = lrintf(attitude_q.x * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quat[2] = lrintf(attitude_q.y * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quat[3] = lrintf(attitude_q.z * UNIT_FLOAT_TO_SIGNED16VB); // FRD and not FLU
    blackboxCurrent->alpha[0] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDot_fs.V.X) * 0.1f);
    blackboxCurrent->alpha[1] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDot_fs.V.Y) * 0.1f);
    blackboxCurrent->alpha[2] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDot_fs.V.Z) * 0.1f);
    blackboxCurrent->quatSp[0] = lrintf(indiRun.attSpNed.w * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quatSp[1] = lrintf(indiRun.attSpNed.x * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quatSp[2] = lrintf(indiRun.attSpNed.y * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->quatSp[3] = lrintf(indiRun.attSpNed.z * UNIT_FLOAT_TO_SIGNED16VB); // FRD and not FLU
    blackboxCurrent->gyroSp[0] = lrintf(RADIANS_TO_DEGREES(indiRun.rateSpBody.V.X) * blackboxHighResolutionScale);
    blackboxCurrent->gyroSp[1] = lrintf(RADIANS_TO_DEGREES(indiRun.rateSpBody.V.Y) * blackboxHighResolutionScale);
    blackboxCurrent->gyroSp[2] = lrintf(RADIANS_TO_DEGREES(indiRun.rateSpBody.V.Z) * blackboxHighResolutionScale);
    blackboxCurrent->alphaSp[0] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDotSpBody.V.X) * 0.1f);
    blackboxCurrent->alphaSp[1] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDotSpBody.V.Y) * 0.1f);
    blackboxCurrent->alphaSp[2] = lrintf(RADIANS_TO_DEGREES(indiRun.rateDotSpBody.V.Z) * 0.1f);
    blackboxCurrent->spfSp[0] = lrintf(indiRun.spfSpBody.V.X*100.f);
    blackboxCurrent->spfSp[1] = lrintf(indiRun.spfSpBody.V.Y*100.f);
    blackboxCurrent->spfSp[2] = lrintf(indiRun.spfSpBody.V.Z*100.f);
    for (int i=0; i < MAXV; i++)
        blackboxCurrent->dv[i] = lrintf(indiRun.dv[i] * 10.f);
    for (int i=0; i < MAXU; i++) {
        blackboxCurrent->u[i] = lrintf(indiRun.u[i] * UNIT_FLOAT_TO_SIGNED16VB);
        blackboxCurrent->u_state[i] = lrintf(indiRun.uState[i] * UNIT_FLOAT_TO_SIGNED16VB);
        blackboxCurrent->omega[i] = lrintf(indiRun.omega_fs[i]);
        blackboxCurrent->omegaUnfiltered[i] = lrintf(indiRun.omega[i]);
        blackboxCurrent->omega_dot[i] = lrintf(indiRun.omegaDot_fs[i] * 0.01f);
    }
#endif
#ifdef USE_LOCAL_POSITION
    blackboxCurrent->pos[0] = lrintf(posEstNed.V.X * METER_TO_MM);
    blackboxCurrent->pos[1] = lrintf(posEstNed.V.Y * METER_TO_MM);
    blackboxCurrent->pos[2] = lrintf(posEstNed.V.Z * METER_TO_MM);
    blackboxCurrent->extTime = lrintf(posMeasNed.time_us);
    blackboxCurrent->extPos[0] = lrintf(posMeasNed.pos.V.X * METER_TO_MM);
    blackboxCurrent->extPos[1] = lrintf(posMeasNed.pos.V.Y * METER_TO_MM);
    blackboxCurrent->extPos[2] = lrintf(posMeasNed.pos.V.Z * METER_TO_MM);
    blackboxCurrent->posSp[0] = lrintf(posSpNed.pos.V.X * METER_TO_MM);
    blackboxCurrent->posSp[1] = lrintf(posSpNed.pos.V.Y * METER_TO_MM);
    blackboxCurrent->posSp[2] = lrintf(posSpNed.pos.V.Z * METER_TO_MM);
    blackboxCurrent->vel[0] = lrintf(velEstNed.V.X * METER_TO_CM);
    blackboxCurrent->vel[1] = lrintf(velEstNed.V.Y * METER_TO_CM);
    blackboxCurrent->vel[2] = lrintf(velEstNed.V.Z * METER_TO_CM);
    blackboxCurrent->extVel[0] = lrintf(posMeasNed.vel.V.X * METER_TO_CM);
    blackboxCurrent->extVel[1] = lrintf(posMeasNed.vel.V.Y * METER_TO_CM);
    blackboxCurrent->extVel[2] = lrintf(posMeasNed.vel.V.Z * METER_TO_CM);
    blackboxCurrent->velSp[0] = lrintf(posSpNed.vel.V.X * METER_TO_CM);
    blackboxCurrent->velSp[1] = lrintf(posSpNed.vel.V.Y * METER_TO_CM);
    blackboxCurrent->velSp[2] = lrintf(posSpNed.vel.V.Z * METER_TO_CM);
    blackboxCurrent->accSp[0] = lrintf(accSpNedFromPos.V.X * METER_TO_CM);
    blackboxCurrent->accSp[1] = lrintf(accSpNedFromPos.V.Y * METER_TO_CM);
    blackboxCurrent->accSp[2] = lrintf(accSpNedFromPos.V.Z * METER_TO_CM);
    blackboxCurrent->extAtt[0] = lrintf(posMeasNed.att.angles.roll * 1000.f); // milirad
    blackboxCurrent->extAtt[1] = lrintf(posMeasNed.att.angles.pitch * 1000.f); // milirad
    blackboxCurrent->extAtt[2] = lrintf(posMeasNed.att.angles.yaw * 1000.f); // milirad
#ifdef USE_VIO_POSE
    blackboxCurrent->vioTime = lrintf(vioPosNed.time_us);
    blackboxCurrent->vioPos[0] = lrintf(vioPosNed.x * METER_TO_MM);
    blackboxCurrent->vioPos[1] = lrintf(vioPosNed.y * METER_TO_MM);
    blackboxCurrent->vioPos[2] = lrintf(vioPosNed.z * METER_TO_MM);
    blackboxCurrent->vioVel[0] = lrintf(vioPosNed.vx * METER_TO_CM);
    blackboxCurrent->vioVel[1] = lrintf(vioPosNed.vy * METER_TO_CM);
    blackboxCurrent->vioVel[2] = lrintf(vioPosNed.vz * METER_TO_CM);
    blackboxCurrent->vioQuat[0] = lrintf(vioPosNed.qw * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->vioQuat[1] = lrintf(vioPosNed.qx * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->vioQuat[2] = lrintf(vioPosNed.qy * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->vioQuat[3] = lrintf(vioPosNed.qz * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->vioRate[0] = lrintf(RADIANS_TO_DEGREES(vioPosNed.p));
    blackboxCurrent->vioRate[1] = lrintf(RADIANS_TO_DEGREES(vioPosNed.q));
    blackboxCurrent->vioRate[2] = lrintf(RADIANS_TO_DEGREES(vioPosNed.r));
#endif
#endif

#ifdef USE_EKF
    float *ekf_X = ekf_get_X();
    blackboxCurrent->ekf_pos[0] = lrintf(ekf_X[0] * METER_TO_MM);
    blackboxCurrent->ekf_pos[1] = lrintf(ekf_X[1] * METER_TO_MM);
    blackboxCurrent->ekf_pos[2] = lrintf(ekf_X[2] * METER_TO_MM);
    blackboxCurrent->ekf_vel[0] = lrintf(ekf_X[3] * METER_TO_CM);
    blackboxCurrent->ekf_vel[1] = lrintf(ekf_X[4] * METER_TO_CM);
    blackboxCurrent->ekf_vel[2] = lrintf(ekf_X[5] * METER_TO_CM);
    blackboxCurrent->ekf_att[0] = lrintf(ekf_X[6] * 1000); // milirad
    blackboxCurrent->ekf_att[1] = lrintf(ekf_X[7] * 1000); // milirad
    blackboxCurrent->ekf_att[2] = lrintf(ekf_X[8] * 1000); // milirad
    blackboxCurrent->ekf_acc_b[0] = lrintf(ekf_X[9] * 1000); // mm/s^2
    blackboxCurrent->ekf_acc_b[1] = lrintf(ekf_X[10] * 1000); // mm/s^2
    blackboxCurrent->ekf_acc_b[2] = lrintf(ekf_X[11] * 1000); // mm/s^2
    blackboxCurrent->ekf_gyro_b[0] = lrintf(RADIANS_TO_DEGREES(ekf_X[12])); // deg/s
    blackboxCurrent->ekf_gyro_b[1] = lrintf(RADIANS_TO_DEGREES(ekf_X[13])); // deg/s
    blackboxCurrent->ekf_gyro_b[2] = lrintf(RADIANS_TO_DEGREES(ekf_X[14])); // deg/s
#endif

#ifdef USE_LEARNER
    for (int motor = 0; motor < BLACKBOX_LEARNER_N; motor++) {
        for (int i = 0; i < BLACKBOX_LEARNER_MOTOR_RLS_N; i++) {
            blackboxCurrent->motor_rls_x[motor][i] = lrintf(1e3f*motorRls[motor].x[i]);
        }
    }
    for (int i = 0; i < 3; i++) {
        blackboxCurrent->imu_rls_x[i] = lrintf(1e3f*imuRls.x[i]);
    }
    for (int i = 0; i < MIN(BLACKBOX_LEARNER_N, fxRls[0].n); i++) {
        // specific force setpoints (fxRls 0, 1, 2)
        blackboxCurrent->fx_x_rls_x[i] = lrintf(1e3f*fxRls[0].x[i]);
        blackboxCurrent->fx_y_rls_x[i] = lrintf(1e3f*fxRls[0].x[i]);
        blackboxCurrent->fx_z_rls_x[i] = lrintf(1e3f*fxRls[0].x[i]);
    }
    // two for loops for the rate, because we need to ensure that the omega_dot
    // regressors start at BLACKBOX_LEARNER_N and not fxRateDotRls.n which is unknown
    // in the logs
    for (int i = 0; i < MIN(BLACKBOX_LEARNER_N, fxRls[3].n >> 1); i++) {
        // rateDot (fxRls 3, 4, 5)
        blackboxCurrent->fx_p_rls_x[i]                    = lrintf(1e3f*fxRls[3].x[i]);
        blackboxCurrent->fx_q_rls_x[i]                    = lrintf(1e3f*fxRls[4].x[i]);
        blackboxCurrent->fx_r_rls_x[i]                    = lrintf(1e3f*fxRls[5].x[i]);
        blackboxCurrent->fx_p_rls_x[BLACKBOX_LEARNER_N+i] = lrintf(1e3f*fxRls[3].x[(fxRls[3].n >> 1) + i]);
        blackboxCurrent->fx_q_rls_x[BLACKBOX_LEARNER_N+i] = lrintf(1e3f*fxRls[4].x[(fxRls[3].n >> 1) + i]);
        blackboxCurrent->fx_r_rls_x[BLACKBOX_LEARNER_N+i] = lrintf(1e3f*fxRls[5].x[(fxRls[3].n >> 1) + i]);
    }

    for (int motor = 0; motor < BLACKBOX_LEARNER_N; motor++) {
        blackboxCurrent->eVarsMotor[motor] = 0.1f * ((1 << 16) - 1) * motorRls[motor].fortescue.errorMV.variance;
        blackboxCurrent->lambdasMotor[motor] = UNIT_FLOAT_TO_UNSIGNED16VB * motorRls[motor].lambda;
    }

    for (int axis = 0; axis < 6; axis++) {
        blackboxCurrent->eVarsFx[axis] = 0.1f * ((1 << 16) - 1) * fxRls[axis].fortescue.errorMV.variance;
        blackboxCurrent->lambdasFx[axis] = UNIT_FLOAT_TO_UNSIGNED16VB * fxRls[axis].lambda;
    }

    for (int loop = 0; loop < LEARNER_LOOP_COUNT; loop++)
        blackboxCurrent->learnerGains[loop] = lrintf(10.f * learnRun.gains[loop]);

    blackboxCurrent->hoverAttitude[0] = lrintf(hoverAttitude.w * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->hoverAttitude[1] = lrintf(hoverAttitude.x * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->hoverAttitude[2] = lrintf(hoverAttitude.y * UNIT_FLOAT_TO_SIGNED16VB);
    blackboxCurrent->hoverAttitude[3] = lrintf(hoverAttitude.z * UNIT_FLOAT_TO_SIGNED16VB); // FRD and not FLU

    blackboxCurrent->learnerTimings[0] = learnerTimings.filters;
    blackboxCurrent->learnerTimings[1] = learnerTimings.imu;
    blackboxCurrent->learnerTimings[2] = learnerTimings.fx;
    blackboxCurrent->learnerTimings[3] = learnerTimings.motor;
    blackboxCurrent->learnerTimings[4] = learnerTimings.gains;
    blackboxCurrent->learnerTimings[5] = learnerTimings.updating;
    blackboxCurrent->learnerTimings[6] = learnerTimings.hover;
#endif

#else
    UNUSED(currentTimeUs);
#endif // UNIT_TEST
}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    } else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */

    // On our first call we need to print the name of the header and a colon
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true; // Try again later
        }

        blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

        xmitState.u.fieldIndex++;
        needComma = false;
    }

    // The longest we expect an integer to be as a string:
    const uint32_t LONGEST_INTEGER_STRLEN = 2;

    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            // First (over)estimate the length of the string we want to print

            int32_t bytesToWrite = 1; // Leading comma

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
            } else {
                //The other headers are integers
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }

            // Now perform the write if the buffer is large enough
            if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
                // Ran out of space!
                return true;
            }

            blackboxHeaderBudget -= bytesToWrite;

            if (needComma) {
                blackboxWrite(',');
            } else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                blackboxWriteString(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                //The other headers are integers
                blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
        blackboxHeaderBudget--;
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

// Buf must be at least FORMATTED_DATE_TIME_BUFSIZE
STATIC_UNIT_TESTED char *blackboxGetStartDateTime(char *buf)
{
#ifdef USE_RTC_TIME
    dateTime_t dt;
    // rtcGetDateTime will fill dt with 0000-01-01T00:00:00
    // when time is not known.
    rtcGetDateTime(&dt);
    dateTimeFormatLocal(buf, &dt);
#else
    buf = "0000-01-01T00:00:00.000";
#endif

    return buf;
}

#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                blackboxPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo(void)
{
#ifndef UNIT_TEST
    const uint16_t motorOutputLowInt = lrintf(getMotorOutputLow());
    const uint16_t motorOutputHighInt = lrintf(getMotorOutputHigh());

    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

    char buf[FORMATTED_DATE_TIME_BUFSIZE];

#ifdef USE_RC_SMOOTHING_FILTER
    rcSmoothingFilter_t *rcSmoothingData = getRcSmoothingData();
#endif

#ifdef USE_INDI
    const indiProfile_t *indiProfile = indiProfiles(systemConfig()->indiProfileIndex);
#endif
#ifdef USE_POS_CTL
    const positionProfile_t *posProfile = positionProfiles(systemConfig()->positionProfileIndex);
#endif

    const controlRateConfig_t *currentControlRateProfile = controlRateProfiles(systemConfig()->activeRateProfile);
    switch (xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Cleanflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    FC_FIRMWARE_NAME, FC_VERSION_STRING, shortGitRevision, targetName);
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                buildDate, buildTime);
#ifdef USE_BOARD_INFO
        BLACKBOX_PRINT_HEADER_LINE("Board information", "%s %s",            getManufacturerId(), getBoardName());
#endif
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              blackboxGetStartDateTime(buf));
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      pilotConfig()->craftName);
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      blackboxIInterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      blackboxPInterval);
        BLACKBOX_PRINT_HEADER_LINE("P ratio", "%d",                         (uint16_t)(blackboxIInterval / blackboxPInterval));
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     motorConfig()->minthrottle);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     motorConfig()->maxthrottle);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     castFloatBytesToInt(1.0f));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  motorOutputLowInt, motorOutputHighInt);
#if defined(USE_ACC)
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          acc.dev.acc_1G);
#endif

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
                blackboxPrintfHeaderLine("vbat_scale", "%u", voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale);
            } else {
                xmitState.headerIndex += 2; // Skip the next two vbat fields too
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u",           batteryConfig()->vbatmincellvoltage,
                                                                            batteryConfig()->vbatwarningcellvoltage,
                                                                            batteryConfig()->vbatmaxcellvoltage);
        BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u",                         vbatReference);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
                blackboxPrintfHeaderLine("currentSensor", "%d,%d",currentSensorADCConfig()->offset, currentSensorADCConfig()->scale);
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        gyro.sampleLooptime);
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_PID_PROCESS_DENOM, "%d",      activePidLoopDenom);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THR_MID, "%d",                currentControlRateProfile->thrMid8);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THR_EXPO, "%d",               currentControlRateProfile->thrExpo8);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_TPA_MODE, "%d",               currentPidProfile->tpa_mode);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_TPA_RATE, "%d",               currentPidProfile->tpa_rate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_TPA_BREAKPOINT, "%d",         currentPidProfile->tpa_breakpoint);
        BLACKBOX_PRINT_HEADER_LINE("rc_rates", "%d,%d,%d",                  currentControlRateProfile->rcRates[ROLL],
                                                                            currentControlRateProfile->rcRates[PITCH],
                                                                            currentControlRateProfile->rcRates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rc_expo", "%d,%d,%d",                   currentControlRateProfile->rcExpo[ROLL],
                                                                            currentControlRateProfile->rcExpo[PITCH],
                                                                            currentControlRateProfile->rcExpo[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rates", "%d,%d,%d",                     currentControlRateProfile->rates[ROLL],
                                                                            currentControlRateProfile->rates[PITCH],
                                                                            currentControlRateProfile->rates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rate_limits", "%d,%d,%d",               currentControlRateProfile->rate_limit[ROLL],
                                                                            currentControlRateProfile->rate_limit[PITCH],
                                                                            currentControlRateProfile->rate_limit[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rollPID", "%d,%d,%d",                   currentPidProfile->pid[PID_ROLL].P,
                                                                            currentPidProfile->pid[PID_ROLL].I,
                                                                            currentPidProfile->pid[PID_ROLL].D);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID", "%d,%d,%d",                  currentPidProfile->pid[PID_PITCH].P,
                                                                            currentPidProfile->pid[PID_PITCH].I,
                                                                            currentPidProfile->pid[PID_PITCH].D);
        BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d",                    currentPidProfile->pid[PID_YAW].P,
                                                                            currentPidProfile->pid[PID_YAW].I,
                                                                            currentPidProfile->pid[PID_YAW].D);
        BLACKBOX_PRINT_HEADER_LINE("levelPID", "%d,%d,%d",                  currentPidProfile->pid[PID_LEVEL].P,
                                                                            currentPidProfile->pid[PID_LEVEL].I,
                                                                            currentPidProfile->pid[PID_LEVEL].D);
        BLACKBOX_PRINT_HEADER_LINE("magPID", "%d",                          currentPidProfile->pid[PID_MAG].P);
#ifdef USE_D_MIN
        BLACKBOX_PRINT_HEADER_LINE("d_min", "%d,%d,%d",                     currentPidProfile->d_min[ROLL],
                                                                            currentPidProfile->d_min[PITCH],
                                                                            currentPidProfile->d_min[YAW]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_D_MAX_GAIN, "%d",             currentPidProfile->d_min_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_D_MAX_ADVANCE, "%d",          currentPidProfile->d_min_advance);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF1_TYPE, "%d",        currentPidProfile->dterm_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF1_STATIC_HZ, "%d",   currentPidProfile->dterm_lpf1_static_hz);
#ifdef USE_DYN_LPF
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf1_dyn_hz", "%d,%d",            currentPidProfile->dterm_lpf1_dyn_min_hz,
                                                                            currentPidProfile->dterm_lpf1_dyn_max_hz);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF2_TYPE, "%d",        currentPidProfile->dterm_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF2_STATIC_HZ, "%d",   currentPidProfile->dterm_lpf2_static_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_YAW_LOWPASS_HZ, "%d",         currentPidProfile->yaw_lowpass_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_NOTCH_HZ, "%d",         currentPidProfile->dterm_notch_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_NOTCH_CUTOFF, "%d",     currentPidProfile->dterm_notch_cutoff);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ITERM_WINDUP, "%d",           currentPidProfile->itermWindupPointPercent);
#if defined(USE_ITERM_RELAX)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ITERM_RELAX, "%d",            currentPidProfile->iterm_relax);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ITERM_RELAX_TYPE, "%d",       currentPidProfile->iterm_relax_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ITERM_RELAX_CUTOFF, "%d",     currentPidProfile->iterm_relax_cutoff);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_PID_AT_MIN_THROTTLE, "%d",    currentPidProfile->pidAtMinThrottle);

        // Betaflight PID controller parameters
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ANTI_GRAVITY_GAIN, "%d",      currentPidProfile->anti_gravity_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ANTI_GRAVITY_CUTOFF_HZ, "%d",    currentPidProfile->anti_gravity_cutoff_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ANTI_GRAVITY_P_GAIN, "%d",    currentPidProfile->anti_gravity_p_gain);

#ifdef USE_ABSOLUTE_CONTROL
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ABS_CONTROL_GAIN, "%d",       currentPidProfile->abs_control_gain);
#endif
#ifdef USE_INTEGRATED_YAW_CONTROL
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_USE_INTEGRATED_YAW, "%d",     currentPidProfile->use_integrated_yaw);
#endif
        BLACKBOX_PRINT_HEADER_LINE("ff_weight", "%d,%d,%d",                 currentPidProfile->pid[PID_ROLL].F,
                                                                            currentPidProfile->pid[PID_PITCH].F,
                                                                            currentPidProfile->pid[PID_YAW].F);
#ifdef USE_FEEDFORWARD
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_TRANSITION, "%d",     currentPidProfile->feedforward_transition);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_AVERAGING, "%d",      currentPidProfile->feedforward_averaging);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_SMOOTH_FACTOR, "%d",  currentPidProfile->feedforward_smooth_factor);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_JITTER_FACTOR, "%d",  currentPidProfile->feedforward_jitter_factor);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_BOOST, "%d",          currentPidProfile->feedforward_boost);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_FEEDFORWARD_MAX_RATE_LIMIT, "%d", currentPidProfile->feedforward_max_rate_limit);
#endif

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_LIMIT_YAW, "%d",          currentPidProfile->yawRateAccelLimit);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_LIMIT, "%d",              currentPidProfile->rateAccelLimit);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_PIDSUM_LIMIT, "%d",           currentPidProfile->pidSumLimit);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_PIDSUM_LIMIT_YAW, "%d",       currentPidProfile->pidSumLimitYaw);
        // End of Betaflight controller parameters

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DEADBAND, "%d",               rcControlsConfig()->deadband);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_YAW_DEADBAND, "%d",           rcControlsConfig()->yaw_deadband);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_HARDWARE_LPF, "%d",      gyroConfig()->gyro_hardware_lpf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF1_TYPE, "%d",         gyroConfig()->gyro_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF1_STATIC_HZ, "%d",    gyroConfig()->gyro_lpf1_static_hz);
#ifdef USE_DYN_LPF
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf1_dyn_hz", "%d,%d",             gyroConfig()->gyro_lpf1_dyn_min_hz,
                                                                            gyroConfig()->gyro_lpf1_dyn_max_hz);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF2_TYPE, "%d",         gyroConfig()->gyro_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF2_STATIC_HZ, "%d",    gyroConfig()->gyro_lpf2_static_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz", "%d,%d",                gyroConfig()->gyro_soft_notch_hz_1,
                                                                            gyroConfig()->gyro_soft_notch_hz_2);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff", "%d,%d",            gyroConfig()->gyro_soft_notch_cutoff_1,
                                                                            gyroConfig()->gyro_soft_notch_cutoff_2);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_TO_USE, "%d",            gyroConfig()->gyro_to_use);
#ifdef USE_DYN_NOTCH_FILTER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_MAX_HZ, "%d",       dynNotchConfig()->dyn_notch_max_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_COUNT, "%d",        dynNotchConfig()->dyn_notch_count);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_Q, "%d",            dynNotchConfig()->dyn_notch_q);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_MIN_HZ, "%d",       dynNotchConfig()->dyn_notch_min_hz);
#endif
#ifdef USE_DSHOT_TELEMETRY
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DSHOT_BIDIR, "%d",            motorConfig()->dev.useDshotTelemetry);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_POLES, "%d",            motorConfig()->motorPoleCount);
#endif
#ifdef USE_RPM_FILTER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RPM_FILTER_HARMONICS, "%d",     rpmFilterConfig()->rpm_filter_harmonics);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RPM_FILTER_Q, "%d",             rpmFilterConfig()->rpm_filter_q);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RPM_FILTER_MIN_HZ, "%d",        rpmFilterConfig()->rpm_filter_min_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RPM_FILTER_FADE_RANGE_HZ, "%d", rpmFilterConfig()->rpm_filter_fade_range_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RPM_FILTER_LPF_HZ, "%d",        rpmFilterConfig()->rpm_filter_lpf_hz);
#endif
#if defined(USE_ACC)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_LPF_HZ, "%d",        (int)(accelerometerConfig()->acc_lpf_hz * 100.0f));
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_HARDWARE, "%d",            accelerometerConfig()->acc_hardware);
#endif
#ifdef USE_BARO
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_BARO_HARDWARE, "%d",           barometerConfig()->baro_hardware);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_ALTITUDE_SOURCE, "%d",      positionConfig()->altitude_source);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_ALTITUDE_PREFER_BARO, "%d", positionConfig()->altitude_prefer_baro);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_ALTITUDE_LPF, "%d",         positionConfig()->altitude_lpf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_ALTITUDE_D_LPF, "%d",       positionConfig()->altitude_d_lpf);

#ifdef USE_MAG
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MAG_HARDWARE, "%d",           compassConfig()->mag_hardware);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_CAL_ON_FIRST_ARM, "%d",  armingConfig()->gyro_cal_on_first_arm);
        BLACKBOX_PRINT_HEADER_LINE("airmode_activate_throttle", "%d",       rxConfig()->airModeActivateThreshold);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SERIAL_RX_PROVIDER, "%d",     rxConfig()->serialrx_provider);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_USE_UNSYNCED_PWM, "%d",       motorConfig()->dev.useUnsyncedPwm);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_PWM_PROTOCOL, "%d",     motorConfig()->dev.motorPwmProtocol);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_PWM_RATE, "%d",         motorConfig()->dev.motorPwmRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DSHOT_IDLE_VALUE, "%d",       motorConfig()->digitalIdleOffsetValue);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DEBUG_MODE, "%d",             debugMode);
        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        featureConfig()->enabledFeatures);

#ifdef USE_RC_SMOOTHING_FILTER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING, "%d",                      rxConfig()->rc_smoothing_mode);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR, "%d",          rxConfig()->rc_smoothing_auto_factor_rpy);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR_THROTTLE, "%d", rxConfig()->rc_smoothing_auto_factor_throttle);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_FEEDFORWARD_CUTOFF, "%d", rcSmoothingData->ffCutoffSetting);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_SETPOINT_CUTOFF, "%d",    rcSmoothingData->setpointCutoffSetting);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_THROTTLE_CUTOFF, "%d",    rcSmoothingData->throttleCutoffSetting);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_DEBUG_AXIS, "%d",         rcSmoothingData->debugAxis);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RC_SMOOTHING_ACTIVE_CUTOFFS, "%d,%d,%d", rcSmoothingData->feedforwardCutoffFrequency,
                                                                            rcSmoothingData->setpointCutoffFrequency,
                                                                            rcSmoothingData->throttleCutoffFrequency);
        BLACKBOX_PRINT_HEADER_LINE("rc_smoothing_rx_average", "%d",         rcSmoothingData->averageFrameTimeUs);
#endif // USE_RC_SMOOTHING_FILTER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RATES_TYPE, "%d",             currentControlRateProfile->rates_type);

        BLACKBOX_PRINT_HEADER_LINE("fields_disabled_mask", "%d",            blackboxConfig()->fields_disabled_mask);
        BLACKBOX_PRINT_HEADER_LINE("blackbox_high_resolution", "%d",        blackboxConfig()->high_resolution);

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_VBAT_SAG_COMPENSATION, "%d",   currentPidProfile->vbat_sag_compensation);
#endif

#if defined(USE_DYN_IDLE)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_IDLE_MIN_RPM, "%d",        currentPidProfile->dyn_idle_min_rpm);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_IDLE_P_GAIN, "%d",         currentPidProfile->dyn_idle_p_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_IDLE_I_GAIN, "%d",         currentPidProfile->dyn_idle_i_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_IDLE_D_GAIN, "%d",         currentPidProfile->dyn_idle_d_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_IDLE_MAX_INCREASE, "%d",   currentPidProfile->dyn_idle_max_increase);
#endif

#ifdef USE_SIMPLIFIED_TUNING
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_PIDS_MODE, "%d",            currentPidProfile->simplified_pids_mode);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_MASTER_MULTIPLIER, "%d",    currentPidProfile->simplified_master_multiplier);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_I_GAIN, "%d",               currentPidProfile->simplified_i_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_D_GAIN, "%d",               currentPidProfile->simplified_d_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_PI_GAIN, "%d",              currentPidProfile->simplified_pi_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_DMAX_GAIN, "%d",            currentPidProfile->simplified_dmin_ratio);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_FEEDFORWARD_GAIN, "%d",     currentPidProfile->simplified_feedforward_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_PITCH_D_GAIN, "%d",         currentPidProfile->simplified_roll_pitch_ratio);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_PITCH_PI_GAIN, "%d",        currentPidProfile->simplified_pitch_pi_gain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_DTERM_FILTER, "%d",         currentPidProfile->simplified_dterm_filter);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_DTERM_FILTER_MULTIPLIER, "%d", currentPidProfile->simplified_dterm_filter_multiplier);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_GYRO_FILTER, "%d",             gyroConfig()->simplified_gyro_filter);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SIMPLIFIED_GYRO_FILTER_MULTIPLIER, "%d",  gyroConfig()->simplified_gyro_filter_multiplier);
#endif

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_OUTPUT_LIMIT, "%d",        currentPidProfile->motor_output_limit);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROTTLE_LIMIT_TYPE, "%d",       currentControlRateProfile->throttle_limit_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROTTLE_LIMIT_PERCENT, "%d",    currentControlRateProfile->throttle_limit_percent);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROTTLE_BOOST, "%d",            currentPidProfile->throttle_boost);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROTTLE_BOOST_CUTOFF, "%d",     currentPidProfile->throttle_boost_cutoff)

#ifdef USE_GPS
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_PROVIDER, "%d",               gpsConfig()->provider)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_SET_HOME_POINT_ONCE, "%d",    gpsConfig()->gps_set_home_point_once)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_USE_3D_SPEED, "%d",           gpsConfig()->gps_use_3d_speed)

#ifdef USE_GPS_RESCUE
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_MIN_START_DIST, "%d",  gpsRescueConfig()->minRescueDth)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_ALT_MODE, "%d",        gpsRescueConfig()->altitudeMode)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_INITIAL_CLIMB, "%d",   gpsRescueConfig()->rescueAltitudeBufferM)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_ASCEND_RATE, "%d",     gpsRescueConfig()->ascendRate)

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_RETURN_ALT, "%d",      gpsRescueConfig()->initialAltitudeM)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_RETURN_SPEED, "%d",    gpsRescueConfig()->rescueGroundspeed)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_MAX_RESCUE_ANGLE, "%d", gpsRescueConfig()->maxRescueAngle)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_ROLL_MIX, "%d",        gpsRescueConfig()->rollMix)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_PITCH_CUTOFF, "%d",    gpsRescueConfig()->pitchCutoffHz)

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_DESCENT_DIST, "%d",    gpsRescueConfig()->descentDistanceM)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_DESCEND_RATE, "%d",    gpsRescueConfig()->descendRate)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_LANDING_ALT, "%d",     gpsRescueConfig()->targetLandingAltitudeM)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_DISARM_THRESHOLD, "%d", gpsRescueConfig()->disarmThreshold)

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_MIN, "%d",    gpsRescueConfig()->throttleMin)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_MAX, "%d",    gpsRescueConfig()->throttleMax)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_HOVER, "%d",  gpsRescueConfig()->throttleHover)

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_SANITY_CHECKS, "%d",   gpsRescueConfig()->sanityChecks)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_MIN_SATS, "%d",        gpsRescueConfig()->minSats)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_ALLOW_ARMING_WITHOUT_FIX, "%d", gpsRescueConfig()->allowArmingWithoutFix)

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_P, "%d",      gpsRescueConfig()->throttleP)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_I, "%d",      gpsRescueConfig()->throttleI)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_THROTTLE_D, "%d",      gpsRescueConfig()->throttleD)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_VELOCITY_P, "%d",      gpsRescueConfig()->velP)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_VELOCITY_I, "%d",      gpsRescueConfig()->velI)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_VELOCITY_D, "%d",      gpsRescueConfig()->velD)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_YAW_P, "%d",           gpsRescueConfig()->yawP)

#ifdef USE_MAG
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GPS_RESCUE_USE_MAG, "%d",         gpsRescueConfig()->useMag)
#endif
#endif
#endif

#ifdef USE_INDI
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_GAINS, "%d,%d,%d",                indiProfile->attGains[0],
                                                                                              indiProfile->attGains[1],
                                                                                              indiProfile->attGains[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RATE_GAINS, "%d,%d,%d",                    indiProfile->rateGains[0],
                                                                                              indiProfile->rateGains[1],
                                                                                              indiProfile->rateGains[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_MAX_TILT_RATE, "%d",              indiProfile->attMaxTiltRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_MAX_YAW_RATE, "%d",               indiProfile->attMaxYawRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MAX_RATE_SETPOINT, "%d,%d,%d",             indiProfile->maxRateSp[0],
                                                                                              indiProfile->maxRateSp[1],
                                                                                              indiProfile->maxRateSp[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_USE_COORDINATED_YAW, "%d",          indiProfile->manualUseCoordinatedYaw);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_MAX_UPWARDS_ACCEL, "%d",            indiProfile->manualMaxUpwardsSpf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_MAX_TILT, "%d",                     indiProfile->manualMaxTilt);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_INCREMENT, "%d",                       indiProfile->useIncrement);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_RPM_DOT_FEEDBACK, "%d",                indiProfile->useRpmDotFeedback);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_NUM, "%d",                             indiProfile->actNum);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_TIME_CONSTANT_MS, "%d,%d,%d,%d",       indiProfile->actTimeConstMs[0],
                                                                                              indiProfile->actTimeConstMs[1],
                                                                                              indiProfile->actTimeConstMs[2],
                                                                                              indiProfile->actTimeConstMs[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_MAX_RPM, "%d,%d,%d,%d",                indiProfile->actMaxRpm[0],
                                                                                              indiProfile->actMaxRpm[1],
                                                                                              indiProfile->actMaxRpm[2],
                                                                                              indiProfile->actMaxRpm[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_NONLINEARITY, "%d,%d,%d,%d",           indiProfile->actNonlinearity[0],
                                                                                              indiProfile->actNonlinearity[1],
                                                                                              indiProfile->actNonlinearity[2],
                                                                                              indiProfile->actNonlinearity[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_LIMIT, "%d,%d,%d,%d",                  indiProfile->actLimit[0],
                                                                                              indiProfile->actLimit[1],
                                                                                              indiProfile->actLimit[2],
                                                                                              indiProfile->actLimit[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FX, "%d,%d,%d,%d",                  indiProfile->actG1_fx[0],
                                                                                              indiProfile->actG1_fx[1],
                                                                                              indiProfile->actG1_fx[2],
                                                                                              indiProfile->actG1_fx[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FY, "%d,%d,%d,%d",                  indiProfile->actG1_fy[0],
                                                                                              indiProfile->actG1_fy[1],
                                                                                              indiProfile->actG1_fy[2],
                                                                                              indiProfile->actG1_fy[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FZ, "%d,%d,%d,%d",                  indiProfile->actG1_fz[0],
                                                                                              indiProfile->actG1_fz[1],
                                                                                              indiProfile->actG1_fz[2],
                                                                                              indiProfile->actG1_fz[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_ROLL, "%d,%d,%d,%d",                indiProfile->actG1_roll[0],
                                                                                              indiProfile->actG1_roll[1],
                                                                                              indiProfile->actG1_roll[2],
                                                                                              indiProfile->actG1_roll[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_PITCH, "%d,%d,%d,%d",               indiProfile->actG1_pitch[0],
                                                                                              indiProfile->actG1_pitch[1],
                                                                                              indiProfile->actG1_pitch[2],
                                                                                              indiProfile->actG1_pitch[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_YAW, "%d,%d,%d,%d",                 indiProfile->actG1_yaw[0],
                                                                                              indiProfile->actG1_yaw[1],
                                                                                              indiProfile->actG1_yaw[2],
                                                                                              indiProfile->actG1_yaw[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_ROLL, "%d,%d,%d,%d",                indiProfile->actG2_roll[0],
                                                                                              indiProfile->actG2_roll[1],
                                                                                              indiProfile->actG2_roll[2],
                                                                                              indiProfile->actG2_roll[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_PITCH, "%d,%d,%d,%d",               indiProfile->actG2_pitch[0],
                                                                                              indiProfile->actG2_pitch[1],
                                                                                              indiProfile->actG2_pitch[2],
                                                                                              indiProfile->actG2_pitch[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_YAW, "%d,%d,%d,%d",                 indiProfile->actG2_yaw[0],
                                                                                              indiProfile->actG2_yaw[1],
                                                                                              indiProfile->actG2_yaw[2],
                                                                                              indiProfile->actG2_yaw[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_SYNC_LOWPASS_HZ, "%d",                     indiProfile->imuSyncLp2Hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_AXES_WEIGHTS, "%d,%d,%d,%d,%d,%d",     indiProfile->wlsWv[0],
                                                                                              indiProfile->wlsWv[1],
                                                                                              indiProfile->wlsWv[2],
                                                                                              indiProfile->wlsWv[3],
                                                                                              indiProfile->wlsWv[4],
                                                                                              indiProfile->wlsWv[5]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_ACT_PENALTIES, "%d,%d,%d,%d",          indiProfile->wlsWu[0],
                                                                                              indiProfile->wlsWu[1],
                                                                                              indiProfile->wlsWu[2],
                                                                                              indiProfile->wlsWu[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_ACT_PREFERRED_STATE, "%d,%d,%d,%d",    indiProfile->u_pref[0],
                                                                                              indiProfile->u_pref[1],
                                                                                              indiProfile->u_pref[2],
                                                                                              indiProfile->u_pref[3]);
#endif
#ifdef USE_POS_CTL
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_HORIZONTAL_P, "%d",  posProfile->horz_p);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_HORIZONTAL_I, "%d",  posProfile->horz_i);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_HORIZONTAL_D, "%d",  posProfile->horz_d);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_HORIZONTAL_SPEED, "%d",  posProfile->horz_max_v);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_HORIZONTAL_ACCEL, "%d",  posProfile->horz_max_a);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_TILT, "%d",  posProfile->max_tilt);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_VERTICAL_P, "%d",  posProfile->vert_p);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_VERTICAL_I, "%d",  posProfile->vert_i);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_VERTICAL_D, "%d",  posProfile->vert_d);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_UPWARDS_SPEED, "%d",  posProfile->vert_max_v_up);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_DOWNWARDS_SPEED, "%d",  posProfile->vert_max_v_down);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_UPWARDS_ACCEL, "%d",  posProfile->vert_max_a_up);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_MAX_DOWNWARDS_ACCEL, "%d",  posProfile->vert_max_v_down);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_YAW_P, "%d",  posProfile->yaw_p);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_WEATHERVANE_P, "%d",  posProfile->weathervane_p);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_WEATHERVANE_MIN_V, "%d",  posProfile->weathervane_min_v);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_POSITION_THRUST_ATTENUATION, "%d",  posProfile->use_spf_attenuation);
#endif
#ifdef USE_EKF
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_USE_ATTITUDE_ESTIMATE, "%d",  ekfConfig()->use_attitude_estimate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_USE_POSITION_ESTIMATE, "%d",  ekfConfig()->use_position_estimate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_USE_ANGLE_MEASUREMENTS, "%d,%d,%d",  ekfConfig()->use_angle_measurements[0],
                                                                                       ekfConfig()->use_angle_measurements[1],
                                                                                       ekfConfig()->use_angle_measurements[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_PROC_NOISE_ACC, "%d,%d,%d",  ekfConfig()->proc_noise_acc[0],
                                                                                       ekfConfig()->proc_noise_acc[1],
                                                                                       ekfConfig()->proc_noise_acc[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_PROC_NOISE_GYRO, "%d,%d,%d",  ekfConfig()->proc_noise_gyro[0],
                                                                                       ekfConfig()->proc_noise_gyro[1],
                                                                                       ekfConfig()->proc_noise_gyro[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_MEAS_NOISE_POSITION, "%d,%d,%d",  ekfConfig()->meas_noise_position[0],
                                                                                       ekfConfig()->meas_noise_position[1],
                                                                                       ekfConfig()->meas_noise_position[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_MEAS_NOISE_ANGLES, "%d,%d,%d",  ekfConfig()->meas_noise_angles[0],
                                                                                       ekfConfig()->meas_noise_angles[1],
                                                                                       ekfConfig()->meas_noise_angles[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_EKF_MEAS_DELAY, "%d",  ekfConfig()->meas_delay);
#endif
#ifdef USE_CATAPULT
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_TARGET_ALTITUDE, "%d",  catapultConfig()->altitude);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_TARGET_X_NED, "%d",  catapultConfig()->xNed);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_TARGET_Y_NED, "%d",  catapultConfig()->yNed);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_ROTATION_ROLL, "%d",  catapultConfig()->rotationRoll);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_ROTATION_PITCH, "%d",  catapultConfig()->rotationPitch);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_ROTATION_YAW, "%d",  catapultConfig()->rotationYaw);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_ROTATION_RANDOMIZE, "%d",  catapultConfig()->randomizeRotation);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_ROTATION_TIME, "%d",  catapultConfig()->rotationTimeMs);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_CATAPULT_UPWARDS_ACCEL, "%d",  catapultConfig()->upwardsAccel);
#endif
#ifdef USE_THROW_TO_ARM
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_ACC_HIGH, "%d",  throwConfig()->accHighThresh);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_ACC_CLIP, "%d",  throwConfig()->accClipThresh);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_ACC_LOW, "%d",  throwConfig()->accLowAgainThresh);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_GYRO_HIGH, "%d",  throwConfig()->gyroHighThresh);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_MOMENTUM_THRESH, "%d",  throwConfig()->momentumThresh);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_THROW_TO_ARM_RELEASE_DELAY_MS, "%d",  throwConfig()->releaseDelayMs);
#endif
#ifdef USE_LEARNER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_MODE, "%d",  learnerConfig()->mode);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_NUM_ACT, "%d",  learnerConfig()->numAct);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_DELAY_TIME_MS, "%d",  learnerConfig()->delayMs);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_STEP_TIME_MS, "%d",  learnerConfig()->stepMs);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_RAMP_TIME_MS, "%d",  learnerConfig()->rampMs);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_OVERLAP_TIME_MS, "%d",  learnerConfig()->overlapMs);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_STEP_AMPLITUDE, "%d",  learnerConfig()->stepAmp);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_RAMP_AMPLITUDE, "%d",  learnerConfig()->rampAmp);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_GYRO_MAX, "%d",  learnerConfig()->gyroMax);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_IMU_LOWPASS_HZ, "%d",  learnerConfig()->imuFiltHz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_FX_LOWPASS_HZ, "%d",  learnerConfig()->fxFiltHz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_MOTOR_LOWPASS_HZ, "%d",  learnerConfig()->motorFiltHz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_ZETA_RATE, "%d",  learnerConfig()->zetaRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_ZETA_ATTITUDE, "%d",  learnerConfig()->zetaAttitude);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_ZETA_VELOCITY, "%d",  learnerConfig()->zetaVelocity);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_ZETA_POSITION, "%d",  learnerConfig()->zetaPosition);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_APPLY_INDI, "%d",  learnerConfig()->applyIndiProfileAfterQuery);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_LEARNER_APPLY_POSITION, "%d",  learnerConfig()->applyPositionProfileAfterQuery);
#endif
#ifdef USE_NN_CONTROL
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_NN_RATE_DENOM, "%d",  nnConfig()->rate_denom);
#endif

        default:
            return true;
    }

    xmitState.headerIndex++;
#endif // UNIT_TEST
    return false;
}

/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    // Only allow events to be logged after headers have been written
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED)) {
        return;
    }

    //Shared header for event frames
    blackboxWrite('E');
    blackboxWrite(event);

    //Now serialize the data for this specific frame type
    switch (event) {
    case FLIGHT_LOG_EVENT_SYNC_BEEP:
        blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
    case FLIGHT_LOG_EVENT_FLIGHTMODE: // New flightmode flags write
        blackboxWriteUnsignedVB(data->flightMode.flags);
        blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
    case FLIGHT_LOG_EVENT_DISARM:
        blackboxWriteUnsignedVB(data->disarm.reason);
        break;
    case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflightAdjustment.floatFlag) {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
            blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
        } else {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction);
            blackboxWriteSignedVB(data->inflightAdjustment.newValue);
        }
        break;
    case FLIGHT_LOG_EVENT_LOGGING_RESUME:
        blackboxWriteUnsignedVB(data->loggingResume.logIteration);
        blackboxWriteUnsignedVB(data->loggingResume.currentTime);
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        blackboxWriteString("End of log");
        blackboxWrite(0);
        break;
    default:
        break;
    }
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        flightLogEvent_syncBeep_t eventData;
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *)&eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flags, &rcModeActivationMask, sizeof(eventData.flags));
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t *)&eventData);
    }
}

STATIC_UNIT_TESTED bool blackboxShouldLogPFrame(void)
{
    return blackboxPFrameIndex == 0 && blackboxPInterval != 0;
}

STATIC_UNIT_TESTED bool blackboxShouldLogIFrame(void)
{
    return blackboxLoopIndex == 0;
}

/*
 * If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
 * GPS home position.
 *
 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
 * still be interpreted correctly.
 */
#ifdef USE_GPS
STATIC_UNIT_TESTED bool blackboxShouldLogGpsHomeFrame(void)
{
    if ((GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
        || (blackboxPFrameIndex == blackboxIInterval / 2 && blackboxIFrameIndex % 128 == 0)) && isFieldEnabled(FIELD_SELECT(GPS))) {
        return true;
    }
    return false;
}
#endif // GPS

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
STATIC_UNIT_TESTED void blackboxAdvanceIterationTimers(void)
{
    ++blackboxSlowFrameIterationTimer;
    ++blackboxIteration;

    if (++blackboxLoopIndex >= blackboxIInterval) {
        blackboxLoopIndex = 0;
        blackboxIFrameIndex++;
        blackboxPFrameIndex = 0;
    } else if (++blackboxPFrameIndex >= blackboxPInterval) {
        blackboxPFrameIndex = 0;
    }
}

// Called once every FC loop in order to log the current state
STATIC_UNIT_TESTED void blackboxLogIteration(timeUs_t currentTimeUs)
{
    // Write a keyframe every blackboxIInterval frames so we can resynchronise upon missing frames
    if (blackboxShouldLogIFrame()) {
        /*
         * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
         * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
         */
        if (blackboxIsOnlyLoggingIntraframes()) {
            writeSlowFrameIfNeeded();
        }

        loadMainState(currentTimeUs);
        writeIntraframe();
    } else {
        blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode(); // Check for FlightMode status change event

        if (blackboxShouldLogPFrame()) {
            /*
             * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
             * So only log slow frames during loop iterations where we log a main frame.
             */
            writeSlowFrameIfNeeded();

            loadMainState(currentTimeUs);
            writeInterframe();
        }
#ifdef USE_GPS
        if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
            if (blackboxShouldLogGpsHomeFrame()) {
                writeGPSHomeFrame();
                writeGPSFrame(currentTimeUs);
            } else if (gpsSol.numSat != gpsHistory.GPS_numSat
                    || gpsSol.llh.lat != gpsHistory.GPS_coord[GPS_LATITUDE]
                    || gpsSol.llh.lon != gpsHistory.GPS_coord[GPS_LONGITUDE]) {
                //We could check for velocity changes as well but I doubt it changes independent of position
                writeGPSFrame(currentTimeUs);
            }
        }
#endif
    }

    //Flush every iteration so that our runtime variance is minimized
    blackboxDeviceFlush();
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
#include "flight/throw.h"
void blackboxUpdate(timeUs_t currentTimeUs)
{
    static BlackboxState cacheFlushNextState;

    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (ARMING_FLAG(ARMED) || (throwState >= THROW_STATE_WAITING_FOR_THROW)) {
            blackboxOpen();
            blackboxStart();
        }
#ifdef USE_FLASHFS
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_START_ERASE);
        }
#endif
        break;
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        if (blackboxDeviceBeginLog()) {
            blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

        /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
         * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
         */
        if (millis() > xmitState.u.startTime + 100) {
            if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                    blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                    blackboxHeaderBudget--;
                }
                if (blackboxHeader[xmitState.headerIndex] == '\0') {
                    blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                }
            }
        }
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAYLEN(blackboxMainFields),
                &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
#ifdef USE_GPS
            if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADER);
            } else
#endif
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#ifdef USE_GPS
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAYLEN(blackboxGpsHFields),
                NULL, NULL) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAYLEN(blackboxGpsGFields),
                &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#endif
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAYLEN(blackboxSlowFields),
                NULL, NULL)) {
            cacheFlushNextState = BLACKBOX_STATE_SEND_SYSINFO;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (blackboxWriteSysinfo()) {
            /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
            cacheFlushNextState = BLACKBOX_STATE_RUNNING;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        if (blackboxDeviceFlushForceComplete()) {
            blackboxSetState(cacheFlushNextState);
        }
        break;
    case BLACKBOX_STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOX) && blackboxShouldLogIFrame()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            flightLogEvent_loggingResume_t resume;

            resume.logIteration = blackboxIteration;
            resume.currentTime = currentTimeUs;

            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
            blackboxSetState(BLACKBOX_STATE_RUNNING);

            blackboxLogIteration(currentTimeUs);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
        // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
        if (blackboxModeActivationConditionPresent && !IS_RC_MODE_ACTIVE(BOXBLACKBOX) && !startedLoggingInTestMode) {
            blackboxSetState(BLACKBOX_STATE_PAUSED);
        } else if ((!ARMING_FLAG(ARMED)) && (throwState == THROW_STATE_IDLE) && !startedLoggingInTestMode) {
            blackboxFinish();
        } else {
            blackboxLogIteration(currentTimeUs);
        }
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        //On entry of this state, startTime is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
        if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlushForce())) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_STATE_START_ERASE:
        blackboxEraseAll();
        blackboxSetState(BLACKBOX_STATE_ERASING);
        beeper(BEEPER_BLACKBOX_ERASE);
        break;
    case BLACKBOX_STATE_ERASING:
        if (isBlackboxErased()) {
            //Done erasing
            blackboxSetState(BLACKBOX_STATE_ERASED);
            beeper(BEEPER_BLACKBOX_ERASE);
        }
        break;
    case BLACKBOX_STATE_ERASED:
        if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
    break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (isBlackboxDeviceFull()) {
#ifdef USE_FLASHFS
        if (blackboxState != BLACKBOX_STATE_ERASING
            && blackboxState != BLACKBOX_STATE_START_ERASE
            && blackboxState != BLACKBOX_STATE_ERASED)
#endif
        {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
            // ensure we reset the test mode flag if we stop due to full memory card
            if (startedLoggingInTestMode) {
                startedLoggingInTestMode = false;
            }
        }
    } else { // Only log in test mode if there is room!
        switch (blackboxConfig()->mode) {
        case BLACKBOX_MODE_MOTOR_TEST:
            // Handle Motor Test Mode
            if (inMotorTestMode()) {
                if (blackboxState==BLACKBOX_STATE_STOPPED) {
                    startInTestMode();
                }
            } else {
                if (blackboxState!=BLACKBOX_STATE_STOPPED) {
                    stopInTestMode();
                }
            }

            break;
        case BLACKBOX_MODE_ALWAYS_ON:
            if (blackboxState==BLACKBOX_STATE_STOPPED) {
                startInTestMode();
            }

            break;
        case BLACKBOX_MODE_NORMAL:
        default:

            break;
        }
    }
}

int blackboxCalculatePDenom(int rateNum, int rateDenom)
{
    return blackboxIInterval * rateNum / rateDenom;
}

uint8_t blackboxGetRateDenom(void)
{
    return blackboxPInterval;

}

uint16_t blackboxGetPRatio(void)
{
    return blackboxIInterval / blackboxPInterval;
}

uint8_t blackboxCalculateSampleRate(uint16_t pRatio)
{
    return LOG2(32000 / (targetPidLooptime * pRatio));
}

/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(void)
{
    blackboxResetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // targetPidLooptime is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptime is rounded for short looptimes
    blackboxIInterval = (uint16_t)(32 * 1000 / targetPidLooptime);

    blackboxPInterval = 1 << blackboxConfig()->sample_rate;
    if (blackboxPInterval > blackboxIInterval) {
        blackboxPInterval = 0; // log only I frames if logging frequency is too low
    }

    if (blackboxConfig()->device) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }
    blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds

    blackboxHighResolutionScale = blackboxConfig()->high_resolution ? 10.0f : 1.0f;
}
#endif
