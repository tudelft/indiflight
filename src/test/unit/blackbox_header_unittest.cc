/*
 * This file is part of Indiflight.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Indiflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// blackbox.c is built with every Indiflight blackbox feature enabled (see
// blackbox_header_unittest_DEFINES in src/test/Makefile), so the field table
// is the largest any build can log.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "blackbox/blackbox.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "pg/motor.h"
    #include "drivers/serial.h"
    #include "flight/failsafe.h"
    #include "flight/mixer.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "sensors/battery.h"
    #include "sensors/gyro.h"
    #include "flight/throw.h"

    size_t blackboxMainFieldNameLineLength(void);
    unsigned blackboxMainFieldCount(void);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Longest header line the decoders accept. blackbox_decode
// (FLIGHT_LOG_MAX_FRAME_HEADER_LENGTH) and the configurator's Blackbox Viewer
// (FLIGHT_LOG_MAX_HEADER_LINE_LENGTH in flightlog_parser.js) both use 16384.
// A longer line is dropped, so the log has no field names and cannot be opened.
// The viewer allowed only 4096 until indiflight-configurator 6e4c8b9d, and
// Indiflight bff9b05c (344 fields, about 4.6 kB) went over that.
static const size_t DECODER_MAX_HEADER_LINE_LENGTH = 16384;

TEST(BlackboxHeaderTest, MainFieldNameLineFitsDecoders)
{
    const size_t len = blackboxMainFieldNameLineLength();
    printf("%u main fields, \"Field I name\" line is %zu of %zu characters\n",
           blackboxMainFieldCount(), len, DECODER_MAX_HEADER_LINE_LENGTH);

    // sanity: the Indiflight fields are compiled in (344 logged in a DarkO sim)
    EXPECT_GT(blackboxMainFieldCount(), 340u);
    EXPECT_LT(len, DECODER_MAX_HEADER_LINE_LENGTH);
}

// STUBS, as in blackbox_unittest.cc. Only the field table is under test.
extern "C" {

PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);

armingFlag_e armingFlags;
uint8_t stateFlags;
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
gpsSolutionData_t gpsSol;
int32_t GPS_home[3];

gyro_t gyro;

float motor_disarmed[MAX_SUPPORTED_MOTORS];
struct pidProfile_s;
struct pidProfile_s *currentPidProfile;
uint32_t targetPidLooptime;

boxBitmask_t rcModeActivationMask;

void mspSerialAllocatePorts(void) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
uint16_t getBatteryVoltageLatest(void) {return 0;}
uint8_t getMotorCount(void) {return 4;}
bool areMotorsRunning(void) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) {return false;}
bool isModeActivationConditionPresent(boxId_e) {return false;}
uint32_t millis(void) {return 0;}
uint32_t micros(void) {return 0;}
throwState_t throwState;
bool sensors(uint32_t) {return false;}
void serialWrite(serialPort_t *, uint8_t) {}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return false;}
bool featureIsEnabled(uint32_t) {return false;}
void mspSerialReleasePortIfAllocated(serialPort_t *) {}
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
serialPort_t *findSharedSerialPort(uint16_t , serialPortFunction_e ) {return NULL;}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e ) {return PORTSHARING_UNUSED;}
failsafePhase_e failsafePhase(void) {return FAILSAFE_IDLE;}
bool rxAreFlightChannelsValid(void) {return false;}
bool rxIsReceivingSignal(void) {return false;}
bool isRssiConfigured(void) {return false;}
float getMotorOutputLow(void) {return 0.0;}
float getMotorOutputHigh(void) {return 0.0;}
}
