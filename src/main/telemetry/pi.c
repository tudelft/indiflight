/*
(C) tblaha 2023
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

#define USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"

#define TELEMETRY_PI_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_PI_MAXRATE 50
#define TELEMETRY_PI_DELAY ((1000 * 1000) / TELEMETRY_PI_MAXRATE)

static serialPort_t *piPort = NULL;
static const serialPortConfig_t *portConfig;

static bool piTelemetryEnabled =  false;
static portSharing_e piPortSharing;

/* MAVLink datastream rates in Hz */
/*
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 5, //5Hz
    [MAV_DATA_STREAM_POSITION] = 2, //2Hz
    [MAV_DATA_STREAM_EXTRA1] = 10, //10Hz
    [MAV_DATA_STREAM_EXTRA2] = 10 //2Hz
};
*/

// #define MAXSTREAMS ARRAYLEN(piRates)


// static uint8_t mavTicks[MAXSTREAMS];
//static uint8_t piBuffer[PI_MAX_PACKET_LEN];
//static uint32_t lastPiMessage = 0;

// wrapper for serialWrite
static void serialWriter(uint8_t byte) { serialWrite(piPort, byte); }

/*
static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    uint8_t rate = (uint8_t) mavRates[streamNum];
    if (rate == 0) {
        return 0;
    }

    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE;
        }

        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }

    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}
*/

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

    //delay(500);
    //BLINK_ONCE;

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        //baudRateIndex = BAUD_500000;
        baudRateIndex = BAUD_921600;
    }

    //BLINK_ONCE;

    piPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_PI, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_PI_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    //BLINK_ONCE;

    if (!piPort) {
        return;
    }

    //BLINK_ONCE;

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
    piMsgImuTx.time_ms = millis();
    // piMsgImuTx.roll = gyro.gyroADCf[FD_ROLL]; // filtered with notches and lpf
    // piMsgImuTx.pitch = gyro.gyroADCf[FD_PITCH]; // filtered with notches and lpf
    // piMsgImuTx.yaw = gyro.gyroADCf[FD_YAW]; // filtered with notches and lpf
    // piMsgImuTx.x = acc.accADC[X];
    // piMsgImuTx.y = acc.accADC[Y];
    // piMsgImuTx.z = acc.accADC[Z];

    // change to a chill format for logging onto pi..
    // FLU to FRD
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(-gyro.gyroADCf[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(-gyro.gyroADCf[2]);
    piMsgImuTx.x = 9.81 * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.y = 9.81 *-((float)acc.accADC[1]) / ((float)acc.dev.acc_1G);
    piMsgImuTx.z = 9.81 *-((float)acc.accADC[2]) / ((float)acc.dev.acc_1G);
    
    piSendMsg(&piMsgImuTx, &serialWriter);

    // send dummy data to test serialization escaping
    /*
    test_cast.s.A = PI_STX_ESC;
    test_cast.s.B = PI_ESC_ESC;
    test_cast.s.C = PI_ESC;
    test_cast.s.D = PI_STX;
    int len = piMsgImuPack(
        piBuffer,
        millis(),
        //gyro.gyroADCf[FD_ROLL], // filtered with notches and lpf
        test_cast.f,
        gyro.gyroADCf[FD_PITCH],
        gyro.gyroADCf[FD_YAW],
        acc.accADC[X], // heavily filtered (25Hz?) because only used in the attitude loop, not gyro loop
        acc.accADC[Y],
        acc.accADC[Z]
    );
    */
}

void processPiTelemetry(void)
{
    // could do rate limiting with the stream stuffs above
    piSendIMU();
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
            piParse(&p_telem, serialRead(piPort));
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

    //uint32_t now = micros();
    //if ((now - lastPiMessage) >= TELEMETRY_PI_DELAY) {
        processPiTelemetry();
        processPiUplink();
    //    lastPiMessage = now;
    //}
}

#endif
