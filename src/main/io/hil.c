/*
(C) tblaha 2023
 */
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(HIL_BUILD)

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
#include "io/hil.h"
#include "pi-protocol.h"
#include "pi-messages.h"
#include "flight/mixer_init.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#define HIL_INITIAL_PORT_MODE MODE_RXTX
//#define HIL_MAXRATE 50
//#define HIL_DELAY ((1000 * 1000) / TELEMETRY_PI_MAXRATE)

static serialPort_t *hilPort = NULL;
static const serialPortConfig_t *portConfig;

static bool hilEnabled =  false;
static portSharing_e hilPortSharing;

// input from simulation in natural units (deg/s, g, rpm)
hilInput_t hilInput;
//hilOutput_t hilOutput;

void freeHilPort(void)
{
    closeSerialPort(hilPort);
    hilPort = NULL;
    hilEnabled = false;
}

void initHil(void)
{
    portConfig = findSerialPortConfig(FUNCTION_HIL);
    hilPortSharing = determinePortSharing(portConfig, FUNCTION_HIL);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configureHilPort(void)
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

    hilPort = openSerialPort(portConfig->identifier, FUNCTION_HIL, NULL, NULL, baudRates[baudRateIndex], HIL_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    //BLINK_ONCE;

    if (!hilPort) {
        return;
    }

    //BLINK_ONCE;

    hilEnabled = true;
}

void checkHilState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!hilEnabled && telemetrySharedPort != NULL) {
            hilPort = telemetrySharedPort;
            hilEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(hilPortSharing);

        if (newTelemetryEnabledValue == hilEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configureHilPort();
        else
            freeHilPort();
    }
}

// wrapper for serialWrite
static void serialWriter(uint8_t byte) { serialWrite(hilPort, byte); }

void hilSendActuators(void)
{
    if ((!hilPort) || (!hilEnabled))
        return;

    piMsgHilOutTx.time_us = micros();
#define MOTOR_TO_HIL 32767
    piMsgHilOutTx.set_1 = (int16_t) (MOTOR_TO_HIL * constrainf(scaleRangef(motor[0], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f));
    piMsgHilOutTx.set_2 = (int16_t) (MOTOR_TO_HIL * constrainf(scaleRangef(motor[1], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f));
    piMsgHilOutTx.set_3 = (int16_t) (MOTOR_TO_HIL * constrainf(scaleRangef(motor[2], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f));
    piMsgHilOutTx.set_4 = (int16_t) (MOTOR_TO_HIL * constrainf(scaleRangef(motor[3], mixerRuntime.motorOutputLow, mixerRuntime.motorOutputHigh, 0.f, 1.f), 0.f, 1.f));

    //UNUSED(serialWriter);
    piSendMsg(&piMsgHilOutTx, &serialWriter);
}

//void processHilOutput(void)
//{
//    // could do rate limiting with the stream stuffs above
//    hilSendActuators();
//    //piSendIMU();
//    //serialWrite(hilPort, 'b');
//    //serialWrite(hilPort, '\n');
//}

pi_parse_states_t p_hil;

void processHilInput(void)
{
    if (hilPort) {
        while (serialRxBytesWaiting(hilPort)) {
            piParse(&p_hil, serialRead(hilPort));
            //LED1_TOGGLE;
            //cliPrintLinef("%c", c);
        }
    }

    // TODO: check if messages time updated
    // TODO: actually write this too the correct global variables
#define HIL_TO_DEGS 0.1f
#define HIL_TO_G 0.001f
#define HIL_TO_RPM 10.f
    if (piMsgHilInRxState != PI_MSG_RX_STATE_NONE) {
        hilInput.gyro[0] = HIL_TO_DEGS * piMsgHilInRx->gyro_x;
        hilInput.gyro[1] = HIL_TO_DEGS * piMsgHilInRx->gyro_y;
        hilInput.gyro[2] = HIL_TO_DEGS * piMsgHilInRx->gyro_z;
        hilInput.acc[0]  = HIL_TO_G * piMsgHilInRx->acc_x;
        hilInput.acc[1]  = HIL_TO_G * piMsgHilInRx->acc_y;
        hilInput.acc[2]  = HIL_TO_G * piMsgHilInRx->acc_z;
        hilInput.rpm[0]  = HIL_TO_RPM * piMsgHilInRx->rpm_1;
        hilInput.rpm[1]  = HIL_TO_RPM * piMsgHilInRx->rpm_2;
        hilInput.rpm[2]  = HIL_TO_RPM * piMsgHilInRx->rpm_3;
        hilInput.rpm[3]  = HIL_TO_RPM * piMsgHilInRx->rpm_4;
    }
}

void handleHil(void)
{
    if (!hilEnabled) {
        return;
    }

    if (!hilPort) {
        return;
    }

    processHilInput();

    //processHilOutput(); // done in core
}

#endif
