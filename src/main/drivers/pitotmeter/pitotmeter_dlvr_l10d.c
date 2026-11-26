/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#if defined(USE_PITOT_DLVR) // indiflight: only built when enabled
#include <build/debug.h>

#include "common/utils.h"
#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "common/maths_indi.h"
#include "drivers/time.h"
#include "drivers/pitotmeter/pitotmeter.h"

#define DLVR_L10D_ADDR                 0x28

#define INCH_OF_H2O_TO_PASCAL   248.84f

#define INCH_H2O_TO_PASCAL(press) (INCH_OF_H2O_TO_PASCAL * (press))

#define RANGE_INCH_H2O      10
#define DLVR_OFFSET         8192.0f
#define DLVR_SCALE          16384.0f
// NOTE :: DLVR_OFFSET_CORR can be used for offset correction. Now firmware relies on zero calibration
#define DLVR_OFFSET_CORR    0.0f   //-9.0f


typedef struct __attribute__ ((__packed__)) dlvrCtx_s {
    bool     dataValid;
    uint32_t dlvr_ut;
    uint32_t dlvr_up;
} dlvrCtx_t;

// indiflight: ported from iNav's bus API to Betaflight's extDevice_t (as ms4525)
static dlvrCtx_t dlvrCtx;

static bool dlvr_start(pitotDev_t * pitot)
{
    UNUSED(pitot);
    return true;
}

static bool dlvr_read(pitotDev_t * pitot)
{
    uint8_t rxbuf1[4];

    dlvrCtx_t * ctx = &dlvrCtx;
    ctx->dataValid = false;

    if (!busReadRegisterBuffer(&pitot->dev, 0xFF, rxbuf1, 4)) {
        return false;
    }

    // status = 00 -> ok, new data
	// status = 01 -> reserved
    // status = 10 -> ok, data stale
    // status = 11 -> error
    const uint8_t status = ((rxbuf1[0] & 0xC0) >> 6);

    if (status) {
        // anything other then 00 in the status bits is an error
        return false;
    }

    int16_t dP_raw1, dT_raw1;

    dP_raw1 = 0x3FFF & ((rxbuf1[0] << 8) + rxbuf1[1]);
    dT_raw1 = (0xFFE0 & ((rxbuf1[2] << 8) + rxbuf1[3])) >> 5;

    // Data valid, update ut/up values
    ctx->dataValid = true;
    ctx->dlvr_up = dP_raw1;
    ctx->dlvr_ut = dT_raw1;

    return true;
}

static void dlvr_calculate(pitotDev_t * pitot, float *pressure, float *temperature)
{
    UNUSED(pitot);
    dlvrCtx_t * ctx = &dlvrCtx;

    // pressure in inchH2O
    float dP_inchH2O = 1.25f *  2.0f * RANGE_INCH_H2O  * (((float)ctx->dlvr_up - (DLVR_OFFSET + DLVR_OFFSET_CORR) ) / DLVR_SCALE); 

    // temperature in deg C
    float T_C = (float)ctx->dlvr_ut * (200.0f / 2047.0f) - 50.0f;     

    // result must fit inside the max pressure range
    if ((dP_inchH2O > RANGE_INCH_H2O) || (dP_inchH2O < -RANGE_INCH_H2O)) {
        return;
    }

    if (pressure) {
        *pressure = INCH_H2O_TO_PASCAL( dP_inchH2O);   // Pa
    }

    if (temperature) {
        *temperature = C_TO_KELVIN( T_C); // K
    }
}

bool dlvrDetect(pitotDev_t * pitot)
{
#ifndef USE_I2C
    UNUSED(pitot);
    return false;
#else
    uint8_t rxbuf[4];
    bool ack = false;
    bool defaultAddressApplied = false;
    extDevice_t *dev = &pitot->dev;

    // hardcode I2C with silkscreen label 1 (as ms4525)
    i2cBusSetInstance(dev, 1);
    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = DLVR_L10D_ADDR;
        defaultAddressApplied = true;
    }

    // Read twice to fix:
    // Sending a start-stop condition without any transitions on the SCL line (no clock pulses in between) creates a
    // communication error for the next communication, even if the next start condition is correct and the clock pulse is applied.
    // An additional start condition must be sent, which results in restoration of proper communication.
    ack = busReadRegisterBuffer(dev, 0xFF, rxbuf, 4);
    ack = busReadRegisterBuffer(dev, 0xFF, rxbuf, 4);
    if (!ack) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    dlvrCtx.dataValid = false;
    dlvrCtx.dlvr_ut = 0;
    dlvrCtx.dlvr_up = 0;

    // Initialize pitotDev object
    pitot->delay = 10000;
    pitot->calibThreshold = 0.00001f;   // low noise sensor
    pitot->start = dlvr_start;
    pitot->get = dlvr_read;
    pitot->calculate = dlvr_calculate;
    return true;
#endif
}

#endif // USE_PITOT_DLVR
