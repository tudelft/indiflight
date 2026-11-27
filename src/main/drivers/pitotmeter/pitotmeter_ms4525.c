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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"
#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/time.h"
#include "drivers/pitotmeter/pitotmeter.h"

// MS4525, Standard address 0x28
#define MS4525_ADDR                 0x28

#if defined(USE_PITOT) && defined(USE_PITOT_MS4525)

typedef struct __attribute__ ((__packed__)) ms4525Ctx_s {
    bool     dataValid;
    uint32_t ms4525_ut;
    uint32_t ms4525_up;
} ms4525Ctx_t;

static ms4525Ctx_t ms4525Ctx;

static bool ms4525_start(pitotDev_t * pitot)
{
    UNUSED(pitot);
    return true;
}

static bool ms4525_read(pitotDev_t * pitot)
{
#ifndef USE_I2C
    UNUSED(pitot);
    return false;
#else
    uint8_t rxbuf1[4];
    uint8_t rxbuf2[4];

    ms4525Ctx_t *ctx = &ms4525Ctx;
    ctx->dataValid = false;

    if (!busReadRegisterBuffer(&pitot->dev, 0xFF, rxbuf1, sizeof(rxbuf1))) {
        return false;
    }

    if (!busReadRegisterBuffer(&pitot->dev, 0xFF, rxbuf2, sizeof(rxbuf2))) {
        return false;
    }

    const uint8_t status = ((rxbuf1[0] & 0xC0) >> 6);
    if (status == 2 || status == 3) {
        return false;
    }

    int16_t dP_raw1, dT_raw1;
    int16_t dP_raw2, dT_raw2;

    dP_raw1 = 0x3FFF & ((rxbuf1[0] << 8) + rxbuf1[1]);
    dT_raw1 = (0xFFE0 & ((rxbuf1[2] << 8) + rxbuf1[3])) >> 5;
    dP_raw2 = 0x3FFF & ((rxbuf2[0] << 8) + rxbuf2[1]);
    dT_raw2 = (0xFFE0 & ((rxbuf2[2] << 8) + rxbuf2[3])) >> 5;

    // reject any values that are the absolute minimum or maximums these
    // can happen due to gnd lifts or communication errors on the bus
    if (dP_raw1 == 0x3FFF || dP_raw1 == 0 || dT_raw1 == 0x7FF || dT_raw1 == 0 ||
        dP_raw2 == 0x3FFF || dP_raw2 == 0 || dT_raw2 == 0x7FF || dT_raw2 == 0) {
        return false;
    }

    // reject any double reads where the value has shifted in the upper more than 0xFF
    if (ABS(dP_raw1 - dP_raw2) > 0xFF || ABS(dT_raw1 - dT_raw2) > 0xFF) {
        return false;
    }

    // Data valid, update ut/up values
    ctx->dataValid = true;
    ctx->ms4525_up = (dP_raw1 + dP_raw2) / 2;
    ctx->ms4525_ut = (dT_raw1 + dT_raw2) / 2;

    return true;
#endif
}

static void ms4525_calculate(pitotDev_t * pitot, float *pressure, float *temperature)
{
    UNUSED(pitot);

    ms4525Ctx_t *ctx = &ms4525Ctx;

    const float P_max = 1.0f;
    const float P_min = -P_max;
    const float PSI_to_Pa = 6894.757f;

    //float dP = ctx->ms4525_up * 10.0f * 0.1052120688f;
    const float dP_psi = -((ctx->ms4525_up - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
    float dP = dP_psi * PSI_to_Pa;
    float T  = C_TO_KELVIN((float)(200.0f * (int32_t)ctx->ms4525_ut) / 2047.0f - 50.0f);

    if (pressure) {
        *pressure = dP;    // Pa
    }

    if (temperature) {
        *temperature = T; // K
    }
}

bool ms4525Detect(pitotDev_t * pitot)
{
#ifndef USE_I2C
    UNUSED(pitot);
    pitot->start = ms4525_start;
    pitot->get = ms4525_read;
    pitot->calculate = ms4525_calculate;
    return false;
#else
    uint8_t rxbuf[4];
    bool defaultAddressApplied = false;
    extDevice_t *dev = &pitot->dev;

    // hardcode I2C with silkscreen label 1
    i2cBusSetInstance(dev, 1);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = MS4525_ADDR;
        defaultAddressApplied = true;
    }

    // Read twice to fix:
    // Sending a start-stop condition without any transitions on the SCL line (no clock pulses in between) creates a
    // communication error for the next communication, even if the next start condition is correct and the clock pulse is applied.
    // An additional start condition must be sent, which results in restoration of proper communication.
    if (!busReadRegisterBuffer(dev, 0xFF, rxbuf, sizeof(rxbuf))) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (!busReadRegisterBuffer(dev, 0xFF, rxbuf, sizeof(rxbuf))) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    ms4525Ctx.dataValid = false;
    ms4525Ctx.ms4525_ut = 0;
    ms4525Ctx.ms4525_up = 0;

    pitot->delay = 10000;
    pitot->calibThreshold = 0.00005f;   // noisy sensor
    pitot->start = ms4525_start;
    pitot->get = ms4525_read;
    pitot->calculate = ms4525_calculate;
    return true;
#endif
}

#endif