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
#include <math.h>

#include "platform.h"

#ifdef USE_PITOT

#include "common/maths_indi.h"
#include "common/time.h"
#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/pitotmeter/pitotmeter.h"
#include "drivers/pitotmeter/pitotmeter_ms4525.h"
#include "drivers/pitotmeter/pitotmeter_ms5525.h"
#include "drivers/pitotmeter/pitotmeter_dlvr_l10d.h"
#include "drivers/pitotmeter/pitotmeter_adc.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/time.h"

#include "sensors/pitotmeter.h"

// indiflight: the pitot sensor layer from iNav, without iNav's virtual/MSP pitots,
// HITL simulator and GPS plausibility check (they need iNav subsystems)

pitot_t pitot = {.lastMeasurementUs = 0, .lastSeenHealthyMs = 0};

PG_REGISTER_WITH_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig, PG_PITOTMETER_CONFIG, 2);

#define PITOT_HARDWARE_TIMEOUT_MS   500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define PITOT_HARDWARE_DEFAULT    PITOT_AUTODETECT

PG_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig,
    .pitot_hardware = PITOT_HARDWARE_DEFAULT,
    .pitot_lpf_hz = 3,
    .pitot_scale = 1
);

// Fills dev with the driver of the first sensor found (AUTO: MS4525, MS5525, ADC, FAKE
// in this order; DLVR only when selected, it is indistinguishable from the MS4525).
// Without a sensor, dev keeps no driver and pitotUpdate() does nothing.
static void pitotDetect(pitotDev_t *dev, uint8_t pitotHardwareToUse)
{
    switch (pitotHardwareToUse) {
        case PITOT_AUTODETECT:
        case PITOT_MS4525:
#ifdef USE_PITOT_MS4525
            if (ms4525Detect(dev)) {
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_MS5525:
#ifdef USE_PITOT_MS5525
            if (ms5525Detect(dev)) {
                break;
            }
#endif
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_DLVR:
#ifdef USE_PITOT_DLVR
            if (pitotHardwareToUse != PITOT_AUTODETECT && dlvrDetect(dev)) {
                break;
            }
#endif
            FALLTHROUGH;

        case PITOT_ADC:
#if defined(USE_ADC) && defined(USE_PITOT_ADC)
            if (adcPitotDetect(dev)) {
                break;
            }
#endif
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_FAKE:
#ifdef USE_PITOT_FAKE
            if (fakePitotDetect(dev)) {
                break;
            }
#endif
            FALLTHROUGH;

        case PITOT_NONE:
        default:
            break;
    }
}

bool pitotInit(void)
{
    pitotDetect(&pitot.dev, pitotmeterConfig()->pitot_hardware);
    // Init filter
    if (pitotmeterConfig()->pitot_lpf_hz > 0) {
        float gain = pt1FilterGain(pitotmeterConfig()->pitot_lpf_hz, 1.f / TASK_PITOT_RATE_HZ);
        pt1FilterInit(&pitot.lpfState, gain);
    }
    return true;
}

static bool calibrationIsComplete = false;
static timeUs_t calibrationCycleStartedAt = 0;
static bool calibrationIsRunning = false;
static unsigned int calibrationCycleCount = 0;
#define CALIBRATING_PITOT_TIME_MS 4000
bool pitotIsCalibrationComplete(void)
{
    return calibrationIsComplete;
}

void pitotStartCalibration(void)
{
    calibrationIsComplete = false;
    calibrationIsRunning = true;
    calibrationCycleStartedAt = micros();
    calibrationCycleCount = 0;
    pitot.zeroCalibrationAcc = 0.0f;
}

static void performPitotCalibrationCycle(timeUs_t currentTimeUs)
{
    /*
     * 1. check elapsed time is still within CALIBRATING_PITOT_TIME_MS
     * 2. if yes, accumulate pressure readings into pitot.zeroCalibrationAcc, and increment calibrationCycleCount
     * 3. if no, calculate average pressure and set calibrationIsComplete to true
    */
    if (calibrationIsRunning) {
        if ((currentTimeUs - calibrationCycleStartedAt) < 1e3*(CALIBRATING_PITOT_TIME_MS)) {
            // ignore first 10% of readings
            if ((currentTimeUs - calibrationCycleStartedAt) < 1e3*(CALIBRATING_PITOT_TIME_MS/10)) {
                return;
            }
            // accumulate pressure readings
            pitot.zeroCalibrationAcc += pitot.pressure;
            calibrationCycleCount++;
        } else {
            // calculate average pressure
            if (calibrationCycleCount > 0) {
                pitot.pressureZero = pitot.zeroCalibrationAcc / calibrationCycleCount;
            }
            calibrationIsComplete = true;
            calibrationIsRunning = false;
        }
    }
}

void pitotUpdate(timeUs_t currentTimeUs)
{
    static float pitotPressureTmp;
    static float pitotTemperatureTmp;

    pitot.lastMeasurementUs = currentTimeUs;

    if (!pitot.dev.start || !pitot.dev.get || !pitot.dev.calculate) {
        return;
    }

    if (pitot.lastSeenHealthyMs == 0) {
        if (pitot.dev.start(&pitot.dev)) {
            pitot.lastSeenHealthyMs = millis();
        }
    }

    if ((millis() - pitot.lastSeenHealthyMs) >= (pitot.dev.delay / 1000)) {
        if (pitot.dev.get(&pitot.dev)) {    // read current data
            pitot.lastSeenHealthyMs = millis();
        }

        if (pitot.dev.start(&pitot.dev)) {  // init for next read
            pitot.lastSeenHealthyMs = millis();
        }
    }

    pitot.dev.calculate(&pitot.dev, &pitotPressureTmp, &pitotTemperatureTmp);

#if defined(USE_PITOT_FAKE)
    if (pitotmeterConfig()->pitot_hardware == PITOT_FAKE) {
        pitotPressureTmp = sq(fakePitotGetAirspeed()) * SSL_AIR_DENSITY / 20000.0f + SSL_AIR_PRESSURE;
    }
#endif

    // Calculate IAS
    if (pitotIsCalibrationComplete()) {
        // NOTE ::
        // https://en.wikipedia.org/wiki/Indicated_airspeed
        // Indicated airspeed (IAS) is the airspeed read directly from the airspeed indicator on an aircraft, driven by the pitot-static system.
        // The IAS is an important value for the pilot because it is the indicated speeds which are specified in the aircraft flight manual for
        // such important performance values as the stall speed. A typical aircraft will always stall at the same indicated airspeed (for the current configuration)
        // regardless of density, altitude or true airspeed.
        //
        // Therefore we shouldn't care about CAS/TAS and only calculate IAS since it's more indicative to the pilot and more useful in calculations
        // It also allows us to use pitot_scale to calibrate the dynamic pressure sensor scale

        // NOTE ::filter pressure - apply filter when NOT calibrating for zero !!!
        if (pitotmeterConfig()->pitot_lpf_hz > 0) {
            pitot.pressure = pt1FilterApply(&pitot.lpfState, pitotPressureTmp);
        } else {
            pitot.pressure = pitotPressureTmp;
        }
        pitot.lastMeasurementUs = currentTimeUs;

        float dP = pitot.pressureZero - pitot.pressure;
        if (dP <= 0.f) {
            pitot.airSpeed = 0.f;
        } else {
            pitot.airSpeed = pitotmeterConfig()->pitot_scale * sqrtf(2.0f * dP / SSL_AIR_DENSITY) * 100;  // cm/s
        }
        pitot.temperature = pitotTemperatureTmp;   // Kelvin

    } else {
        pitot.pressure = pitotPressureTmp;
        performPitotCalibrationCycle(currentTimeUs);
        pitot.airSpeed = 0.0f;
    }

#if defined(USE_PITOT_FAKE)
    if (pitotmeterConfig()->pitot_hardware == PITOT_FAKE) {
        pitot.airSpeed = fakePitotGetAirspeed();
    }
#endif
}

bool pitotIsHealthy(void)
{
    return (millis() - pitot.lastSeenHealthyMs) < PITOT_HARDWARE_TIMEOUT_MS;
}

#endif /* USE_PITOT */
