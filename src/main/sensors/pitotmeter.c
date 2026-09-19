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

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/pitotmeter/pitotmeter.h"
#include "drivers/pitotmeter/pitotmeter_ms4525.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "pitotmeter.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "io/gps.h"

#include "sensors/pitotmeter.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "io/gps.h"


extern baro_t baro;

pitot_t pitot = {.lastMeasurementUs = 0, .lastSeenHealthyMs = 0};

// Pitot sensor validation state
static bool pitotHardwareFailed = false;
// SKIP static uint16_t pitotFailureCounter = 0;
// SKIP static uint16_t pitotRecoveryCounter = 0;
static bool pitotAirspeedValidCached = false;
#define PITOT_FAILURE_THRESHOLD 10   // 0.2 seconds at 50Hz - fast detection per LOG00002 analysis
#define PITOT_RECOVERY_THRESHOLD 100 // 2 seconds of consecutive good readings to recover

// Forward declaration for GPS-based airspeed fallback
//SKIP static float getVirtualAirspeedEstimate(void);

PG_REGISTER_WITH_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig, PG_PITOTMETER_CONFIG, 2);

#define PITOT_HARDWARE_TIMEOUT_MS   500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define PITOT_HARDWARE_DEFAULT    PITOT_AUTODETECT

PG_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig,
    .pitot_hardware = PITOT_HARDWARE_DEFAULT,
    .pitot_lpf_hz = 3,
    .pitot_scale = 1
);

bool pitotDetect(pitotDev_t *dev, uint8_t pitotHardwareToUse)
{
    // SKIP pitotSensor_e pitotHardware = PITOT_NONE;
    // SKIP requestedSensors[SENSOR_INDEX_PITOT] = pitotHardwareToUse;

    switch (pitotHardwareToUse) {
        case PITOT_AUTODETECT:
        case PITOT_MS4525:
#ifdef USE_PITOT_MS4525
            if (ms4525Detect(dev)) {
                // SKIP pitotHardware = PITOT_MS4525;
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
                // SKIP pitotHardware = PITOT_MS5525;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_DLVR:

			// Skip autodetection for DLVR (it is indistinguishable from MS4525) and allow only manual config
#ifdef USE_PITOT_DLVR
            if (pitotHardwareToUse != PITOT_AUTODETECT && dlvrDetect(dev)) {
                // SKIP pitotHardware = PITOT_DLVR;
                break;
            }
#endif
            FALLTHROUGH;

        case PITOT_ADC:
#if defined(USE_ADC) && defined(USE_PITOT_ADC)
            if (adcPitotDetect(dev)) {
                // SKIP pitotHardware = PITOT_ADC;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_VIRTUAL:
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
#if defined(USE_WIND_ESTIMATOR) && defined(USE_PITOT_VIRTUAL)
                if (virtualPitotDetect(dev)) {
                    // SKIP pitotHardware = PITOT_VIRTUAL;
                    break;
                }
#endif
                // set requested to None to prevent hardware failure if GPS not enabled
                // SKIP requestedSensors[SENSOR_INDEX_PITOT] = PITOT_NONE;
                break;
            }
            FALLTHROUGH;

        case PITOT_MSP:
#ifdef USE_PITOT_MSP
            // Skip autodetection for MSP baro, only allow manual config
            if (pitotHardwareToUse != PITOT_AUTODETECT && mspPitotmeterDetect(dev)) {
                // SKIP pitotHardware = PITOT_MSP;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_FAKE:
#ifdef USE_PITOT_FAKE
            if (fakePitotDetect(dev)) {
                // SKIP pitotHardware = PITOT_FAKE;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }
            FALLTHROUGH;

        case PITOT_NONE:
            // SKIP pitotHardware = PITOT_NONE;
            break;
    }

    // SKIP if (pitotHardware == PITOT_NONE) {
    // SKIP     sensorsClear(SENSOR_PITOT);
    // SKIP     return false;
    // SKIP }

    // SKIP detectedSensors[SENSOR_INDEX_PITOT] = pitotHardware;
    // SKIP sensorsSet(SENSOR_PITOT);
    return true;
}

bool pitotInit(void)
{
    if (!pitotDetect(&pitot.dev, pitotmeterConfig()->pitot_hardware)) {
        return false;
    }
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
    // SKIP return zeroCalibrationIsCompleteS(&pitot.zeroCalibrationAcc) && zeroCalibrationIsSuccessfulS(&pitot.zeroCalibrationAcc);
    return calibrationIsComplete;
}

void pitotStartCalibration(void)
{
    // SKIP zeroCalibrationStartS(&pitot.zeroCalibrationAcc, CALIBRATING_PITOT_TIME_MS, SSL_AIR_PRESSURE * pitot.dev.calibThreshold, false);
    calibrationIsComplete = false;
    calibrationIsRunning = true;
    calibrationCycleStartedAt = micros();
    calibrationCycleCount = 0;
    pitot.zeroCalibrationAcc = 0.0f;
    return;
}

static void performPitotCalibrationCycle(timeUs_t currentTimeUs)
{
    /*SKIP
    zeroCalibrationAddValueS(&pitot.zeroCalibrationAcc, pitot.pressure);

    if (zeroCalibrationIsCompleteS(&pitot.zeroCalibrationAcc)) {
        zeroCalibrationGetZeroS(&pitot.zeroCalibrationAcc, &pitot.pressureZero);
        LOG_DEBUG(PITOT, "Pitot calibration complete (%d)", (int)lrintf(pitot.pressureZero));
    }
    */

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
            // LOG_DEBUG(PITOT, "Pitot calibration complete (%d)", (int)lrintf(pitot.pressureZero));
        }
    }

   return;
}

void pitotUpdate(timeUs_t currentTimeUs)
{
    static float pitotPressureTmp;
    static float pitotTemperatureTmp;

    pitot.lastMeasurementUs = currentTimeUs;

// SKIP     while(1) {
// SKIP #ifdef USE_SIMULATOR
// SKIP     	while (SIMULATOR_HAS_OPTION(HITL_AIRSPEED) && SIMULATOR_HAS_OPTION(HITL_PITOT_FAILURE))
// SKIP         {
// SKIP             ptDelayUs(10000);
// SKIP     	}
// SKIP #endif
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

#ifdef USE_SIMULATOR
        if (SIMULATOR_HAS_OPTION(HITL_AIRSPEED)) {
            pitotPressureTmp = sq(simulatorData.airSpeed) * SSL_AIR_DENSITY / 20000.0f + SSL_AIR_PRESSURE;
        }
#endif
#if defined(USE_PITOT_FAKE)
        if (pitotmeterConfig()->pitot_hardware == PITOT_FAKE) {
            pitotPressureTmp = sq(fakePitotGetAirspeed()) * SSL_AIR_DENSITY / 20000.0f + SSL_AIR_PRESSURE;
        }
#endif
        pitotAirspeedValidCached = pitotValidateAirspeed();

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

            pitot.airSpeed = pitotmeterConfig()->pitot_scale * sqrtf(2.0f * fabsf(pitot.pressure - pitot.pressureZero) / SSL_AIR_DENSITY) * 100;  // cm/s
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
// SKIP #ifdef USE_SIMULATOR
// SKIP         if (SIMULATOR_HAS_OPTION(HITL_AIRSPEED)) {
// SKIP             pitot.airSpeed = simulatorData.airSpeed;
// SKIP         }
// SKIP #endif
// SKIP     }
}

/*SKIP
void pitotUpdate(void)
{
    pitotThread();
}
*/

/*
 * Airspeed estimate in cm/s
 * Returns hardware pitot if valid, GPS-based virtual airspeed if pitot failed,
 * or raw pitot value as last resort
 */
float getAirspeedEstimate(void)
{
    // If hardware pitot has failed validation, use GPS-based virtual airspeed
    if (pitotHardwareFailed) {
        // SKIP float virtualAirspeed = getVirtualAirspeedEstimate();
        // SKIP if (virtualAirspeed > 0.0f) {
        // SKIP     return virtualAirspeed;
        // SKIP }
        return 0.0f;
    }
    return pitot.airSpeed;
}

bool pitotIsHealthy(void)
{
    return (millis() - pitot.lastSeenHealthyMs) < PITOT_HARDWARE_TIMEOUT_MS;
}

/**
 * Calculate virtual airspeed estimate (same as virtual pitot)
 *
 * Uses GPS velocity with wind correction when available, providing a reference
 * airspeed that already accounts for wind conditions.
 *
 * @return virtual airspeed in cm/s, or 0 if GPS unavailable
 */
/*
static float getVirtualAirspeedEstimate(void)
{
#if defined(USE_GPS) && defined(USE_WIND_ESTIMATOR)
    if (!STATE(GPS_FIX)) {
        return 0.0f;
    }

    float airSpeed = 0.0f;

    // Use wind estimator if available (matches virtual pitot logic)
    if (isEstimatedWindSpeedValid()) {
        uint16_t windHeading;  // centidegrees
        float windSpeed = getEstimatedHorizontalWindSpeed(&windHeading);  // cm/s
        float horizontalWindSpeed = windSpeed * cos_approx(CENTIDEGREES_TO_RADIANS(windHeading - posControl.actualState.yaw));
        airSpeed = posControl.actualState.velXY - horizontalWindSpeed;
        airSpeed = calc_length_pythagorean_2D(airSpeed, getEstimatedActualVelocity(Z) + getEstimatedWindSpeed(Z));
    } else {
        // Fall back to raw GPS velocity if no wind estimator
        airSpeed = calc_length_pythagorean_3D(gpsSol.velNED[X], gpsSol.velNED[Y], gpsSol.velNED[Z]);
    }

    return airSpeed;
#elif defined(USE_GPS)
    // No wind estimator, use raw GPS velocity
    if (!STATE(GPS_FIX)) {
        return 0.0f;
    }
    return calc_length_pythagorean_3D(gpsSol.velNED[X], gpsSol.velNED[Y], gpsSol.velNED[Z]);
#else
    return 0.0f;
#endif
}
*/

/**
 * Pitot sensor sanity check against virtual airspeed
 *
 * Compares hardware pitot reading against virtual airspeed (GPS + wind estimator)
 * to detect gross sensor failures like blocked pitot tubes.
 *
 * Uses wide thresholds to catch implausible readings while avoiding false positives:
 * - Compares against wind-corrected virtual airspeed (not raw GPS groundspeed)
 * - Wide tolerance (30%-200%) catches gross failures only
 * - Detects: blocked pitot reading 25 km/h when virtual shows 85 km/h
 * - Avoids: false positives from sensor accuracy differences
 *
 * @return true if pitot reading appears plausible, false if likely failed
 */
/*
static bool isPitotReadingPlausible(void)
{
#ifdef USE_GPS
    if (!STATE(GPS_FIX)) {
        return true;
    }

    const float virtualAirspeedCmS = getVirtualAirspeedEstimate();
    const float minValidationSpeed = 700.0f;  // 7 m/s

    if (virtualAirspeedCmS < minValidationSpeed) {
        return true;
    }

    const float pitotAirspeedCmS = pitot.airSpeed;

    // Wide thresholds to catch gross failures (blocked pitot) only
    const float minPlausibleAirspeed = virtualAirspeedCmS * 0.3f;  // 30% of virtual
    const float maxPlausibleAirspeed = virtualAirspeedCmS * 2.0f;  // 200% of virtual

    if (pitotAirspeedCmS < minPlausibleAirspeed || pitotAirspeedCmS > maxPlausibleAirspeed) {
        return false;
    }

    return true;
#else
    return true;
#endif
}
*/

/**
 * Check if pitot sensor has failed validation
 *
 * @return true if pitot has failed sanity checks and should not be trusted
 */
bool pitotHasFailed(void)
{
    return pitotHardwareFailed;
}

bool pitotValidateAirspeed(void)
{
    /* SKIP
    bool ret = false;
    ret = pitotIsHealthy() && pitotIsCalibrationComplete();

    // For virtual pitot, we need GPS fix
    if (detectedSensors[SENSOR_INDEX_PITOT] == PITOT_VIRTUAL) {
        ret = ret && STATE(GPS_FIX);
    }

    // For hardware pitot sensors, validate readings against GPS when armed
    // This detects blocked or failed pitot tubes
    if (ret && detectedSensors[SENSOR_INDEX_PITOT] != PITOT_VIRTUAL &&
        detectedSensors[SENSOR_INDEX_PITOT] != PITOT_NONE) {

        if (ARMING_FLAG(ARMED)) {
            // Check if pitot reading is plausible
            if (!isPitotReadingPlausible()) {
                pitotFailureCounter++;
            } else if (pitotFailureCounter > 0) {
                // Decay counter if sensor appears healthy
                pitotFailureCounter--;
            }

            // Declare failure after sustained implausible readings
            if (pitotFailureCounter >= PITOT_FAILURE_THRESHOLD) {
                pitotHardwareFailed = true;
                pitotRecoveryCounter = 0;  // Start recovery tracking
            }

            // Recovery: require sustained consecutive good readings to clear failure
            if (pitotHardwareFailed) {
                if (isPitotReadingPlausible()) {
                    pitotRecoveryCounter++;
                    if (pitotRecoveryCounter >= PITOT_RECOVERY_THRESHOLD) {
                        pitotHardwareFailed = false;  // Sensor has recovered
                        pitotFailureCounter = 0;
                        pitotRecoveryCounter = 0;
                    }
                } else {
                    // Bad reading resets recovery progress
                    pitotRecoveryCounter = 0;
                }
            }
        } else {
            // Reset on disarm for next flight
            pitotHardwareFailed = false;
            pitotFailureCounter = 0;
            pitotRecoveryCounter = 0;
        }

        // If pitot has failed sanity checks, require GPS fix (like virtual pitot)
        if (pitotHardwareFailed) {
            ret = ret && STATE(GPS_FIX);
        }
    }

    return ret;
    */
   return true;
}

bool pitotGetValidForAirspeed(void)
{
    return pitotAirspeedValidCached;
}
#endif /* PITOT */
