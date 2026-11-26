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

#pragma once

#include "common/filter.h"

#include "pg/pg.h"
#include "drivers/pitotmeter/pitotmeter.h"

// order of the pitot_hardware lookup table (cli/settings.c)
typedef enum {
    PITOT_NONE = 0,
    PITOT_AUTODETECT,
    PITOT_MS4525,
    PITOT_ADC,
    PITOT_FAKE,
    PITOT_DLVR,
    PITOT_MS5525,
} pitotSensor_e;

typedef struct pitotmeterConfig_s {
    uint8_t pitot_hardware;          // Pitotmeter hardware to use
    uint8_t pitot_lpf_hz;            // additional LPF to reduce pitot noise in Hz
    float pitot_scale;               // scale value
} pitotmeterConfig_t;

PG_DECLARE(pitotmeterConfig_t, pitotmeterConfig);

#define TASK_PITOT_RATE_HZ 100

typedef struct pito_s {
    pitotDev_t dev;
    float airSpeed;

    float zeroCalibrationAcc;
    pt1Filter_t lpfState;
    timeUs_t lastMeasurementUs;
    timeMs_t lastSeenHealthyMs;

    float pressureZero;
    float pressure;
    float temperature;
} pitot_t;

extern pitot_t pitot;

bool pitotInit(void);
bool pitotIsCalibrationComplete(void);
void pitotStartCalibration(void);
void pitotUpdate(timeUs_t currentTimeUs);
bool pitotIsHealthy(void);
