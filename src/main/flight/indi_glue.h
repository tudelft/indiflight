/*
 * This file is part of Indiflight.
 *
 * Integration of the INDI controller into the upstream Betaflight main loop.
 * Upstream files call only these functions, each under #ifdef USE_INDI.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#pragma once

#include <stdbool.h>
#include "common/time.h"

void indiflightInit(void);                       // fc/init.c, after pidInit()
void indiflightPreController(timeUs_t currentTimeUs); // fc/core.c, before the controller
#define RC_STICK_MOVEMENT_THRESHOLD 0.1f // stick movement threshold to detect stick activity
void setSticksReference(void);
bool haveSticksMoved(void);
bool indiIsActive(void);                         // INDI_MODE: INDI (true) or upstream PID (false)
void indiMixTable(timeUs_t currentTimeUs);       // replaces mixTable() when INDI is active
void indiTrackMotorOutput(void);                 // after mixTable() when INDI is not active
float indiGetMotorNormalized(int i);
