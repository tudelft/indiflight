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
void indiflightProcessRxModes(void);             // fc/core.c processRxModes()
bool indiIsActive(void);                         // INDI (true) or legacy PID (false)
void indiMixTable(timeUs_t currentTimeUs);       // replaces mixTable() when INDI is active
