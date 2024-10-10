/*
 * Configure serial port to parse pi-messages and provide facilities to send
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

/*
(C) tblaha 2023
 */

#pragma once

#include <string.h>
#include "common/maths.h"

void initPiTelemetry(void);
void handlePiTelemetry(void);
void checkPiTelemetryState(void);

void freePiTelemetryPort(void);
void configurePiTelemetryPort(void);

void piSendEkfInputs(timeUs_t currentTimeUs, fp_vector_t* g, fp_vector_t* a, float* omega);
