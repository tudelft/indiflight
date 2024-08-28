/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "pg/rx.h"

#include "rx/rx.h"

struct sbuf_s;

bool srxl2RxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);
bool srxl2RxIsActive(void);
void srxl2RxWriteData(const void *data, int len);
bool srxl2TelemetryRequested(void);
void srxl2InitializeFrame(struct sbuf_s *dst);
void srxl2FinalizeFrame(struct sbuf_s *dst);
void srxl2Bind(void);
