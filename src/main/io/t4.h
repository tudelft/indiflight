/*
 * Configure serial port to interface with teensy actuator board https://github.com/tudelft/t4_actuators_board/
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *c
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

#pragma once

#include "io/t4_protocol.h"

typedef enum {
    T4_IDLE,
    T4_STX_FOUND,
    T4_WAITING_FOR_CHECKSUM,
} t4_parse_state_t;


extern struct ActuatorsT4Out t4_out;

void initActuatorsT4(void);
void handleActuatorsT4(void);
void sendActuatorsT4(void);

void freeActuatorsT4Port(void);
void configureActuatorsT4Port(void);


