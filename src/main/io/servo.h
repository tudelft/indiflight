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

#include "io/servo_protocol.h"



typedef enum {
    SERVO_IDLE,
    SERVO_STX_FOUND,
    SERVO_WAITING_FOR_CHECKSUM,
} Servo_parse_state_t;


extern struct ActuatorsServoOut Servo_out;

void initActuatorsServo(void);
void handleActuatorsServo(void);
void sendActuatorsServo(void);

void freeActuatorsServoPort(void);
void configureActuatorsServoPort(void);


