/*
 * Receive IMU, motor RPM and send motor commands using pi-protocol
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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

typedef struct hilInput_s {
    float gyro[3]; // deg/s
    float acc[3]; // g
    float rpm[4]; // rpm
} hilInput_t;

extern hilInput_t hilInput;

//typedef struct hilOutput_s {
//    float actuator_set[4];
//} hilOutput_t;

//extern hilOutput_t hilOutput;

void initHil(void);
void handleHil(void);
void checkHilState(void);

void freeHilPort(void);
void configureHilPort(void);

void hilSendActuators(void);
