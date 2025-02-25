/*
 * Interface to run a subset of Indiflight without real-time scheduler
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

#ifndef MOCKUP_MAIN_H
#define MOCKUP_MAIN_H

void setImu(const float *g, const float *a);
void setMotorSpeed(const float *omega, const int n);
void setMocap(const float *pos, const float *vel, const float *q);
void setMocapT(const float *pos, const float *vel, const float *q, const uint32_t time_us);
void setPosSetpoint(const float *pos, const float yaw);
void getMotorOutputCommands(float *cmd, int n);
void tick(void);

#endif // MOCKUP_MAIN_H