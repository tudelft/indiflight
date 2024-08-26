/*
 *
 *
 * Copyright 2023, 2024 Robin Ferede (Delft University of Technology)
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

#ifndef TRAJ_TRACKER
#define TRAJ_TRACKER

#include "common/time.h"		   // for timeUs_t

// UGLY HACK: initialization, setting speed and stopping happens in external_pos.c by using the velocity setpoints as communication
void initRecoveryMode(void);
bool isActiveTrajectoryTracker(void);
bool isActiveTrajectoryTrackerRecovery(void);
void initTrajectoryTracker(void);
void setSpeedTrajectoryTracker(float speed);
void incrementSpeedTrajectoryTracker(float inc);
void stopTrajectoryTracker(void);
void toggleHeadingTracking(void);

// main task
void updateTrajectoryTracker(timeUs_t current);

#endif
