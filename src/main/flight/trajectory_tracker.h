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

// heading modes
typedef enum {
    TT_LOOK_AT_NOTHING,
    TT_LOOK_AT_GATES,
    TT_LOOK_AT_REF,
} tt_heading_mode_t;

extern tt_heading_mode_t tt_heading_mode;

void initRecoveryMode(void);
bool isActiveTrajectoryTracker(void);
bool isActiveTrajectoryTrackerRecovery(void);
void initTrajectoryTracker(void);
void setSpeedTrajectoryTracker(float speed);
void incrementSpeedTrajectoryTracker(float inc);
void stopTrajectoryTracker(void);

// main task
void updateTrajectoryTracker(timeUs_t current);

#endif
