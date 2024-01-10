#ifndef TRAJ_TRACKER
#define TRAJ_TRACKER

#include "common/time.h"		   // for timeUs_t

// state of trajectory tracker:
typedef enum {
    INACTIVE,   // drone is not in trajectory tracker mode
    INIT,       // drone is going to starting point of trajectory
    ACTIVE,     // drone is tracking trajectory
    EXIT        // stop tracking. TODO: safe way to exit trajectory tracker mode
} tt_state_t;

void setSpeedFactorTrajectoryTracker(float speed_factor);

void updateTrajectoryTracker(timeUs_t current);

#endif