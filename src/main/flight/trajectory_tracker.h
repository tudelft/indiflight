#ifndef TRAJ_TRACKER
#define TRAJ_TRACKER

#include "common/time.h"		   // for timeUs_t

// UGLY HACK: initialization, setting speed and stopping happens in external_pos.c by using the velocity setpoints as communication
void initRecoveryMode(void);
bool isActiveTrajectoryTracker(void);
void initTrajectoryTracker(void);
void setSpeedTrajectoryTracker(float speed);
void stopTrajectoryTracker(void);

// main task
void updateTrajectoryTracker(timeUs_t current);

#endif