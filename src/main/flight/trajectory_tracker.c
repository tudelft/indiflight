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

#include "trajectory_tracker.h"

#include <math.h>
#include "flight/imu.h"
#include "io/external_pos.h"
#include "flight/indi.h"
#include "pos_ctl.h"
#include "fc/runtime_config.h"


#ifdef USE_TRAJECTORY_TRACKER

#ifndef USE_POS_CTL
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_POS_CTL"
#endif

#ifndef USE_TELEMETRY_PI
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_TELEMETRY_PI"
#endif

// state of trajectory tracker:
bool tt_active = false;

// acceleration and yaw rate setpoints
float tt_acc_sp[3] = {0};

// reference trajectory
float tt_pos_ref[3] = {0};
float tt_vel_ref[3] = {0};
float tt_acc_ref[3] = {0};
float tt_yaw_ref = 0.;
float tt_yaw_rate_ref = 0.;
bool tt_track_heading = false;

// time stuff
float tt_speed_factor = 0.0f;
float tt_progress = 0.0f;
float tt_time = 0.0f;
timeUs_t last = 0;

// gains
float tt_pos_gain = 2.0; //1.5;
float tt_vel_gain = 3.0; //2.5;
// float tt_yaw_gain = 1.0;

// radius of circular trajectory
float tt_R = 3.0f;

// recovery algorithm
bool tt_recovery_active = false;

struct recovery_problem {
    float R;         // radius of circular trajectory for recovery
    float theta0;    // starting angle along the cicle
    float omega0;    // starting angular velocity
    float tf;        // time when recovery should be finished
} tt_recovery_problem;

void initRecoveryMode(void) {
    // check current position to determine the radius of the recovery trajectory
    float x = posEstNed.V.X;
    float y = posEstNed.V.Y;
    tt_recovery_problem.R = sqrtf(x*x + y*y);

    // final time hardcoded
    tt_recovery_problem.tf = 2.0f;

    // make sure radius is less then 4.0
    if (tt_recovery_problem.R > 4.0) {
        tt_recovery_problem.R = 4.0;
    }

    // get starting angle
    tt_recovery_problem.theta0 = atan2f(y, x);

    // get starting angular velocity
    float vx = velEstNed.V.X;
    float vy = velEstNed.V.Y;
    float v_perp = (-y*vx + x*vy)/tt_recovery_problem.R;
    tt_recovery_problem.omega0 = v_perp/tt_recovery_problem.R;

    // set time to zero
    tt_time = 0.0f;

    // set active to true
    tt_active = true;
    tt_recovery_active = true;
}

void getRefsRecoveryTrajectory(float t) {
    // get stuff from recovery problem into floats (for readability)
    float R = tt_recovery_problem.R;
    float theta0 = tt_recovery_problem.theta0;
    float omega0 = tt_recovery_problem.omega0;
    float tf = tt_recovery_problem.tf;

    // safety check
    if (t > tf) {
        t = tf;
    }

    float omega = omega0*(1.0f - t/tf);                 // angular velocity
    float theta = theta0 + omega0*(t - 0.5f*t*t/tf);    // angle along the circle
    while (theta >  M_PIf) theta -= (2.0f * M_PIf);  // always wrap input angle to -PI..PI
    while (theta < -M_PIf) theta += (2.0f * M_PIf);

    // position refs
    tt_pos_ref[0] = R*cosf(theta);
    tt_pos_ref[1] = R*sinf(theta);
    tt_pos_ref[2] = -1.5f;

    // velocity refs
    tt_vel_ref[0] = -R*sinf(theta)*omega;
    tt_vel_ref[1] = R*cosf(theta)*omega;
    tt_vel_ref[2] = 0.0f;

    // acceleration refs
    tt_acc_ref[0] = -R*cosf(theta)*omega*omega -R*sinf(theta)*(-omega0/tf);
    tt_acc_ref[1] = -R*sinf(theta)*omega*omega +R*cosf(theta)*(-omega0/tf);
    tt_acc_ref[2] = 0.0f;

    // heading (we dont want to track that in recovery though)
    tt_yaw_ref = theta;
    tt_yaw_rate_ref = omega;
    posSpNed.trackPsi = false;
}

bool isActiveTrajectoryTracker(void) {
    return tt_active;
}

bool isActiveTrajectoryTrackerRecovery(void) {
    return tt_recovery_active;
}

void getRefsTrajectoryTracker(float p) {
    // hard coded circular trajectory with radius R
    // x(t)   = R*cos(p(t))
    // y(t)   = R*sin(p(t))
    // z(t)   = -1.5
    // psi(t) = p(t) + pi/2
    // where:
    // dp/dt  = speed_factor (piecewise constant)
    while (tt_progress >  M_PIf) tt_progress -= (2.0f * M_PIf);  // always wrap input angle to -PI..PI
    while (tt_progress < -M_PIf) tt_progress += (2.0f * M_PIf);

    // position refs
    tt_pos_ref[0] = tt_R*cosf(p);
    tt_pos_ref[1] = tt_R*sinf(p);
    tt_pos_ref[2] = -1.5f;

    // velocity refs
    tt_vel_ref[0] = -tt_R*tt_speed_factor*sinf(p);
    tt_vel_ref[1] = tt_R*tt_speed_factor*cosf(p);
    tt_vel_ref[2] = 0.0f;

    // acceleration refs
    tt_acc_ref[0] = -tt_R*tt_speed_factor*tt_speed_factor*cosf(p);
    tt_acc_ref[1] = -tt_R*tt_speed_factor*tt_speed_factor*sinf(p);
    tt_acc_ref[2] = 0.0f;

    // heading
    tt_yaw_ref = p + M_PIf/2.0f;
    tt_yaw_rate_ref = tt_speed_factor;
    posSpNed.trackPsi = tt_track_heading; // choice: track heading or neglect it?
}

void initTrajectoryTracker(void) {
    // reset everything
    tt_progress = 0.0f;
    tt_speed_factor = 0.0f;
    tt_yaw_ref = 0.f;
    //tt_active = true; //dont activate yet, just go the starting point with default controller

    // setpoint = starting point of trajectory
    getRefsTrajectoryTracker(0.0f);
    posSpNed.pos.V.X = tt_pos_ref[0];
    posSpNed.pos.V.Y = tt_pos_ref[1];
    posSpNed.pos.V.Z = tt_pos_ref[2];
    posSpNed.psi = tt_yaw_ref;
    posSpNed.trackPsi = true;
    posSetpointState = EXT_POS_NEW_MESSAGE;
}

void setSpeedTrajectoryTracker(float speed) {
    tt_speed_factor = speed/tt_R;
    tt_active = true;
}

void incrementSpeedTrajectoryTracker(float inc) {
    tt_speed_factor += inc/tt_R;
    tt_active = true;
}

void toggleHeadingTracking(void) {
    tt_track_heading ^= true;
}

void stopTrajectoryTracker(void) {
    // set speed factor to zero, get new setpoints, and deactivate
    tt_speed_factor = 0.0f;
    getRefsTrajectoryTracker(tt_progress);
    tt_active = false;
    tt_recovery_active = false;

    // reset I terms
    resetIterms();
}

void updateTrajectoryTracker(timeUs_t current) {
    // only does something when tt_active is true
    if (tt_active && ARMING_FLAG(ARMED)) {
        // Track trajectory and overwrite pos_ctl

        // update tt_progress
        tt_progress += tt_speed_factor*(current - last)*1e-6f;

        // update references
        getRefsTrajectoryTracker(tt_progress);

        // recovery mode
        if (tt_recovery_active) {
            // update time
            tt_time += (current - last)*1e-6f;

            // get references
            getRefsRecoveryTrajectory(tt_time);

            // check if recovery is finished
            if (tt_time >= tt_recovery_problem.tf) {
                tt_recovery_active = false;
                tt_active = false;
            }
        }

        // update setpoints
        // pos error
        float x_error = tt_pos_ref[0] - posEstNed.V.X;
        float y_error = tt_pos_ref[1] - posEstNed.V.Y;
        float z_error = tt_pos_ref[2] - posEstNed.V.Z;

        // vel setpoint
        float vx_sp = tt_vel_ref[0] + tt_pos_gain*x_error;
        float vy_sp = tt_vel_ref[1] + tt_pos_gain*y_error;
        float vz_sp = tt_vel_ref[2] + tt_pos_gain*z_error;

        // acc setpoint
        tt_acc_sp[0] = tt_acc_ref[0] + tt_vel_gain*(vx_sp - velEstNed.V.X);
        tt_acc_sp[1] = tt_acc_ref[1] + tt_vel_gain*(vy_sp - velEstNed.V.Y);
        tt_acc_sp[2] = tt_acc_ref[2] + tt_vel_gain*(vz_sp - velEstNed.V.Z);

        // overwrite accSpNedFromPos and rateSpBodyFromPos (from pos_ctl.c)
        accSpNedFromPos.V.X = tt_acc_sp[0];
        accSpNedFromPos.V.Y = tt_acc_sp[1];
        accSpNedFromPos.V.Z = tt_acc_sp[2];

        // overwrite posSpNed (from pos_ctl.c)
        posSpNed.pos.V.X = tt_pos_ref[0];
        posSpNed.pos.V.Y = tt_pos_ref[1];
        posSpNed.pos.V.Z = tt_pos_ref[2];

        // overwrite velSetpointNed (from pos_ctl.c)
        posSpNed.vel.V.X = vx_sp;
        posSpNed.vel.V.Y = vy_sp; //random comment
        posSpNed.vel.V.Z = vz_sp;

        // overwrite yawSetpoint (from pos_ctl.c)
        posSpNed.psi = tt_yaw_ref;

        posSetpointState = EXT_POS_NEW_MESSAGE;

        // overwrite rateSpBody 
        if (posSpNed.trackPsi) {
            rateSpBodyFromPos = coordinatedYaw(tt_yaw_rate_ref);
        } else {
            rateSpBodyFromPos.V.X = 0;
            rateSpBodyFromPos.V.Y = 0;
            rateSpBodyFromPos.V.Z = 0;
        }
    }

    // update last
    last = current;
}


#endif
