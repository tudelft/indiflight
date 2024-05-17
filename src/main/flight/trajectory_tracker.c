#include "trajectory_tracker.h"

#include <math.h>
#include "flight/imu.h"
#include "io/external_pos.h"
#include "flight/indi.h"
#include "pos_ctl.h"


#ifdef USE_TRAJECTORY_TRACKER

#ifndef USE_POS_CTL
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_POS_CTL"
#endif

// state of trajectory tracker:
bool tt_active = false;

// acceleration and yaw rate setpoints
float tt_acc_sp[3];
float tt_yaw_rate_sp;

// reference trajectory
float tt_pos_ref[3];
float tt_vel_ref[3];
float tt_acc_ref[3];
float tt_yaw_ref;
float tt_yaw_rate_ref;

// time stuff
float tt_speed_factor = 0.0f;
float tt_progress = 0.0f;
timeUs_t last = 0;

// gains
//float tt_pos_gain = 1.5;
//float tt_vel_gain = 2.5;
//float tt_yaw_gain = 1.0;

// radius of circular trajectory
float tt_R = 1.0f;

void getRefsTrajectoryTracker(float p) {
    // hard coded circular trajectory with radius R
    // x(t)   = R*cos(p(t))
    // y(t)   = R*sin(p(t))
    // z(t)   = -1.5
    // psi(t) = p(t) + pi/2
    // where:
    // dp/dt  = speed_factor (piecewise constant)

    //tt_pos_ref[0] = tt_R*cosf(p);
    //tt_pos_ref[1] = tt_R*sinf(p);
    //tt_pos_ref[2] = -1.5f;

    //tt_vel_ref[0] = -tt_R*tt_speed_factor*sinf(p);
    //tt_vel_ref[1] = tt_R*tt_speed_factor*cosf(p);
    //tt_vel_ref[2] = 0.0f;

    //tt_acc_ref[0] = -tt_R*tt_speed_factor*tt_speed_factor*cosf(p);
    //tt_acc_ref[1] = -tt_R*tt_speed_factor*tt_speed_factor*sinf(p);
    //tt_acc_ref[2] = 0.0f;

    //tt_yaw_ref = p + M_PIf/2.0f;
    //tt_yaw_rate_ref = tt_speed_factor;

    // Lissajou trajectory from https://arxiv.org/pdf/2311.13081.pdf
    // cos(2πt/T ) sin(4πt/T )/2 const
    tt_pos_ref[0] = tt_R*cosf(p);
    tt_pos_ref[1] = tt_R*sinf(2.f*p)/2.f;
    tt_pos_ref[2] = -1.5f;

    tt_vel_ref[0] = -tt_R*tt_speed_factor*sinf(p);
    tt_vel_ref[1] = tt_R*tt_speed_factor*cosf(2.f*p);
    tt_vel_ref[2] = 0.0f;

    tt_acc_ref[0] = -tt_R*tt_speed_factor*tt_speed_factor*cosf(p);
    tt_acc_ref[1] = -tt_R*2.f*tt_speed_factor*tt_speed_factor*sinf(2.f*p);
    tt_acc_ref[2] = 0.0f;

    tt_yaw_ref = 0.f;
    tt_yaw_rate_ref = 0.f;
}

void getSetpointsTrajectoryTracker(void) {
    // pos error
    float x_error = tt_pos_ref[0] - posEstNed.V.X;
    float y_error = tt_pos_ref[1] - posEstNed.V.Y;
    float z_error = tt_pos_ref[2] - posEstNed.V.Z;

    // vel setpoint
    float Dgain = posRuntime.horz_d > 1.f  ?  posRuntime.horz_d  :  1.f;
    float Pgain = posRuntime.horz_p / Dgain;
    float vx_sp = tt_vel_ref[0] + Pgain*x_error;
    float vy_sp = tt_vel_ref[1] + Pgain*y_error;
    float vz_sp = tt_vel_ref[2] + Pgain*z_error;

    // acc setpoint
    tt_acc_sp[0] = tt_acc_ref[0] + Dgain*(vx_sp - velEstNed.V.X);
    tt_acc_sp[1] = tt_acc_ref[1] + Dgain*(vy_sp - velEstNed.V.Y);
    tt_acc_sp[2] = tt_acc_ref[2] + Dgain*(vz_sp - velEstNed.V.Z);

    // yaw error
    float yaw_error = tt_yaw_ref - DECIDEGREES_TO_RADIANS(attitude.angles.yaw);
    while (yaw_error > M_PIf)
        yaw_error -= 2.f * M_PIf;
    while (yaw_error < -M_PIf)
        yaw_error += 2.f * M_PIf;

    // yaw rate setpoint
    tt_yaw_rate_sp = tt_yaw_rate_ref + posRuntime.yaw_p*yaw_error;
}

void initTrajectoryTracker(void) {
    // reset everything
    tt_progress = 0.0f;
    tt_speed_factor = 0.0f;
    tt_active = false;

    // setpoint = starting point of trajectory
    getRefsTrajectoryTracker(0.0f);
    posSpNed.pos.V.X = tt_pos_ref[0];
    posSpNed.pos.V.Y = tt_pos_ref[1];
    posSpNed.pos.V.Z = tt_pos_ref[2];
    posSpNed.psi = tt_yaw_ref;
}

void setSpeedTrajectoryTracker(float speed) {
    tt_speed_factor = speed/tt_R;
    tt_active = true;
}

void stopTrajectoryTracker(void) {
    // set speed factor to zero, get new setpoints, and deactivate
    tt_speed_factor = 0.0f;
    getRefsTrajectoryTracker(tt_progress);
    getSetpointsTrajectoryTracker();
    tt_active = false;
}

void updateTrajectoryTracker(timeUs_t current) {
    // only does something when tt_active is true
    if (tt_active) {
        // Track trajectory and overwrite pos_ctl

        // update tt_progress
        tt_progress += tt_speed_factor*(current - last)*1e-6f;

        // update references
        getRefsTrajectoryTracker(tt_progress);

        // update acceleration and yaw rate setpoints
        getSetpointsTrajectoryTracker();

        // overwrite accSpNedFromPos and rateSpBodyFromPos (from pos_ctl.c)
        accSpNedFromPos.V.X = tt_acc_sp[0];
        accSpNedFromPos.V.Y = tt_acc_sp[1];
        accSpNedFromPos.V.Z = tt_acc_sp[2];

        rateSpBodyFromPos = coordinatedYaw(tt_yaw_rate_sp);

        // overwrite posSpNed (from pos_ctl.c)
        posSpNed.pos.V.X = tt_pos_ref[0];
        posSpNed.pos.V.Y = tt_pos_ref[1];
        posSpNed.pos.V.Z = tt_pos_ref[2];

        // overwrite velSetpointNed (from pos_ctl.c)
        posSpNed.vel.V.X = tt_vel_ref[0];
        posSpNed.vel.V.Y = tt_vel_ref[1];
        posSpNed.vel.V.Z = tt_vel_ref[2];

        // overwrite yawSetpoint (from pos_ctl.c)
        posSpNed.psi = tt_yaw_ref;
    }

    // update last
    last = current;
}

#endif
