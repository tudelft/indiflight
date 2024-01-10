#include "trajectory_tracker.h"

#include <math.h>
#include "flight/imu.h"
#include "io/external_pos.h"
#include "pos_ctl.h"


#ifdef USE_TRAJECTORY_TRACKER

#ifndef USE_POS_CTL
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_POS_CTL"
#endif

// state of trajectory tracker:
tt_state_t tt_state = INACTIVE;

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
float tt_pos_gain = 1.5;
float tt_vel_gain = 2.5;
float tt_yaw_gain = 1.0;

// radius of circular trajectory
float tt_R = 2.0f;

void setSpeedFactorTrajectoryTracker(float speed_factor) {
    tt_speed_factor = speed_factor;
}

void getRefsTrajectoryTracker(float p) {
    // hard coded circular trajectory with radius R
    // x(t)   = R*cos(p(t))
    // y(t)   = R*sin(p(t))
    // z(t)   = -1.5
    // psi(t) = p(t) + pi/2
    // where:
    // dp/dt  = speed_factor (piecewise constant)

    tt_pos_ref[0] = tt_R*cosf(p);
    tt_pos_ref[1] = tt_R*sinf(p);
    tt_pos_ref[2] = -1.5f;

    tt_vel_ref[0] = -tt_R*tt_speed_factor*sinf(p);
    tt_vel_ref[1] = tt_R*tt_speed_factor*cosf(p);
    tt_vel_ref[2] = 0.0f;

    tt_acc_ref[0] = -tt_R*tt_speed_factor*tt_speed_factor*cosf(p);
    tt_acc_ref[1] = -tt_R*tt_speed_factor*tt_speed_factor*sinf(p);
    tt_acc_ref[2] = 0.0f;

    tt_yaw_ref = p + M_PIf/2.0f;
    tt_yaw_rate_ref = tt_speed_factor;
}

void getSetpointsTrajectoryTracker(void) {
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

    // yaw error
    float yaw_error = tt_yaw_ref - DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    while (yaw_error > M_PIf)
        yaw_error -= 2.f * M_PIf;
    while (yaw_error < -M_PIf)
        yaw_error += 2.f * M_PIf;

    // yaw rate setpoint
    tt_yaw_rate_sp = tt_yaw_rate_ref + tt_yaw_gain*yaw_error;
}

void updateTrajectoryTracker(timeUs_t current) {
    switch (tt_state) {
        case INACTIVE: ;
            // wait for posSetpointNed to be set to {2., 0, -1.5}
            if (posSetpointNed.pos.V.X == 2.0f &&posSetpointNed.pos.V.Y == 0.0f && posSetpointNed.pos.V.Z == -1.5f) {
                // setpoint = starting point of trajectory
                getRefsTrajectoryTracker(0.0f);
                posSetpointNed.pos.V.X = tt_pos_ref[0];
                posSetpointNed.pos.V.Y = tt_pos_ref[1];
                posSetpointNed.pos.V.Z = tt_pos_ref[2];
                posSetpointNed.psi = tt_yaw_ref;
                
                // set state to INIT
                tt_state = INIT;
            }
            break;

        case INIT: ;
            // wait until drone reaches starting point
            float dx = posEstNed.V.X - posSetpointNed.pos.V.X;
            float dy = posEstNed.V.Y - posSetpointNed.pos.V.Y;
            float dz = posEstNed.V.Z - posSetpointNed.pos.V.Z;
            float dpsi = DECIDEGREES_TO_RADIANS(attitude.values.yaw) - posSetpointNed.psi;

            // wrap yaw error
            while (dpsi > M_PIf)
                dpsi -= 2.f * M_PIf;
            while (dpsi < -M_PIf)
                dpsi += 2.f * M_PIf;

            if (fabsf(dx) < 0.1f && fabsf(dy) < 0.1f && fabsf(dz) < 0.1f && fabsf(dpsi) < 0.1f) {
                // INITIALIZE TRAJECTORY TRACKER
                tt_speed_factor = 0.0f;
                tt_progress = 0.0f;
                last = current;
                tt_state = ACTIVE;

                // UGLY HACK: we set posSetpointNed.psi to 0.0f and use it as communication for setting tt_speed_factor
                posSetpointNed.psi = 0.0f;                
            }
            break;

        case ACTIVE: ;
            // Track trajectory and overwrite pos_ctl
            
            // update tt_progress
            tt_progress += tt_speed_factor*(current - last)*1e-6f;

            // update references
            getRefsTrajectoryTracker(tt_progress);

            // update acceleration and yaw rate setpoints
            getSetpointsTrajectoryTracker();

            // overwrite accSpNed and yawRateSpFromOuter (from pos_ctl.c)
            accSpNed.V.X = tt_acc_sp[0];
            accSpNed.V.Y = tt_acc_sp[1];
            accSpNed.V.Z = tt_acc_sp[2];
            yawRateSpFromOuter = tt_yaw_rate_sp;

            // overwrite posSetpointNed (from pos_ctl.c)
            posSetpointNed.pos.V.X = tt_pos_ref[0];
            posSetpointNed.pos.V.Y = tt_pos_ref[1];
            posSetpointNed.pos.V.Z = tt_pos_ref[2];

            // overwrite velSetpointNed (from pos_ctl.c)
            posSetpointNed.vel.V.X = tt_vel_ref[0];
            posSetpointNed.vel.V.Y = tt_vel_ref[1];
            posSetpointNed.vel.V.Z = tt_vel_ref[2];

            // overwrite yawSetpoint (from pos_ctl.c)
            // posSetpointNed.psi = tt_yaw_ref;

            // UGLY HACK: we set speed factor using posSetpointNed.psi
            float speed = RADIANS_TO_DEGREES(posSetpointNed.psi);       // speed in m/s
            tt_speed_factor = speed/tt_R;                               // rad/s

            // if speed is negative, set state to EXIT
            if (tt_speed_factor < 0.0f) {
                tt_state = EXIT;
            }
            // update last
            last = current;
            break;
    
        case EXIT: ;
            // reset stuff
            posSetpointNed.psi = 0.0f;
            tt_speed_factor = 0.0f;
            tt_progress = 0.0f;

            tt_state = INACTIVE;

            break;
    }
}

#endif