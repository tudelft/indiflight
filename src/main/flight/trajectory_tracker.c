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
#include "sensors/acceleration.h"
#include "trajectories/min_snap.h"


#ifdef USE_TRAJECTORY_TRACKER

#ifndef USE_POS_CTL
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_POS_CTL"
#endif

#ifndef USE_TELEMETRY_PI
#error "USE_TRAJECTORY_TRACKER only works in combination with USE_TELEMETRY_PI"
#endif

// heading modes
tt_heading_mode_t tt_heading_mode = TT_LOOK_AT_NOTHING;

// state of trajectory tracker:
bool tt_active = false;

// reference trajectory
#define MAX_SUPPORTED_GATES 50
fp_vector_t tt_pos_ref = {0};
fp_vector_t tt_vel_ref = {0};
fp_vector_t tt_acc_ref = {0};
fp_vector_t tt_jerk_ref = {0};
float tt_heading_ref = 0.;
float tt_heading_rate_ref = 0.;
float tt_start_p = 0.f;
gate_t gates[MAX_SUPPORTED_GATES];
int n_gates = 0;

// time stuff
float tt_speed_factor = 0.0f;
float tt_progress = 0.0f;
float tt_time = 0.0f;
timeUs_t last = 0;

// gains
float tt_pos_gain = 1.5; //1.5;
float tt_vel_gain = 2.5; //2.5;
float tt_acc_gain = 0.f; // zero, because we use attitude feedback control in indi.c
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
    bool straight_recovery; // recover in a straight line
    float x_init;    // initial x position
    float y_init;    // initial y position
    float vx_init;   // initial x velocity
    float vy_init;   // initial y velocity
} tt_recovery_problem;

void initRecoveryMode(void) {
    // check current position to determine the radius of the recovery trajectory
    float x = posEstNed.V.X;
    float y = posEstNed.V.Y;
    tt_recovery_problem.R = sqrtf(x*x + y*y);
    tt_recovery_problem.straight_recovery = false;

    // final time hardcoded
    tt_recovery_problem.tf = 1.0f;

    // make sure radius is less then 3.0
    if ((tt_recovery_problem.R < 1.5) || (tt_recovery_problem.R > 3.0)) {
        tt_recovery_problem.straight_recovery = true;
        tt_recovery_problem.tf = .6f;
        tt_recovery_problem.x_init = x;
        tt_recovery_problem.y_init = y;
        tt_recovery_problem.vx_init = velEstNed.V.X;
        tt_recovery_problem.vy_init = velEstNed.V.Y;
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

    // Circular recovery
    if (tt_recovery_problem.straight_recovery) {
        // constant acceleration bringing v_init to zero
        float vx_init = tt_recovery_problem.vx_init;
        float vy_init = tt_recovery_problem.vy_init;
        float x_init = tt_recovery_problem.x_init;
        float y_init = tt_recovery_problem.y_init;

        tt_pos_ref.V.X = x_init + vx_init*t - 0.5f*vx_init/tf*t*t;
        tt_pos_ref.V.Y = y_init + vy_init*t - 0.5f*vy_init/tf*t*t;
        tt_pos_ref.V.Z = -1.5f;

        tt_vel_ref.V.X = vx_init - (vx_init/tf)*t;
        tt_vel_ref.V.Y = vy_init - (vy_init/tf)*t;
        tt_vel_ref.V.Z = 0.0f;

        tt_acc_ref.V.X = -vx_init/tf;
        tt_acc_ref.V.Y = -vy_init/tf;
        tt_acc_ref.V.Z = 0.0f;

        tt_jerk_ref.V.X = 0.0f;
        tt_jerk_ref.V.Y = 0.0f;
        tt_jerk_ref.V.Z = 0.0f;

        // heading (we dont want to track that in recovery though)
        tt_heading_rate_ref = 0.;
        posSpNed.trackPsi = false;
    } else {
        float omega = omega0*(1.0f - t/tf);                 // angular velocity
        float theta = theta0 + omega0*(t - 0.5f*t*t/tf);    // angle along the circle
        while (theta >  M_PIf) theta -= (2.0f * M_PIf);  // always wrap input angle to -PI..PI
        while (theta < -M_PIf) theta += (2.0f * M_PIf);

        // position refs
        tt_pos_ref.V.X = R*cosf(theta);
        tt_pos_ref.V.Y = R*sinf(theta);
        tt_pos_ref.V.Z = -1.5f;

        // velocity refs
        tt_vel_ref.V.X = -R*sinf(theta)*omega;
        tt_vel_ref.V.Y = R*cosf(theta)*omega;
        tt_vel_ref.V.Z = 0.0f;

        // acceleration refs
        tt_acc_ref.V.X = -R*cosf(theta)*omega*omega -R*sinf(theta)*(-omega0/tf);
        tt_acc_ref.V.Y = -R*sinf(theta)*omega*omega +R*cosf(theta)*(-omega0/tf);
        tt_acc_ref.V.Z = 0.0f;

        tt_jerk_ref.V.X = 0.0f;
        tt_jerk_ref.V.Y = 0.0f;
        tt_jerk_ref.V.Z = 0.0f;

        // heading (we dont want to track that in recovery though)
        tt_heading_rate_ref = 0.;
        posSpNed.trackPsi = false;
    }
}

bool isActiveTrajectoryTracker(void) {
    return tt_active;
}

bool isActiveTrajectoryTrackerRecovery(void) {
    return tt_recovery_active;
}

/* circle
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
    tt_heading_ref = p + M_PIf/2.0f;
    tt_heading_rate_ref = tt_speed_factor;
    posSpNed.trackPsi = tt_track_heading; // choice: track heading or neglect it?
}
*/

static float wrap_2pi(float p) {
    while (p >  2.0f*M_PIf) p -= (2.0f * M_PIf);  // always wrap input angle to 0..PI
    while (p < 0.f) p += (2.0f * M_PIf);
    return p;
}

static float wrap_mpi_pi(float p) {
    while (p > M_PIf) p -= (2.0f * M_PIf);  // always wrap input angle to 0..PI
    while (p < -M_PIf) p += (2.0f * M_PIf);
    return p;
}

void getRefsTrajectoryTracker(float p) {
    UNUSED(wrap_2pi); // optional
    // x(t)   = -R*sin(p)*cos(p)
    // y(t)   = +R*sin(p)
    // z(t)   = -1.0
    // psi(t) = look at next gate
    // where:
    // dp/dt  = speed_factor (piecewise constant)

    /*
    float laptime = 1000.f;
    float lam = 0.f;
    if (fabsf(tt_speed_factor) > 0.1f) {
        laptime = tt_R*5.f / tt_speed_factor; // estiamte: lap is 15 meter long for 3 radius
        lam = 1.f / laptime;
    }

    float p = lam * t;
    while (p >  2.0f*M_PIf) p -= (2.0f * M_PIf);  // always wrap input angle to 0..PI
    while (p < 0.f) p += (2.0f * M_PIf);

    if (lam != 0.f) {
        // update progress, incase we wrapped
        tt_progress = p / lam;
    }
    */

    // ----- circle
    /*
    tt_progress = warp_2pi(tt_progress);
    float p = tt_progress;

    // position refs
    tt_pos_ref.V.X = tt_R*cosf(p); //tt_R*(-sinf(p)*cosf(p));
    tt_pos_ref.V.Y = tt_R*sinf(p); //tt_R*(+sinf(p));
    tt_pos_ref.V.Z = -1.0f;

    // velocity refs
    tt_vel_ref.V.X = -tt_R*lam*sinf(p);  //-tt_R*lam*(cosf(p)*cosf(p) - sinf(p)*sinf(p));
    tt_vel_ref.V.Y = tt_R*lam*cosf(p);   //tt_R*lam*cosf(p);
    tt_vel_ref.V.Z = 0.0f;

    // acceleration refs
    tt_acc_ref.V.X = -tt_R*lam*lam*cosf(p);      //4*lam*lam * tt_pos_ref[0];
    tt_acc_ref.V.Y = -tt_R*lam*lam*sinf(p);      //-lam*lam * tt_pos_ref[1];
    tt_acc_ref.V.Z = 0.0f;

    // jerk refs
    tt_jerk_ref.V.X = +tt_R*lam*lam*lam*sinf(p);
    tt_jerk_ref.V.Y = -tt_R*lam*lam*lam*cosf(p);
    tt_jerk_ref.V.Z = 0.0f;

    // heading reference
    tt_heading_ref = 0.0f;
    tt_heading_rate_ref = 0.0f;
    posSpNed.trackPsi = false;
    */

    // ----- figure 8
    /*
    tt_progress = warp_2pi(tt_progress);
    float p = tt_progress;
    gates[0] = (gate_t){.x = 0.f, .y = 0.f, .p = 0.}; // currently broken
    float gates[4][3] = {
        {0.f, 0.f},
        {0.f, 3.f},
        {0.f, 0.f},
        {0.f, -3.f},
    };


    float lam;
    lam = tt_speed_factor;

    // position refs
    tt_pos_ref.V.X = tt_R*(-sinf(p)*cosf(p));
    tt_pos_ref.V.Y = tt_R*(+sinf(p));
    tt_pos_ref.V.Z = -1.0f;

    // velocity refs
    tt_vel_ref.V.X = tt_R*lam*(sinf(p)*sinf(p) - cosf(p)*cosf(p));
    tt_vel_ref.V.Y = tt_R*lam*cosf(p);
    tt_vel_ref.V.Z = 0.0f;

    // acceleration refs
    tt_acc_ref.V.X = -4*lam*lam * tt_pos_ref.V.X;
    tt_acc_ref.V.Y = -lam*lam * tt_pos_ref.V.Y;
    tt_acc_ref.V.Z = 0.0f;

    // jerk refs
    tt_jerk_ref.V.X = -4*lam*lam * tt_vel_ref.V.X;
    tt_jerk_ref.V.Y = -lam*lam * tt_vel_ref.V.Y;
    tt_jerk_ref.V.Z = 0.0f;

    // heading default for this trajectory
    tt_heading_ref = 0.0f;
    tt_heading_rate_ref = 0.0f;
    posSpNed.trackPsi = false;
    */

    // ----- min snap
    // because we plug in p = tt_speed_factor*t
    // we have to apply the chain rule to get the derivatives
    // position refs
    tt_pos_ref.V.X = get_x(p);
    tt_pos_ref.V.Y = get_y(p);
    tt_pos_ref.V.Z = get_z(p);

    // velocity refs
    tt_vel_ref.V.X = tt_speed_factor*get_vx(p);
    tt_vel_ref.V.Y = tt_speed_factor*get_vy(p);
    tt_vel_ref.V.Z = tt_speed_factor*get_vz(p);

    // acceleration refs
    tt_acc_ref.V.X = tt_speed_factor*tt_speed_factor*get_ax(p);
    tt_acc_ref.V.Y = tt_speed_factor*tt_speed_factor*get_ay(p);
    tt_acc_ref.V.Z = tt_speed_factor*tt_speed_factor*get_az(p);

    // jerk refs
    tt_jerk_ref.V.X = tt_speed_factor*tt_speed_factor*tt_speed_factor*get_jx(p);
    tt_jerk_ref.V.Y = tt_speed_factor*tt_speed_factor*tt_speed_factor*get_jy(p);
    tt_jerk_ref.V.Z = tt_speed_factor*tt_speed_factor*tt_speed_factor*get_jz(p);

    // heading
    tt_heading_ref = get_psi(p);
    tt_heading_rate_ref = tt_speed_factor*get_psi_dot(p);
    posSpNed.trackPsi = true;

    // switch case depnding on tt_heading_mode
    switch (tt_heading_mode) {
        case TT_LOOK_AT_REF:
            // default mode, just use what was defined above
            break;
        case TT_LOOK_AT_GATES:
            // broken, should fix it some time, now falltthrough
/*
            if (gates && n_gates) {

            }
            float sf = 0.1f * M_PIf / 2.f;

            int g = 1;
            if (((0.5f * M_PIf - sf) <= p) && (p <= (1.0f * M_PIf - sf))) {
                g = 2;
            } else
            if (((1.0f * M_PIf - sf) <= p) && (p <= (1.5f * M_PIf - sf))) {
                g = 3;
            }
            if (((1.5f * M_PIf - sf) <= p) && (p <= (2.0f * M_PIf - sf))) {
                g = 0;
            }

            fp_vector_t dg = { 
                .V.X = gates[g][0] - tt_pos_ref.V.X,
                .V.Y = gates[g][1] - tt_pos_ref.V.Y,
                .V.Z  = 0.f // unused
            };

            float d2 = dg.V.X*dg.V.X  +  dg.V.Y*dg.V.Y; // x**2 + y**2
            tt_heading_ref = atan2f(dg.V.Y, dg.V.X); // look at next gate

            // total derivative of the atan2: -gy / d2 * dgx/dt  +  gx / d2 * dgy/dt
            // total derivative of the atan2: -gy / d2 * (-dx/dt)  +  gx / d2 * (-dy/dt)
            if (d2 < 0.01f) { // within 10cm of next gate, do nothing. this should never happen, because switchover to the next gate should happen sooner
                tt_heading_rate_ref = 0.f;
                posSpNed.trackPsi = false;
            } else {
                tt_heading_rate_ref = (dg.V.Y * tt_vel_ref.V.X  -  dg.V.X * tt_vel_ref.V.Y) / d2;
                posSpNed.trackPsi = true; // choice: track heading or neglect it?
            }
            break;   
*/
        case TT_LOOK_AT_NOTHING:
            // do nothing
            tt_heading_rate_ref = 0.f;
            posSpNed.trackPsi = false;
            break;
        case TT_LOOK_ALONG_VELOCITY:
            {
            // heading should align with velocity ref
            float d2 = tt_vel_ref.V.X*tt_vel_ref.V.X  +  tt_vel_ref.V.Y*tt_vel_ref.V.Y; // x**2 + y**2
            tt_heading_ref = atan2f(tt_vel_ref.V.Y, tt_vel_ref.V.X); // look along velocity ref vector

            // total derivative of the atan2: -vy / d2 * dvx/dt  +  vx / d2 * dvy/dt
            if (d2 < 0.01f) { // velocity too small, do nothing
                tt_heading_rate_ref = 0.f;
                posSpNed.trackPsi = false;
            } else {
                tt_heading_rate_ref = (-tt_vel_ref.V.Y * tt_acc_ref.V.X  +  tt_vel_ref.V.X * tt_acc_ref.V.Y) / d2;
                posSpNed.trackPsi = true; // choice: track heading or neglect it?
            }
            break;
            }
    }
}

void initTrajectoryTracker(void) {
    // reset everything
    tt_progress = 0; // todo: make option to start just infront of gate 0
    tt_speed_factor = 0.0f;
    tt_heading_ref = 0.f;
    //tt_active = true; //dont activate yet, just go the starting point with default controller

    // setpoint = starting point of trajectory
    getRefsTrajectoryTracker(tt_progress);
    posSpNed.pos.V.X = tt_pos_ref.V.X;
    posSpNed.pos.V.Y = tt_pos_ref.V.Y;
    posSpNed.pos.V.Z = tt_pos_ref.V.Z;
    posSpNed.psi = wrap_mpi_pi(tt_heading_ref);
    posSpNed.trackPsi = true;
    posSetpointState = EXT_POS_NEW_MESSAGE;
}

void setSpeedTrajectoryTracker(float speed) {
    tt_speed_factor = speed;
    tt_active = true;
}

void incrementSpeedTrajectoryTracker(float inc) {
    tt_speed_factor += inc;
    tt_active = true;
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

        // update velocity setpoints
        // pos error = tt_pos_ref - posEstNed
        fp_vector_t pos_error = tt_pos_ref;
        VEC3_SCALAR_MULT_ADD(pos_error, -1.f, posEstNed); // subtract estiamte from reference

        // vel setpoint = tt_vel_ref + tt_pos_gain * pos_error
        fp_vector_t vel_sp = tt_vel_ref;
        VEC3_SCALAR_MULT_ADD(vel_sp, tt_pos_gain, pos_error);

        // update acceleration setpoints
        // vel error = vel_sp - velEstNed
        fp_vector_t vel_error = vel_sp;
        VEC3_SCALAR_MULT_ADD(vel_error, -1.f, velEstNed); // subtract estiamte from reference

        // acc setpoint = tt_acc_ref + tt_vel_gain * vel_error
        fp_vector_t acc_sp = tt_acc_ref;
        VEC3_SCALAR_MULT_ADD(acc_sp, tt_vel_gain, vel_error);

        // update jerk setpoints
        // acc estimate in NED
        fp_vector_t accEstNed = { .V.X = acc.accADCf[0], .V.Y = acc.accADCf[1], .V.Z = acc.accADCf[2] };
        rotate_vector_with_rotationMatrix(&accEstNed, &rMat);
        VEC3_SCALAR_MULT(accEstNed, GRAVITYf / acc.dev.acc_1G);
        accEstNed.V.Z += GRAVITYf; // remove gravity from accelerometer

        // acc error = acc_sp - accEstNed
        fp_vector_t acc_error = acc_sp;
        VEC3_SCALAR_MULT_ADD(acc_error, -1.f, accEstNed); // subtract estiamte from reference

        // jerk setpoint = tt_jerk_ref + tt_acc_gain * acc_error
        fp_vector_t jerk_sp = tt_jerk_ref;
        VEC3_SCALAR_MULT_ADD(jerk_sp, tt_acc_gain, acc_error);

        // overwrite set points velSetpointNed (from pos_ctl.c)
        posSpNed.pos = tt_pos_ref;
        posSpNed.vel = vel_sp;
        accSpNedFromPos = acc_sp; 
        // todo: do attitude conversion here, because looking at gates is actually more complex than just a psi reference

        // transform jerk to body rates
        // some math:
        //   - derivative of rotation matrix is   Rdot = R omega_skew, where omega_skew is a skew symmetric matrix of body rates
        //   - omega_skew = [ 0 -r q; r 0 -p; -q p 0 ]
        //   - derivative of a norm d/dt ||r|| = rT r_dot / ||r||
        // drag model:
        //   dB = k \circ vB
        //   dI = R k \circ vB = R (k \circ R-1 vI)
        //   dI_dot = R omega_skew (k \circ R**T vI)  +  R (k \circ (R omega_skew)**T vI)  +  R (k \circ R**T aI)
        //   dI_dot ~ R (k \circ R**T aI)   // impossible to solve with omega_skew in there
        // some models
        //   aI = R (0,0,fzb)  +  gI  +  dI
        //   fzb = || aI - gI - dI ||
        //   fzI_n := (aI - gI - dI) / || aI - gI - dI ||
        //   fzb_dot = (aI - gI - dI)**T (jI - dI_dot) / || aI - gI - dI ||
        //   fzb_dot = fzI_n**T (jI - dI_dot)

        // finally, take aI derivative to solve for wx, wy
        //   aI = R (0,0,fzb)  +  gI  +  dI
        //   jI = R omega_skew (0,0,fzb)  +  R (0,0,fzb_dot)  +  dI_dot
        //   jI - dI_dot = fzb * R (-q p 0)**T  +  R (0,0,fzI_n**T (jI - dI_dot))
        //   R**T (jI - dI_dot) / fzb = (-q p 0)**T  +  (0,0,fzI_n**T (jI - dI_dot))
        //   (-q p)**T = I_(2x3) R**T (jI - dI_dot) / fzb
        //   (-q p)**T ~ I_(2x3) R**T (jI + R (k \circ R**T aI)) / || aI - gI + R (k \circ R**T vI) ||

        // get rotation matrix inverse from state
        fp_rotationMatrix_t rMatT = rMat;
        rMatT.m[0][1] = rMat.m[1][0];
        rMatT.m[0][2] = rMat.m[2][0];
        rMatT.m[1][0] = rMat.m[0][1];
        rMatT.m[1][2] = rMat.m[2][1];
        rMatT.m[2][0] = rMat.m[0][2];
        rMatT.m[2][1] = rMat.m[1][2];

        //fp_vector_t drag_pars = { .V.X = -0.f, .V.Y = -0.f, .V.Z = 0.f };
        fp_vector_t drag_pars = { .V.X = -0.5f, .V.Y = -0.5f, .V.Z = 0.f };

        fp_vector_t dI = posSpNed.vel;
        rotate_vector_with_rotationMatrix(&dI, &rMatT); // todo: we may need rMat from trajectory not state?
        VEC3_ELEM_MULT(dI, drag_pars);
        rotate_vector_with_rotationMatrix(&dI, &rMat); 

        fp_vector_t dI_dot = posSpNed.vel;
        rotate_vector_with_rotationMatrix(&dI_dot, &rMatT); // todo: not q, we need the q from trajectory
        VEC3_ELEM_MULT(dI_dot, drag_pars);
        rotate_vector_with_rotationMatrix(&dI_dot, &rMat); 

        //   R (0,0,fzb)  =  aI  -  gI  -  dI
        fp_vector_t Rfzb = accSpNedFromPos; 
        VEC3_SCALAR_MULT_ADD(Rfzb, -1.f, dI);
        Rfzb.V.Z -= GRAVITYf; // signs?

        //  fzb  =  || R (0,0,fzb) ||
        float fzb = (VEC3_LENGTH(Rfzb) > 0.01f) ? VEC3_LENGTH(Rfzb) : 0.01f;

        // finally, (-q p)**T = I_(2x3) R**T (jI - dI_dot) / fzb
        // finally, (-q p 0)**T = R**T (jI - dI_dot) / fzb
        fp_vector_t minq_p_0 = tt_jerk_ref;
        VEC3_SCALAR_MULT_ADD(minq_p_0, -1.f, dI_dot);
        VEC3_SCALAR_MULT(minq_p_0, 1.f / fzb);
        rotate_vector_with_rotationMatrix(&minq_p_0, &rMatT);

        rateSpBodyFromPos.V.X = minq_p_0.V.Y;
        rateSpBodyFromPos.V.Y = -minq_p_0.V.X;
        rateSpBodyFromPos.V.Z = 0.;

        // keep looking at fix point xf
        // (q R_cam nxB q-)_xy

        // solve for wz to keep looking at a fix point xf
        //   - this is a complementary contraint:  (xf - xI)**T nyI = 0, where nyI is the y axis of the body in inertial coordinates. (also (xf - xI)**T nxI >= 0)
        //   - the attitude mapper (posGetAttSpNedAndSpfSpBody) already satisfies this constraint, we need to now compute wz to track it (to first order)
        //   - notice that nyI = R (0 1 0)**T, so nyI_dot = R omega_skew (0 1 0)**T
        //   - also, define d = (xf - xI)
        //   - derivative of d**T nyI = 0:
        //       (0 - vI)**T nyI  +  d**T nyI_dot  =  0
        //       vI**T nyI  =  d**T R omega_skew (0 1 0)**T
        //       vI**T nyI  =  d**T R (wz 0 -wx)**T
        //       vI**T nyI  =  d**T (nxI wz - nzI wx)
        //       d**T (nxI wz)  =  vI**T nyI  +  (xf - xI)**T (nzI wx)
        //       (d**T nxI) wz  =  vI**T nyI  +  (xf - xI)**T (nzI wx)
        //       wz  =  ( vI**T nyI  +  d**T (nzI wx) ) / (d**T nxI)
        // boom, roasted. not tested yet, seems hard to implement/debug

        // overwrite yawSetpoint (from pos_ctl.c)
        posSpNed.psi = wrap_mpi_pi(tt_heading_ref);

        posSetpointState = EXT_POS_NEW_MESSAGE;

        fp_euler_t eulers;
        fp_euler_of_i16_euler(&eulers, &attitude);
        float cosphi = cosf(eulers.angles.roll);
        if ((posSpNed.trackPsi) && (fabsf(cosphi) > 0.05)) {
            // we can use yaw rate r feedforward to improve heading tracking
            //  equation is  psid*cos(theta) = q sin(phi) + r cos(phi)
            //  solves to r = ( psid*cos(theta) - q sin(phi) )  /  cos(phi)
            float sinphi = sinf(eulers.angles.roll);
            float costheta = sinf(eulers.angles.pitch);
            float q = rateSpBodyFromPos.V.Y; // or take actual?
            float r = (tt_heading_rate_ref * costheta - q * sinphi ) / cosphi;
            rateSpBodyFromPos.V.Z += r;
        }

        // overwrite rateSpBody 
        //if (posSpNed.trackPsi) {
        //    rateSpBodyFromPos = coordinatedYaw(tt_heading_rate_ref);
        //} else {
        //    rateSpBodyFromPos.V.X = 0;
        //    rateSpBodyFromPos.V.Y = 0;
        //    rateSpBodyFromPos.V.Z = 0;
        //}
    }

    // update last
    last = current;
}


#endif
