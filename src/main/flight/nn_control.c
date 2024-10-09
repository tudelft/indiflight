/*
 *
 *
 * Copyright 2024 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Added parameter group and compile time macros
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

#include "nn_control.h"
#include "ekf_calc.h"			// FOR NOW: hard coded to use ekf states
#include "sensors/gyro.h"		// for gyro
#include "flight/indi.h"		// for omega
#include "flight/trajectory_tracker.h"		// for recovery mode
#include "io/external_pos.h"	// for setpoints
#include "pos_ctl.h"			// for resetIterms
#include "pg/pg_ids.h"              // for config
#include "fc/runtime_config.h"  // for ENABLE_FLIGHT_MODE
#include "fc/core.h"            // for resetInnerLoopCounter

#include "flight/neural_controllers/nn_controller.h"

#ifdef USE_NN_CONTROL

#ifndef USE_EKF
#error "USE_NN_CONTROL requires the use of USE_EKF"
#endif

#ifndef USE_POS_CTL
#error "USE_NN_CONTROL requires the use of USE_POS_CTL"
#endif

#ifndef USE_TRAJECTORY_TRACKER // for recovery mode
#error "USE_NN_CONTROL requires the use if USE_TRAJECTORY_TRACER"
#endif

#pragma message "You are compiling with dangerous code!"

PG_REGISTER_WITH_RESET_TEMPLATE(nnConfig_t, nnConfig, PG_NN_CONFIG, 0);
PG_RESET_TEMPLATE(nnConfig_t, nnConfig, 
    .rate_denom = 20,
); 

float nn_motor_cmds[4] = {0.};
bool nn_active = false;

void nn_init(void) {
	// initialize the neural network controller
	nn_reset();
	// set starting point
	posSpNed.pos.V.X = start_pos[0];
	posSpNed.pos.V.Y = start_pos[1];
	posSpNed.pos.V.Z = start_pos[2];
	posSpNed.psi = start_yaw;
    posSpNed.trackPsi = true;
    posSetpointState = EXT_POS_NEW_MESSAGE;
}

void nn_activate(void) {
	// only activate when the drone is at start point (within 0.5 meters)
	if (FLIGHT_MODE(POSITION_MODE)) {

        nn_active = true;

		// initialize the neural network controller
		nn_reset();

        // enable flight mode:
        // --> this will cause taskMainInnerLoop to run nn_compute_motor_cmds()
        ENABLE_FLIGHT_MODE(NN_MODE);
        resetInnerLoopCounter();
	}
}

void nn_deactivate(void) {
	nn_active = false;

	// recovery mode
	initRecoveryMode();

    // reset I terms
    resetIterms();

    // disable flight mode:
    // --> this will cause taskMainInnerLoop to run nn_compute_motor_cmds()
    DISABLE_FLIGHT_MODE(NN_MODE);
}

bool nn_is_active(void) {
	return nn_active;
}

void nn_compute_motor_cmds(void) {
	// compute motor commands based on the world_state[16] (pos, vel, att, rate, motorspeeds)
    // todo: use system state and not hardcoded EKF
	float world_state[16] = {0.};
	float* ekf_state = ekf_get_X();
	// pos NED
	world_state[0] = ekf_state[0];
	world_state[1] = ekf_state[1];
	world_state[2] = ekf_state[2];
	// UGLY HACK:
	// world_state[2] -= 0.4; // add 0.4 meters to the z position
	// vel NED
	world_state[3] = ekf_state[3];
	world_state[4] = ekf_state[4];
	world_state[5] = ekf_state[5];
	// att
    fp_quaternion_t q = { .w = ekf_state[6], .x = ekf_state[7], .y = ekf_state[8], .z = ekf_state[9] };
    fp_quaternionProducts_t qp;
    fp_euler_t eulers;
    quaternionProducts_of_quaternion(&qp, &q);
    fp_euler_of_quaternionProducts(&eulers, &qp);
	world_state[6] = eulers.angles.roll;
	world_state[7] = eulers.angles.pitch;
	world_state[8] = eulers.angles.yaw;
	// rate (in FRD)
	world_state[9]  = DEGREES_TO_RADIANS(gyro.gyroADCf[0]); // TODO: figure out if we need gyroADCf or gyroADC
	world_state[10] = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
	world_state[11] = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);
	// unfiltered motorspeeds
	world_state[12] = (float) indiRun.omega[0];
	world_state[13] = (float) indiRun.omega[1];
	world_state[14] = (float) indiRun.omega[2];
	world_state[15] = (float) indiRun.omega[3];

	// call the neural network controller (output is in range [0,1])
	nn_control(world_state, nn_motor_cmds);

    if (cmpTimeUs(micros(), extLatestMsgTimeReceived) > NN_DEADRECKONING_TIMEOUT_US) {
        // deadreckoning for too long --> abort
        nn_deactivate(); // fallback on position
    }

	// DEACTIVE WHEN IN DANGER ZONE
	// danger zone:
	// abs(x) > 3.0
	// abs(y) > 8.0
	// abs(z) > 3.5
	if (fabsf(world_state[0]) > 3.0f || fabsf(world_state[1]) > 8.0f || fabsf(world_state[2]) > 3.5f) {
		nn_deactivate();
	}


	// set nn_motor_cmds to 0.1
	// for (int i = 0; i < 4; i++) {
	// 	nn_motor_cmds[i] = 0.1;
	// }
}

float* nn_get_motor_cmds(void) {
	return nn_motor_cmds;
}

#endif
