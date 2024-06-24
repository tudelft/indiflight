#include "nn_control.h"
#include "ekf_calc.h"			// FOR NOW: hard coded to use ekf states
#include "sensors/gyro.h"		// for gyro
#include "flight/indi.h"		// for omega
#include "io/external_pos.h"	// for setpoints
#include "pos_ctl.h"			// for resetIterms

#include "flight/neural_controllers/nn_controller.h"

#ifdef USE_NN_CONTROL

#ifndef USE_EKF
#error "USE_NN_CONTROL requires the use of USE_EKF"
#endif

//#ifndef USE_GPS_PI  // implicit in EKF
//#error "USE_NN_CONTROL requires the use of USE_GPS_PI"
//#endif

#ifndef USE_POS_CTL
#error "USE_NN_CONTROL requires the use of USE_POS_CTL"
#endif

#pragma message "You are compiling with dangerous code!"

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
}

void nn_activate(void) {
	// only activate when the drone is at start point (within 0.5 meters)
	if ((fabsf(ekf_get_X()[0] - start_pos[0]) < 0.5f) &&
		(fabsf(ekf_get_X()[1] - start_pos[1]) < 0.5f) &&
		(fabsf(ekf_get_X()[2] - start_pos[2]) < 0.5f)) {
		nn_active = true;
		
		// initialize the neural network controller
		nn_reset();
	}	
}

void nn_deactivate(void) {
	nn_active = false;
	// current pos, yaw becomes the setpoint
	posSpNed.pos.V.X =  0.0f; //ekf_get_X()[0];
	posSpNed.pos.V.Y =  0.0f; //ekf_get_X()[1];
	posSpNed.pos.V.Z = -1.5f; //ekf_get_X()[2];
	posSpNed.psi = ekf_get_X()[8];

	// posSpNed.pos.V.X = gate_pos[target_gate_index+1][0];
	// posSpNed.pos.V.Y = gate_pos[target_gate_index+1][1];
	// posSpNed.pos.V.Z = gate_pos[target_gate_index+1][2];
	// posSpNed.psi = gate_yaw[target_gate_index+1];

    // reset I terms
    resetIterms();
}

bool nn_is_active(void) {
	return nn_active;
}

void nn_compute_motor_cmds(void) {
	// compute motor commands based on the world_state[16] (pos, vel, att, rate, motorspeeds)
	float world_state[16] = {0.};
	float* ekf_state = ekf_get_X();
	// pos NED
	world_state[0] = ekf_state[0];
	world_state[1] = ekf_state[1];
	world_state[2] = ekf_state[2];
	// vel NED
	world_state[3] = ekf_state[3];
	world_state[4] = ekf_state[4];
	world_state[5] = ekf_state[5];
	// att
	world_state[6] = ekf_state[6];
	world_state[7] = ekf_state[7];
	world_state[8] = ekf_state[8];
	// rate (in FRD)
	world_state[9]  = DEGREES_TO_RADIANS(gyro.gyroADCf[0]); // TODO: figure out if we need gyroADCf or gyroADC
	world_state[10] = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
	world_state[11] = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);
	// motorspeeds
	world_state[12] = (float) indiRun.omega[0];
	world_state[13] = (float) indiRun.omega[1];
	world_state[14] = (float) indiRun.omega[2];
	world_state[15] = (float) indiRun.omega[3];

	// call the neural network controller (output is in range [0,1])
	nn_control(world_state, nn_motor_cmds);

	// set nn_motor_cmds to 0.1
	// for (int i = 0; i < 4; i++) {
	// 	nn_motor_cmds[i] = 0.1;
	// }
}

float* nn_get_motor_cmds(void) {
	return nn_motor_cmds;
}

#endif