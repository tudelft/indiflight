/*
 * 
 *
 * Copyright 2023 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Improved integration with legacy estimator, added parameters
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

#include "ekf.h"

#include "io/local_pos.h"  		// for extPosNed
#include "fc/runtime_config.h"		// for FLIGHT_MODE
#include "common/maths.h"      		// for DEGREES_TO_RADIANS
#include "sensors/gyro.h"			// for gyro
#include "sensors/acceleration.h"   // for acc
#include "imu.h"                    // for fallback if no GPS
#include "pi-messages.h"            // for keeping track of message times

#include "pg/pg_ids.h"              // for config

#ifdef USE_EKF

#ifndef USE_ACC
#error "USE_EKF requires USE_ACC"
#endif

#ifndef USE_GYRO
#error "USE_EKF requires USE_GYRO"
#endif

#ifndef USE_LOCAL_POSITION_PI
#error "USE_EKF requires USE_LOCAL_POSITION_PI"
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(ekfConfig_t, ekfConfig, PG_EKF_CONFIG, 0);
PG_RESET_TEMPLATE(ekfConfig_t, ekfConfig, 
    .use_attitude_estimate = 0,
    .use_position_estimate = 0,
    .use_angle_measurements = { 1, 1, 1 },
    .proc_noise_acc = { 5000, 5000, 5000 },
    .proc_noise_gyro = { 1000, 1000, 1000 },
    .meas_noise_position = { 10, 10, 10 },
    .meas_noise_angles = { 100, 100, 100 },
    .meas_delay = 0,
); 

bool ekf_initialized = false;
timeUs_t lastTimeUs = 0;

float ekf_Z[N_MEASUREMENTS] = {0.};
float ekf_U[N_INPUTS] = {0.};

// EKF history buffers
#define EKF_HISTORY_SIZE 10 // 10 --> at 500Hz this is 20ms
float ekf_t_history[EKF_HISTORY_SIZE];							// time in seconds
float ekf_X_history[EKF_HISTORY_SIZE][N_STATES];				// state vector
float ekf_P_history[EKF_HISTORY_SIZE][N_STATES*(N_STATES+1)/2]; // covariance matrix
float ekf_U_history[EKF_HISTORY_SIZE][N_INPUTS];				// input vector
int ekf_history_index = 0;										// index

void ekf_add_to_history(float t) {
	ekf_history_index = (ekf_history_index + 1) % EKF_HISTORY_SIZE;		// increment index and wrap around
	// t
	ekf_t_history[ekf_history_index] = t;
	// ekf_X
	for (int i = 0; i < N_STATES; i++) {
		ekf_X_history[ekf_history_index][i] = ekf_get_X()[i];
	}
	// ekf_P
	for (int i = 0; i < N_STATES*(N_STATES+1)/2; i++) {
		ekf_P_history[ekf_history_index][i] = ekf_get_P()[i];
	}
	// ekf_U
	for (int i = 0; i < N_INPUTS; i++) {
		ekf_U_history[ekf_history_index][i] = ekf_U[i];
	}
}

// ekf update step that takes into account the time delay
void ekf_update_delayed(float Z[N_MEASUREMENTS], float t) {
	// find the first index in the history that is older than t
	int index = ekf_history_index;
	for (int i = 0; i < EKF_HISTORY_SIZE; i++) {
		index = (index - 1 + EKF_HISTORY_SIZE) % EKF_HISTORY_SIZE;
		if (ekf_t_history[index] < t) {
			break;
		}
	}
	// set the ekf state to the found state
	ekf_set_X(ekf_X_history[index]);
	ekf_set_P(ekf_P_history[index]);

	// prediction step to get to the exact time t
	ekf_predict(ekf_U_history[index], t - ekf_t_history[index]);

	// update step
	ekf_update(Z);

	// multiple prediction steps to get back to the current time
	for (int i = 0; i < EKF_HISTORY_SIZE; i++) {
		if (ekf_t_history[index+1] - ekf_t_history[index] > 0) {
			ekf_predict(ekf_U_history[index], ekf_t_history[index+1] - ekf_t_history[index]);
			index = (index + 1) % EKF_HISTORY_SIZE;
		} else {
			break;
		}
	}
}

void initEkf(timeUs_t currentTimeUs) {
	// set ekf parameters
	ekf_use_phi = ekfConfig()->use_angle_measurements[0];
	ekf_use_theta = ekfConfig()->use_angle_measurements[1];
	ekf_use_psi = ekfConfig()->use_angle_measurements[2];

	// process noise covariance
	float Q[N_STATES] = {
		((float) ekfConfig()->proc_noise_acc[0]) * 1e-4f, // ax
		((float) ekfConfig()->proc_noise_acc[1]) * 1e-4f, // ay
		((float) ekfConfig()->proc_noise_acc[2]) * 1e-4f, // az
		((float) ekfConfig()->proc_noise_gyro[0]) * 1e-4f, // p
		((float) ekfConfig()->proc_noise_gyro[1]) * 1e-4f, // q
		((float) ekfConfig()->proc_noise_gyro[2]) * 1e-4f // r
	};

	// measurement noise covariance
	float R[N_MEASUREMENTS] = {
		((float) ekfConfig()->meas_noise_position[0]) * 1e-4f, // posN
		((float) ekfConfig()->meas_noise_position[1]) * 1e-4f, // posE
		((float) ekfConfig()->meas_noise_position[2]) * 1e-4f, // posD
		((float) ekfConfig()->meas_noise_angles[0]) * 1e-4f, // phi
		((float) ekfConfig()->meas_noise_angles[1]) * 1e-4f, // theta
		((float) ekfConfig()->meas_noise_angles[2]) * 1e-4f, // psi
	};

	// sets initial state to the latest external pos and att
	float X0[N_STATES] = {
		posMeasNed.pos.V.X,
		posMeasNed.pos.V.Y,
		posMeasNed.pos.V.Z,
		0., 0., 0., // vel
		posMeasNed.att.angles.roll,
		posMeasNed.att.angles.pitch,
		posMeasNed.att.angles.yaw,
		0., 0., 0., 0., 0., 0. // acc and gyro biases
	};

	// sets initial covariance to 1
	float P_diag0[N_STATES] = {
		1., 1., 1., // pos
		1., 1., 1., // vel
		1., 1., 1., // att
		1., 1., 1., 1., 1., 1. // acc and gyro biases (to turn off bias estimation, set these to 0)!
	};

    ekf_Z[5] = posMeasNed.att.angles.yaw;
	
	// initialize ekf
	ekf_set_Q(Q);
	ekf_set_R(R);
	ekf_set_X(X0);
	ekf_set_P_diag(P_diag0);
	ekf_initialized = true;
	lastTimeUs = currentTimeUs;
}

void runEkf(timeUs_t currentTimeUs) {
    static timeUs_t lastUpdateTimestamp = 0;
	// PREDICTION STEP
    // FRD frame's, which we have now everywhere in INDIFlight
	ekf_U[0] = GRAVITYf * ((float)acc.accADCf[0]) / ((float)acc.dev.acc_1G);
	ekf_U[1] = GRAVITYf * ((float)acc.accADCf[1]) / ((float)acc.dev.acc_1G);
	ekf_U[2] = GRAVITYf * ((float)acc.accADCf[2]) / ((float)acc.dev.acc_1G);
	ekf_U[3] = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
	ekf_U[4] = DEGREES_TO_RADIANS(gyro.gyroADCf[1]);
	ekf_U[5] = DEGREES_TO_RADIANS(gyro.gyroADCf[2]);

	// add to history (will be used in the update step)
	// ekf_add_to_history(currentTimeUs * 1e-6);

	// PREDICTION STEP
	float dt = (currentTimeUs - lastTimeUs) * 1e-6;
	ekf_predict(ekf_U, dt);

	// UPDATE STEP 			(only when new measurement is available)
	if (cmpTimeUs(posLatestMsgTime, lastUpdateTimestamp) > 0) {
        lastUpdateTimestamp = posLatestMsgTime;
		ekf_Z[0] = posMeasNed.pos.V.X;
		ekf_Z[1] = posMeasNed.pos.V.Y;
		ekf_Z[2] = posMeasNed.pos.V.Z;
		ekf_Z[3] = posMeasNed.att.angles.roll;
		ekf_Z[4] = posMeasNed.att.angles.pitch;

		// fix yaw discontinuity (rad)
		float delta_psi = posMeasNed.att.angles.yaw - ekf_Z[5];
		while (delta_psi > M_PI) {
			delta_psi -= 2 * M_PI;
		}
		while (delta_psi < -M_PI) {
			delta_psi += 2 * M_PI;
		}

		ekf_Z[5] += delta_psi;

		// old update:
		ekf_update(ekf_Z);

		// new update that takes into account the time delay:
		// ekf_update_delayed(ekf_Z, extLatestMsgTime * 1e-6);
		// float delay = ((float) ekfConfig()->meas_delay) * 1e-3f; // in seconds
		// ekf_update_delayed(ekf_Z, currentTimeUs * 1e-6 - delay);
	} 
	// else {
	// 	// no new measurement available, only predict
	// 	float dt = (currentTimeUs - lastTimeUs) * 1e-6;
	// 	ekf_predict(ekf_U, dt);
	// }
	lastTimeUs = currentTimeUs;
}

void updateEkf(timeUs_t currentTimeUs) {
	// reset ekf LOCAL_POS_NO_SIGNAL
    if (posMeasState == LOCAL_POS_NO_SIGNAL) {
        ekf_initialized = false;
    } else if (!ekf_initialized) {
        // when ekf is not initialized, we need to wait for the first external position message
        if (posMeasState != LOCAL_POS_NO_SIGNAL) {
            // INIT EKF
            initEkf(currentTimeUs);
        }
    } else {
        // ekf is initialized and POSITION_MODE, we can run the ekf
		runEkf(currentTimeUs);
    }

    // run fallback in advance so it doesnt lose sync
    imuUpdateAttitude(currentTimeUs);

    // update system state with EKF data, if possible and configured
    if (ekf_initialized && (posMeasState != LOCAL_POS_NO_SIGNAL)) {
        // additional safety check: use EKF only, if recent update from optitrack
        float *ekf_X = ekf_get_X();

        if (ekfConfig()->use_attitude_estimate) {
            fp_quaternion_t q;
            fp_euler_t e = { .angles.roll = ekf_X[6], .angles.pitch = ekf_X[7], .angles.yaw = ekf_X[8] }; // rad
            quaternion_of_fp_euler(&q, &e);
            setAttitudeWithQuaternion(&q);
        }

        if (ekfConfig()->use_position_estimate) {
            fp_vector_t posNed_set;
            posNed_set.V.X = ekf_X[0];
            posNed_set.V.Y = ekf_X[1];
            posNed_set.V.Z = ekf_X[2];

            fp_vector_t velNed_set;
            velNed_set.V.X = ekf_X[3];
            velNed_set.V.Y = ekf_X[4];
            velNed_set.V.Z = ekf_X[5];

            setPositionState(posNed_set, velNed_set);
        }
    }
}

#endif // USE_EKF