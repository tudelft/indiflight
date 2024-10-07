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

#include "io/external_pos.h"  		// for extPosNed
#include "fc/runtime_config.h"		// for FLIGHT_MODE
#include "common/maths.h"      		// for DEGREES_TO_RADIANS
#include "sensors/gyro.h"			// for gyro
#include "sensors/acceleration.h"   // for acc
#include "imu.h"                    // for fallback if no GPS
#include "pi-messages.h"            // for keeping track of message times

#include <stdbool.h>

#include "pg/pg_ids.h"              // for config

#ifdef USE_EKF

#ifndef USE_ACC
#error "USE_EKF requires USE_ACC"
#endif

#ifndef USE_GYRO
#error "USE_EKF requires USE_GYRO"
#endif

#ifndef USE_GPS_PI
#error "USE_EKF requires USE_GPS_PI"
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(ekfConfig_t, ekfConfig, PG_EKF_CONFIG, 0);
PG_RESET_TEMPLATE(ekfConfig_t, ekfConfig, 
    .use_attitude_estimate = 0,
    .use_position_estimate = 0,
    .use_quat_measurement  = 1,
    .proc_noise_acc        = { 500000, 500000, 500000 }, // micro-covariances
    .proc_noise_gyro       = { 100000, 100000, 100000 },
    .proc_noise_acc_bias   = { 100, 100, 100 },
    .proc_noise_gyro_bias  = { 10, 10, 10 },
    .meas_noise_position   = { 1000, 1000, 1000 },
    .meas_noise_quat       = { 50000, 50000, 50000, 50000 },
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
	//ekf_set_P(ekf_P_history[index]); // apparently, P_diag is the only one now?

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

extern bool ekf_use_quat;

bool isInitializedEkf(void) {
    return ekf_initialized;
}

void initEkf(timeUs_t currentTimeUs) {
    if (extPosState == EXT_POS_NO_SIGNAL) {
        return;
    }

	// set ekf parameters
	ekf_use_quat = ekfConfig()->use_quat_measurement;

	// process noise covariance
	float Q[N_STATES] = {
		((float) ekfConfig()->proc_noise_acc[0]) * 1e-6f, // ax
		((float) ekfConfig()->proc_noise_acc[1]) * 1e-6f, // ay
		((float) ekfConfig()->proc_noise_acc[2]) * 1e-6f, // az
		((float) ekfConfig()->proc_noise_gyro[0]) * 1e-6f, // p
		((float) ekfConfig()->proc_noise_gyro[1]) * 1e-6f, // q
		((float) ekfConfig()->proc_noise_gyro[2]) * 1e-6f, // r
		((float) ekfConfig()->proc_noise_acc_bias[0]) * 1e-6f, // ax
		((float) ekfConfig()->proc_noise_acc_bias[1]) * 1e-6f, // ay
		((float) ekfConfig()->proc_noise_acc_bias[2]) * 1e-6f, // az 
		((float) ekfConfig()->proc_noise_gyro_bias[0]) * 1e-6f, // p
		((float) ekfConfig()->proc_noise_gyro_bias[1]) * 1e-6f, // q
		((float) ekfConfig()->proc_noise_gyro_bias[2]) * 1e-6f // r
	};

	// measurement noise covariance
	float R[N_MEASUREMENTS] = {
		((float) ekfConfig()->meas_noise_position[0]) * 1e-6f, // posN
		((float) ekfConfig()->meas_noise_position[1]) * 1e-6f, // posE
		((float) ekfConfig()->meas_noise_position[2]) * 1e-6f, // posD
		((float) ekfConfig()->meas_noise_quat[0]) * 1e-6f, // qw
		((float) ekfConfig()->meas_noise_quat[1]) * 1e-6f, // qx
		((float) ekfConfig()->meas_noise_quat[2]) * 1e-6f, // qy
		((float) ekfConfig()->meas_noise_quat[3]) * 1e-6f // qz
	};

	// sets initial state to the latest external pos and att
	float X0[N_STATES] = {
		extPosNed.pos.V.X,
		extPosNed.pos.V.Y,
		extPosNed.pos.V.Z,
		0., 0., 0., // vel
        1., 0., 0., 0., // quaternion
		0., 0., 0., 0., 0., 0. // acc and gyro biases
	};

	// sets initial covariance to 1
	float P_diag0[N_STATES] = {
		1., 1., 1., // pos
		1., 1., 1., // vel
		1., 1., 1., 1., // quat
		1e-2f, 1e-2f, 1e-2f, 1e-2f, 1e-2f, 1e-2f // acc and gyro biases (to turn off bias estimation, set these to 0)!
	};

    if (ekf_use_quat) {
		X0[6] = extPosNed.quat.w;
		X0[7] = extPosNed.quat.x;
		X0[8] = extPosNed.quat.y;
		X0[9] = extPosNed.quat.z;
    } else {
        P_diag0[13] = 0.f; // turn off gyro bias estimation
        P_diag0[14] = 0.f; // turn off gyro bias estimation
        P_diag0[15] = 0.f; // turn off gyro bias estimation
        Q[13] = 0.f;
        Q[14] = 0.f;
        Q[15] = 0.f;
    }

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
	if (cmpTimeUs(extLatestMsgTime, lastUpdateTimestamp) > 0) {
        lastUpdateTimestamp = extLatestMsgTime;
		ekf_Z[0] = extPosNed.pos.V.X;
		ekf_Z[1] = extPosNed.pos.V.Y;
		ekf_Z[2] = extPosNed.pos.V.Z;

		ekf_Z[3] = (ekf_use_quat) * extPosNed.quat.w;
		ekf_Z[4] = (ekf_use_quat) * extPosNed.quat.x;
		ekf_Z[5] = (ekf_use_quat) * extPosNed.quat.y;
		ekf_Z[6] = (ekf_use_quat) * extPosNed.quat.z;

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
	// reset ekf EXT_POS_NO_SIGNAL
    if (extPosState == EXT_POS_NO_SIGNAL) {
        ekf_initialized = false;
    } else if (!ekf_initialized) {
        // NEVER EVER auto-reinitialize EKF. bad things will happen. use a switch or the keyboard
/*
        // when ekf is not initialized, we need to wait for the first external position message
        if (extPosState != EXT_POS_NO_SIGNAL) {
            // INIT EKF
            if (ekfConfig()->use_quat_measurement) {
                // auto re-init only when we have an absolute source of heading?
                initEkf(currentTimeUs);
            }
        }
*/
    } else {
        // ekf is initialized and POSITION_MODE, we can run the ekf
		runEkf(currentTimeUs);
    }

    // run fallback in advance so it doesnt lose sync
    imuUpdateAttitude(currentTimeUs);

    // update system state with EKF data, if possible and configured
    if (ekf_initialized && (extPosState != EXT_POS_NO_SIGNAL)) {
        // additional safety check: use EKF only, if recent update from optitrack
        float *ekf_X = ekf_get_X();

        if (ekfConfig()->use_attitude_estimate) {
            fp_quaternion_t q = { .w = ekf_X[6], .x = ekf_X[7], .y = ekf_X[8], .z = ekf_X[9], };
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