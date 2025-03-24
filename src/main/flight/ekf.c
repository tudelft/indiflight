/*
 * 
 *
 * Copyright 2023 Robin Ferede (Delft University of Technology)
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *     Improved integration with legacy estimator, added parameters
 * Copyright 2025 Till Blaha (Delft University of Technology)
 *     Upstream quaternion EKF, remove legacy estimator
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

#include "io/local_pos.h"  		    // for posMeasNed
#include "fc/runtime_config.h"		// for FLIGHT_MODE
#include "common/maths.h"      		// for DEGREES_TO_RADIANS
#include "sensors/gyro.h"			// for gyro
#include "sensors/acceleration.h"   // for acc
#include "ahrs.h"                   // for fallback if no GPS
#include "telemetry/pi.h"
#include "pi-messages.h"            // for keeping track of message times
#include "sensors/barometer.h"
#include "flight/indi.h"
#include "flight/throw.h"
#include "drivers/dshot.h"
#include <stdbool.h>

#include "pg/pg_ids.h"              // for config

#ifdef USE_EKF

PG_REGISTER_WITH_RESET_TEMPLATE(ekfConfig_t, ekfConfig, PG_EKF_CONFIG, 2);
PG_RESET_TEMPLATE(ekfConfig_t, ekfConfig, 
    .use_quat_measurement = 1,
    .use_for_manual_flight = 0,
    .meas_source = LOCAL_POS_SOURCE_PI,
    .global_home_lat = 519906500,
    .global_home_lon =  43766250,
    .proc_noise_acc       = { 500000, 500000, 500000 },
    .proc_noise_gyro      = { 100000, 100000, 100000 },
    .proc_noise_acc_bias  = { 100, 100, 100 },
    .proc_noise_gyro_bias = { 10, 10, 10 },
    .meas_noise_position  = { 1000, 1000, 1000 },
    .meas_noise_quat      = { 50000, 50000, 50000, 50000 },
    .meas_delay = 0,
); 

fp_quaternion_t qEkf = QUATERNION_INITIALIZE;
fp_vector_t posEstNed = {0};
fp_vector_t velEstNed = {0};

bool ekf_initialized = false;
bool ekf_converged = false;
bool ekf_should_use = false;
timeUs_t lastInitializedTimeUs = 0;
timeUs_t lastPredictTimeUs = 0;

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

bool isConvergedEkf(void) {
    return ekf_converged;
}

bool shouldBeUsedEkf(void) {
    return ekf_should_use;
}

void forceDeinitEkf(void) {
    ekf_initialized = false;
    ekf_converged = false;
}


void initEkf(timeUs_t currentTimeUs) {
    if ( !(posMeasNed.new)
            || (cmpTimeUs(currentTimeUs, posMeasNed.time_us) > EKF_MAX_MEAS_AGE_US) ) {
        return;
    }
    posMeasNed.new = false;

	// set ekf parameters
	bool use_quat = ekfConfig()->use_quat_measurement;

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
		posMeasNed.pos.V.X,
		posMeasNed.pos.V.Y,
		posMeasNed.pos.V.Z,
		0., 0., 0., // vel
        1., 0., 0., 0., // quaternion
		0., 0., 0., 0., 0., 0. // acc and gyro biases
	};

	// sets initial covariance to 1
	float P_diag0[N_STATES] = {
		1., 1., 1.,     // pos
		1., 1., 1.,     // vel
		1., 1., 1., 1., // att
		1e-2f, 1e-2f, 1e-2f, // acc biases (to turn off bias estimation, set these to 0)!
        1e-2f, 1e-2f, 1e-2f  // gyro biases (to turn off bias estimation, set these to 0)!
	};

    if (use_quat) {
		X0[6] = posMeasNed.quat.w;
		X0[7] = posMeasNed.quat.x;
		X0[8] = posMeasNed.quat.y;
		X0[9] = posMeasNed.quat.z;
    } else {
        P_diag0[13] = 0.f; // turn off gyro bias estimation
        P_diag0[14] = 0.f; // turn off gyro bias estimation
        P_diag0[15] = 0.f; // turn off gyro bias estimation
        Q[13] = 0.f;
        Q[14] = 0.f;
        Q[15] = 0.f;
    }

#if defined(USE_BARO) && false  // todo: figure this out properly
    // IMAV hack: this should be a parameter, not a macro, or even better, some decent fusion.
    if (sensors(SENSOR_BARO)) {
        float mean = 0.f;
#if (defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY))
        if (isDshotTelemetryActive()) { 
            for (int i=0; i<indiRun.actNum; i++) {
                mean += indiRun.omega_fs[i] / indiRun.actNum;
            }
        }
#endif
        //X0[2] = -(0.01f*baro.altitude - 0.24f*sq(1e-3f*mean)); // calibrate sensor for prop speeds. doesnt take into account ground effect
        X0[2] = posMeasNed.pos.V.Z; //todo testingggg
    }
#endif

	// initialize ekf
    ekf_set_use_quat(use_quat);
	ekf_set_Q(Q);
	ekf_set_R(R);
	ekf_set_X(X0);
	ekf_set_P_diag(P_diag0);
	ekf_initialized = true;
    lastInitializedTimeUs = currentTimeUs;
	lastPredictTimeUs = currentTimeUs;
}

void updateEkf(timeUs_t currentTimeUs) {
    // --- check init and convergence
    // if not init, try init and exit
    if (!ekf_initialized) {
        ekf_converged = false;
        initEkf(currentTimeUs);
        return;
    }

    // set ekf converged if healthy for long enough
    if (!ekf_converged && cmpTimeUs(currentTimeUs, lastInitializedTimeUs) > EKF_CONVERGE_TIME_US) {
        ekf_converged = true;
    }

    // --- check if ekf will be used by control/ahrs
    ekf_should_use = ( FLIGHT_MODE(POSITION_MODE) 
                            || FLIGHT_MODE(VELOCITY_MODE)
                            || FLIGHT_MODE(NN_MODE)
                            || FLIGHT_MODE(CATAPULT_MODE) );
    ekf_should_use |= ekfConfig()->use_for_manual_flight;

    // --- deinit because of lost position, if we are not flying/expecting to fly soon
    bool ekf_inhibit_deinit = ekf_should_use && ARMING_FLAG(ARMED);
    timeDelta_t deinit_timeout = EKF_DEINIT_TIMEOUT;

#ifdef USE_THROW_TO_ARM
    // if expected to be thrown at any moment, increase deinit timeout
    if (throwState >= THROW_STATE_WAITING_FOR_THROW) {
        deinit_timeout = MAX(deinit_timeout, THROW_TO_ARM_EKF_DEINIT_TIME_US);
    }
#endif

    // deinit if lost position signal
    if ( !ekf_inhibit_deinit && cmpTimeUs(currentTimeUs, posMeasNed.time_us) > deinit_timeout ) {
        ekf_initialized = false;
        ekf_converged = false;
        return;
    }


    // --- actually run ekf ---

	// PREDICTION STEP
    // FRD frame's, which we have now everywhere in INDIFlight
	ekf_U[0] = GRAVITYf * ((float) acc.dev.acc_1G_rec) * acc.accADCafterRpm[0];
	ekf_U[1] = GRAVITYf * ((float) acc.dev.acc_1G_rec) * acc.accADCafterRpm[1];
	ekf_U[2] = GRAVITYf * ((float) acc.dev.acc_1G_rec) * acc.accADCafterRpm[2];
	ekf_U[3] = DEGREES_TO_RADIANS( gyro.gyroADCafterRpm[0] );
	ekf_U[4] = DEGREES_TO_RADIANS( gyro.gyroADCafterRpm[1] );
	ekf_U[5] = DEGREES_TO_RADIANS( gyro.gyroADCafterRpm[2] );

	// add to history (will be used in the update step)
	// ekf_add_to_history(currentTimeUs * 1e-6);

	// PREDICTION STEP
	float dt = cmpTimeUs(currentTimeUs, lastPredictTimeUs) * 1e-6f;
	ekf_predict(ekf_U, dt);
	lastPredictTimeUs = currentTimeUs;

	// UPDATE STEP 			(only when new measurement is available)
	if ( posMeasNed.new 
            && cmpTimeUs(posMeasNed.time_us, currentTimeUs) <= EKF_MAX_MEAS_AGE_US) {

        posMeasNed.new = false;

		ekf_Z[0] = posMeasNed.pos.V.X;
		ekf_Z[1] = posMeasNed.pos.V.Y;
#if defined(USE_BARO) && false // figure this out properly
        if (sensors(SENSOR_BARO)) {
            // IMAV hack: this should be a parameter, not a macro, or even better, some decent fusion
            float mean = 0.f;
#if (defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY))
            if (isDshotTelemetryActive()) { 
                for (int i=0; i<indiRun.actNum; i++) {
                    mean += indiRun.omega_fs[i] / indiRun.actNum;
                }
            }
#endif
            float baroCalib = -(0.01f*baro.altitude - 0.24f*sq(1e-3f*mean)); // calibrate sensor for prop speeds. doesnt take into account ground effect
            DEBUG_SET(DEBUG_BARO, 3, lrintf(-100.f*baroCalib));
            ekf_Z[2] = posMeasNed.pos.V.Z; //todo testingggg
            //ekf_Z[2] = baroCalib; //todo testingggg
        } else
#endif
        {// GPS
		    ekf_Z[2] = posMeasNed.pos.V.Z;
        }

        ekf_Z[3] = (ekfConfig()->use_quat_measurement) * posMeasNed.quat.w;
		ekf_Z[4] = (ekfConfig()->use_quat_measurement) * posMeasNed.quat.x;
		ekf_Z[5] = (ekfConfig()->use_quat_measurement) * posMeasNed.quat.y;
		ekf_Z[6] = (ekfConfig()->use_quat_measurement) * posMeasNed.quat.z;

		// old update:
		ekf_update(ekf_Z);

		// new update that takes into account the time delay:
		// ekf_update_delayed(ekf_Z, posLatestMsgTime * 1e-6);
		// float delay = ((float) ekfConfig()->meas_delay) * 1e-3f; // in seconds
		// ekf_update_delayed(ekf_Z, currentTimeUs * 1e-6 - delay);
	}

    float *ekf_X = ekf_get_X();

    // update position
    posEstNed = (fp_vector_t) { .V = {ekf_X[0], ekf_X[1], ekf_X[2]} };
    velEstNed = (fp_vector_t) { .V = {ekf_X[3], ekf_X[4], ekf_X[5]} };

    // set ekf quaternion
    qEkf = (fp_quaternion_t) { .w = ekf_X[6], .x = ekf_X[7], .y = ekf_X[8], .z = ekf_X[9] };
}

#endif // USE_EKF