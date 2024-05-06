#include "ekf.h"

#include "io/external_pos.h"  		// for extPosNed
#include "fc/runtime_config.h"		// for FLIGHT_MODE
#include "common/maths.h"      		// for DEGREES_TO_RADIANS
#include "sensors/gyro.h"			// for gyro
#include "sensors/acceleration.h"   // for acc
#include "imu.h"                    // for fallback if no GPS
#include "pi-messages.h"            // for keeping track of message times

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
	ekf_use_phi = 1;
	ekf_use_theta = 1;
	ekf_use_psi = 1;

	// process noise covariance
	float Q[N_STATES] = {
		0.5, // ax
		0.5, // ay
		0.5, // az
		0.1, // p
		0.1, // q
		0.1, // r
	};

	// measurement noise covariance
	float R[N_MEASUREMENTS] = {
		0.001, // posN
		0.001, // posE
		0.001, // posD
		0.01,  // phi
		0.01,  // theta
		0.01,  // psi
	};


	// sets initial state to the latest external pos and att
	float X0[N_STATES] = {
		extPosNed.pos.V.X,
		extPosNed.pos.V.Y,
		extPosNed.pos.V.Z,
		0., 0., 0., // vel
		extPosNed.att.angles.roll,
		extPosNed.att.angles.pitch,
		extPosNed.att.angles.yaw,
		0., 0., 0., 0., 0., 0. // acc and gyro biases
	};

	// sets initial covariance to 1
	float P_diag0[N_STATES] = {
		1., 1., 1., // pos
		1., 1., 1., // vel
		1., 1., 1., // att
		1., 1., 1., 1., 1., 1. // acc and gyro biases (to turn off bias estimation, set these to 0)!
	};

    ekf_Z[5] = extPosNed.att.angles.yaw;
	
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

	// GET INPUTS
	// gyro and acc transformed from FLU to FRD
	ekf_U[0] = 9.81 * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G);
	ekf_U[1] = 9.81 *-((float)acc.accADC[1]) / ((float)acc.dev.acc_1G);
	ekf_U[2] = 9.81 *-((float)acc.accADC[2]) / ((float)acc.dev.acc_1G);
	ekf_U[3] = DEGREES_TO_RADIANS(gyro.gyroADCf[0]);
	ekf_U[4] = DEGREES_TO_RADIANS(-gyro.gyroADCf[1]);
	ekf_U[5] = DEGREES_TO_RADIANS(-gyro.gyroADCf[2]);

	// add to history (will be used in the update step)
	ekf_add_to_history(currentTimeUs * 1e-6);

	// PREDICTION STEP
	// float dt = (currentTimeUs - lastTimeUs) * 1e-6;
	// ekf_predict(ekf_U, dt);

	// UPDATE STEP 			(only when new measurement is available)
	if (cmpTimeUs(extLatestMsgTime, lastUpdateTimestamp) > 0) {
        lastUpdateTimestamp = extLatestMsgTime;
		ekf_Z[0] = extPosNed.pos.V.X;
		ekf_Z[1] = extPosNed.pos.V.Y;
		ekf_Z[2] = extPosNed.pos.V.Z;
		ekf_Z[3] = extPosNed.att.angles.roll;
		ekf_Z[4] = extPosNed.att.angles.pitch;

		// fix yaw discontinuity (rad)
		float delta_psi = extPosNed.att.angles.yaw - ekf_Z[5];
		while (delta_psi > M_PI) {
			delta_psi -= 2 * M_PI;
		}
		while (delta_psi < -M_PI) {
			delta_psi += 2 * M_PI;
		}

		ekf_Z[5] += delta_psi;

		// old update:
		// ekf_update(ekf_Z);

		// new update that takes into account the time delay:
		// ekf_update_delayed(ekf_Z, extLatestMsgTime * 1e-6);
		float delay = 0.008;
		ekf_update_delayed(ekf_Z, currentTimeUs * 1e-6 - delay);
	} 
	else {
		// no new measurement available, only predict
		float dt = (currentTimeUs - lastTimeUs) * 1e-6;
		ekf_predict(ekf_U, dt);
	}
	lastTimeUs = currentTimeUs;
}

void updateEkf(timeUs_t currentTimeUs) {
	// reset ekf EXT_POS_NO_SIGNAL
    if (extPosState == EXT_POS_NO_SIGNAL) {
        ekf_initialized = false;
    } else if (!ekf_initialized) {
        // when ekf is not initialized, we need to wait for the first external position message
        if (extPosState != EXT_POS_NO_SIGNAL) {
            // INIT EKF
            initEkf(currentTimeUs);
        }
    } else {
        // ekf is initialized and POSITION_MODE, we can run the ekf
		runEkf(currentTimeUs);
    }

    // run fallback in advance so it doesnt lose sync
    imuUpdateAttitude(currentTimeUs);

#if defined(USE_EKF_ATTITUDE) || defined(USE_EKF_POSITION)
    // update system state with EKF data, if possible
    if (ekf_initialized && (extPosState != EXT_POS_NO_SIGNAL)) {
        // additional safety check: use EKF only, if recent update from optitrack
        float *ekf_X = ekf_get_X();

#ifdef USE_EKF_ATTITUDE
        setAttitudeState(ekf_X[6], -ekf_X[7], ekf_X[8]); // roll pitch yaw in rad
#endif
#ifdef USE_EKF_POSITION
        t_fp_vector posNed_set;
        posNed_set.V.X = ekf_X[0];
        posNed_set.V.Y = ekf_X[1];
        posNed_set.V.Z = ekf_X[2];

        t_fp_vector velNed_set;
        velNed_set.V.X = ekf_X[3];
        velNed_set.V.Y = ekf_X[4];
        velNed_set.V.Z = ekf_X[5];

        setPositionState(posNed_set, velNed_set);
#endif
    }
#endif // defined(USE_EKF_ATTITUDE) || defined(USE_EKF_POSITION)
}

#endif // USE_EKF