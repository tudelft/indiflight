#include "ekf.h"

#include "io/external_pos.h"  		// for extPosNed
#include "fc/runtime_config.h"		// for FLIGHT_MODE
#include "common/maths.h"      		// for DEGREES_TO_RADIANS
#include "sensors/gyro.h"			// for gyro
#include "sensors/acceleration.h"   // for acc

bool ekf_initialized = false;
timeUs_t lastTimeUs = 0;
float ekf_Z[N_MEASUREMENTS] = {0.};

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
		0.01, // posN
		0.01, // posE
		0.01, // posD
		0.1,  // phi
		0.1,  // theta
		0.1,  // psi
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
		1., 1., 1., 1., 1., 1. // acc and gyro biases (to turn off bias estimation, set these to 0)
	};
	
	// initialize ekf
	ekf_set_Q(Q);
	ekf_set_R(R);
	ekf_set_X(X0);
	ekf_set_P_diag(P_diag0);
	ekf_initialized = true;
	lastTimeUs = currentTimeUs;
}

void runEkf(timeUs_t currentTimeUs) {
	// PREDICTION STEP
	// gyro and acc transformed from FLU to FRD
	float U[N_INPUTS] = {
		9.81 * ((float)acc.accADC[0]) / ((float)acc.dev.acc_1G),
		9.81 *-((float)acc.accADC[1]) / ((float)acc.dev.acc_1G),
		9.81 *-((float)acc.accADC[2]) / ((float)acc.dev.acc_1G),
		DEGREES_TO_RADIANS(gyro.gyroADCf[0]), // TODO: figure out if we need gyroADCf or gyroADC
		DEGREES_TO_RADIANS(-gyro.gyroADCf[1]),
		DEGREES_TO_RADIANS(-gyro.gyroADCf[2])
	};

	// get delta t
	float dt = (currentTimeUs - lastTimeUs) * 1e-6;

	ekf_predict(U, dt);

	// UPDATE STEP
	if (ekf_Z[0] != extPosNed.pos.V.X) { // only update if new data is available [TODO: make this more elegant]
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

		ekf_update(ekf_Z);
	}
	lastTimeUs = currentTimeUs;
}

void updateEkf(timeUs_t currentTimeUs) {
	// reset ekf EXT_POS_NO_SIGNAL
    if (extPosState == EXT_POS_NO_SIGNAL) {
        ekf_initialized = false;
        return;
    }

    // when ekf is not initialized, we need to wait for the first external position message
    if (!ekf_initialized) {
        if (extPosState != EXT_POS_NO_SIGNAL) {
            // INIT EKF
            initEkf(currentTimeUs);
        }
    } else {
        // ekf is initialized, we can run the ekf
		runEkf(currentTimeUs);
    }
}