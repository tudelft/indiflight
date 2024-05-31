/*
 * INDI based attitude and rate controller for UAV
 *
 * Copyright 2023, 2024 Till Blaha (Delft University of Technology)
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


#pragma once

#include <stdbool.h>

#include "platform.h"
#include "flight/pid.h"
#include "solveActiveSet.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config.h"

#define MAXU MAX_SUPPORTED_MOTORS
#define MAX_SUPPORTED_PSEUDOCONTROLS 6
#define MAXV MAX_SUPPORTED_PSEUDOCONTROLS

#if (MAXU > 8)
#error "Blackbox will crash. Fix it to accept more that MAXU > 8"
#endif

#if (MAXV > 8)
#error "Blackbox will crash. Fix it to accept more that MAXV > 8"
#endif

typedef struct indiProfile_s {
    // ---- Att/Rate config
    uint16_t attGains[3]; // attitude error to rotational accel gain * 10
    uint16_t rateGains[3]; // rate error to rotational accel gain * 10
    uint16_t attMaxTiltRate; // max tilt rate in deg/s, if attitude is controlled
    uint16_t attMaxYawRate; // max yaw rate in deg/s, if attitude is controlled
    uint16_t maxRateSp[3];          // maximum rate setpoint in deg/s
    uint8_t manualUseCoordinatedYaw;     // bool: coordinate yaw in manual flight in Angle/Horizon mode
    uint8_t manualMaxUpwardsSpf; // maximum upwards specific force in manual flight in N/kg. 255 means max of the platform
    uint8_t manualMaxTilt; // in manual flight in deg (0, 180)
    // ---- general INDI config
    uint8_t useIncrement;          // bool: use incremental law. NDI (or more precisely linDI) otherwise
    uint8_t useRpmDotFeedback;     // bool: make use of dshot rpm derivative data in feedback loop if available
    // ---- INDI actuator config
    uint8_t actNum;                 // number of actuators
    uint8_t actTimeConstMs[MAXU];     // time constant for actuator spool up in ms
    /*
    uint32_t actPropConst[MAXU];    // propeller constant in N / (rad/s)^2 * 1e11
    uint16_t actMaxT[MAXU];         // max motor thrust in N*100 (centinewton)
    */
    uint32_t actMaxRpm[MAXU];       // max rpm.
    uint32_t actHoverRpm[MAXU];     // approximate rpm in hover flight. FIXME: make an estimator for this
    uint8_t actNonlinearity[MAXU];  // motor nonlinearity percentage between (0, 100)
    uint8_t actLimit[MAXU];         // limit motor output in percent (100 full power)
    int16_t actG1_fx[MAXU];      // actuator effectiveness (N/kg) / u * 100
    int16_t actG1_fy[MAXU];      // actuator effectiveness * 100
    int16_t actG1_fz[MAXU];      // actuator effectiveness * 100
    int16_t actG1_roll[MAXU];    // actuator effectiveness (Nm/(kg m^2)) / u * 10
    int16_t actG1_pitch[MAXU];   // actuator effectiveness * 10 
    int16_t actG1_yaw[MAXU];     // actuator effectiveness * 10
    int16_t actG2_roll[MAXU];    // actuator rate effectiveness (Nm/(kg m^2)) / (rad/s/s) * hoverRpm * 10 TODO: change scaling
    int16_t actG2_pitch[MAXU];   // actuator rate effectiveness
    int16_t actG2_yaw[MAXU];     // actuator rate effectiveness
    // ---- Filtering config
    uint8_t imuSyncLp2Hz;        // 2nd order butterworth break frequency in Hz for imu synchoronous filtering
    // ---- WLS config
    uint8_t wlsWv[MAXV];         // control objective weighing (1, 100)
    uint8_t wlsWu[MAXU];         // actuator penalties (1, 100)
    int8_t u_pref[MAXU];         // least energy consumption u * 100

    // -------- inaccessible parameters for now (will always be the values from the reset function)
    // ---- Att/Rate config
    uint8_t attRateDenom;        // only execute attitude loop every attRateDenom loops
    // ---- general INDI config
    uint8_t useConstantG2;         // bool: do not adapt spinup terms based on rpm data, if available and useWls is true
    uint8_t useRpmFeedback;        // bool: make use of dshot rpm data in feedback loop if available. FIXME: actually implement this
    // ---- WLS config
    uint8_t useWls;              // bool: enable Wls in favour of static pinv
    uint8_t wlsWarmstart;        // bool: use warmstarting of wls
    uint8_t wlsMaxIter;          // wls iteration limit per loop. Keep to 1 usually, if warmstarting is used
    uint8_t wlsAlgo;             // 0: QR, but slow. 1: QR, 2: Chol
    uint16_t wlsCondBound;       // condition number bound / 1e4
    uint16_t wlsTheta;           // objective segragation / 1e-4
    uint8_t wlsNanLimit;        // disarm because of consequtive failures in wls. Keep low FIXME: make this fallback to pinv
} indiProfile_t;

// linearization
typedef struct actLin_s {
    float A;
    float B;
    float C;
    float k;
} actLin_t;

typedef struct indiRuntime_s {
    // ---- Att/Rate config
    fp_vector_t attGains;
    fp_vector_t rateGains;
    float attMaxTiltRate; // rad/s
    float attMaxYawRate;  // rad/s
    uint8_t attRateDenom;
    bool manualUseCoordinatedYaw;     // bool: coordinate yaw in manual flight in Angle/Horizon mode
    float manualMaxUpwardsSpf; // 255 means sum(G1[2, :])
    float manualMaxTilt; // rad
    // ---- general INDI config
    bool useIncrement;
    bool useConstantG2;
    bool useRpmFeedback;
    bool useRpmDotFeedback;
    fp_vector_t maxRateSp;          // maximum rate setpoint in deg/s
    // ---- INDI actuator config
    uint8_t actNum;
    /*
    float actPropConst[MAXU];    // propeller constant in N / (rad/s)^2 * 1e11
    float actMaxT[MAXU];         // N
    */
    float actMaxOmega[MAXU];    // rad/s
    float actMaxOmega2[MAXU];    // rad/s
    float actHoverOmega[MAXU];   // rad/s
    float actTimeConstS[MAXU];   // sec
    float actNonlinearity[MAXU]; // - 
    float actLimit[MAXU];        // 
    float actG1[MAXV][MAXU];
    float actG2[3][MAXU];
    float G2_scaler[MAXU];
    // ---- Filtering config
    float imuSyncLp2Hz;        // 2nd order butterworth break frequency in Hz for imu synchoronous filtering
    // ---- WLS config
    float wlsWv[MAXV];
    float wlsWu[MAXU];
    float u_pref[MAXU];
    activeSetAlgoChoice wlsAlgo;
    bool useWls;
    bool wlsWarmstart;
    uint8_t wlsMaxIter;
    float wlsCondBound;
    float wlsTheta;
    uint8_t wlsNanLimit;
    // ---- runtime values -- actauators
    float d[MAXU]; // command issued to the actuators on [-1, 1] scale
    float u[MAXU]; // control variable proportional to output force, but on [-1, 1], for motors [0, 1]
    float uState[MAXU]; // estimated force state of the actuators [-1, 1]
    float uState_fs[MAXU]; // sync-filtered estiamted force state
    actLin_t lin[MAXU]; // linearization. u = indiOutputCurve(lin, d) and d = indiLinearization(lin, u)
    float omega[MAXU]; // unfiltered motor speed rad/s
    float omega_fs[MAXU]; // sync-filtered motor speed rad/s
    //float omegaDot[MAXU]; // unfiltered motor rate rad/s/s
    float omegaDot_fs[MAXU]; // sync-filtered motor rate rad/s/s
    float erpmToRads; // factor to move from motor erpm to rad/s
    // ---- runtime values -- axes
    fp_vector_t attGainsCasc; // attitude gains simulating parallel PD
    fp_quaternion_t attSpNed; // attitude setpoint in NED coordinates
    fp_quaternion_t attErrBody; // attitude error in body coordinates
    fp_vector_t rateSpBody; // rate setpoint in body coordinates
    fp_vector_t rateSpBodyCommanded; // rate setpoint before attitude control
    fp_vector_t rateDotSpBody; // rate derivative setpoint in body coordinates
    fp_vector_t spfSpBody; // specific force setpoint in body coordinates
    float dv[MAXV]; // delta-pseudo controls in N/kg and Nm/(kgm^2)
    //fp_vector_t rate_fs; // sync-filtered gyro in rad/s
    fp_vector_t rateIMU; // unfiltered gyro in rad/s
    fp_vector_t rate_f; // unfiltered gyro in rad/s
    fp_vector_t rateDotIMU; // unfiltered gyro derivative in rad/s/s
    fp_vector_t rateDot_fs; // sync-filtered gyro derivative in rad/s/s
    fp_vector_t spfIMU; // unfiltered accelerometer (specific force) in N/kg
    fp_vector_t spf_fs; // sync-filterd accelerometer (specific force) in N/kg
    // ---- filters
    pt1Filter_t uLagFilter[MAXU]; // to simulate spinup
    biquadFilter_t uStateFilter[MAXU]; // only support 2nd order butterworth second order section for now
    biquadFilter_t omegaFilter[MAXU]; // only support 2nd order butterworth second order section for now
    biquadFilter_t rateFilter[3]; // only support 2nd order butterworth second order section for now
    biquadFilter_t spfFilter[3]; // only support 2nd order butterworth second order section for now
    // ---- housekeeping
    float dT; // target looptime in Sec
    float indiFrequency; // frequency in Hz
    uint8_t attExecCounter; // count executions (wrapping)
    uint16_t nanCounter; // count times consequtive nans appear in allocation
    // ---- control law selection
    bool bypassControl; // no control at all. u and d are unmodified by loop
    bool controlAttitude; // attempt to reach tilt given by attSpNed
    bool trackAttitudeYaw; // also attempt to reach yaw given by attSpNed
} indiRuntime_t;

#define INDI_PROFILE_COUNT 3
PG_DECLARE_ARRAY(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles);

extern indiRuntime_t indiRun;

void indiController(timeUs_t current);
void updateLinearization(actLin_t* lin, float k);
float indiLinearization(actLin_t* lin, float in);
float indiOutputCurve(actLin_t* lin, float in);

float getYawWithoutSingularity(void);
fp_vector_t coordinatedYaw(float yaw);
void getSetpoints(timeUs_t current);
void getAlphaSpBody(timeUs_t current);
void getMotorCommands(timeUs_t current);
void indiUpdateActuatorState( float* motor_normalized );
