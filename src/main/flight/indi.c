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


#include <string.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_INDI

#include "common/maths.h"
#include "common/axis.h"
#include "common/filter.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/mixer_init.h"
#include "flight/pos_ctl.h"
#include "flight/catapult.h"
#include "flight/rpm_filter.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include <math.h>
#include "config/config.h"
#include "setupWLS.h"
#include "solveActiveSet.h"
#include "flight/learner.h"

#include "io/local_pos.h"
#include "io/beeper.h"

#include "pg/pg.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "indi_init.h"
#include "indi.h"


#ifndef USE_ACC
#error "Must use accelerometer (USE_ACC) to USE_INDI"
#endif


PG_REGISTER_ARRAY_WITH_RESET_FN(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles, PG_INDI_PROFILE, 0);

FAST_DATA_ZERO_INIT indiRuntime_t indiRun;

#define RC_SCALE_THROTTLE 0.001f
#define RC_OFFSET_THROTTLE 1000.f

// refurbish this code somehow
#if (MAXU > AS_N_U) || (MAXV > AS_N_V)
#error "Sizes may be too much for ActiveSetCtlAlloc library"
#endif

#ifdef STM32H7
FAST_CODE
#endif
void indiController(timeUs_t current) {
    if (flightModeFlags & ~(CATAPULT_MODE | LEARNER_MODE)) {
        // any flight mode active other than catapult, learner or acro (acro is all off)?
        if ( ((++indiRun.attExecCounter)%indiRun.attRateDenom) == 1 ) {
            // rate limit attitude control
            getSetpoints(current);
            indiRun.attExecCounter = 1;
        }
    } else {
        // function is cheap, let's do it an all iterations
        getSetpoints(current);
    }

    // for any flight mode:
    // 1. compute desired angular accelerations
    getAlphaSpBody(current);

    // allocation and INDI
    getMotorCommands(current);

#ifdef USE_LEARNER
    // update learner.
    // FIXME: delay setpoints used by learner by one sample to be in sync
    updateLearner(current);
#endif
}

#ifdef STM32H7
FAST_CODE
#endif
void getSetpoints(timeUs_t current) {
#if !defined(USE_CATAPULT) && !defined(USE_LEARNER)
    UNUSED(current);
#endif

    indiRun.attSpNed.w = 1.f;
    indiRun.attSpNed.x = 0.f;
    indiRun.attSpNed.y = 0.f;
    indiRun.attSpNed.z = 0.f;

    indiRun.spfSpBody.V.X = 0.f;
    indiRun.spfSpBody.V.Y = 0.f;
    indiRun.spfSpBody.V.Z = 0.f;

    indiRun.rateSpBodyCommanded.V.X = 0.f;
    indiRun.rateSpBodyCommanded.V.Y = 0.f;
    indiRun.rateSpBodyCommanded.V.Z = 0.f;

    indiRun.bypassControl = false;
    indiRun.controlAttitude = true;
    indiRun.trackAttitudeYaw = false;

    // update state machines
#ifdef USE_CATAPULT
    runCatapultStateMachine(current);
#endif
#ifdef USE_LEARNER
    runLearningQueryStateMachine(current);
#endif

    // flight mode handling for setpoints
#ifdef USE_CATAPULT
    if (FLIGHT_MODE(CATAPULT_MODE)
            && (catapultState > CATAPULT_IDLE)
            && (catapultState < CATAPULT_DONE)) {
        indiRun.controlAttitude = controlAttitudeFromCat;
        indiRun.attSpNed = attSpNedFromCat;
        indiRun.spfSpBody = spfSpBodyFromCat;
        indiRun.rateSpBodyCommanded = rateSpBodyFromCat;
    } else
#endif
#ifdef USE_LEARNER
    if (FLIGHT_MODE(LEARNER_MODE)
            && (learningQueryState > LEARNING_QUERY_WAITING_FOR_LAUNCH)
            && (learningQueryState < LEARNING_QUERY_DONE)) {
        indiRun.bypassControl = true;
        for (int i=0; i < indiRun.actNum; i++) {
            indiRun.d[i] = outputFromLearningQuery[i];
        }
    } else
#endif
#ifdef USE_LOCAL_POSITION
    if (FLIGHT_MODE(POSITION_MODE) || FLIGHT_MODE(VELOCITY_MODE)) {
        indiRun.attSpNed = attSpNedFromPos;
        indiRun.trackAttitudeYaw = posSpNed.trackPsi;
        indiRun.spfSpBody = spfSpBodyFromPos;
        indiRun.rateSpBodyCommanded = rateSpBodyFromPos;
    } else
#endif
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get desired attitude setpoints from sticks

        // get proper yaw
        float Psi = getYawWithoutSingularity();

        // find tilt axis --> this can be done better! 
        // Now combined XY input gives bigger tilt angle before limiting,
        // which results in a sort of radial deadzone past the x*y=1 circle
        float roll = getRcDeflection(ROLL);
        float pitch = getRcDeflection(PITCH);
        float maxTilt = indiRun.manualMaxTilt;

        fp_vector_t axis = {
            .V.X = maxTilt*roll,
            .V.Y = maxTilt*pitch,
            .V.Z = 0.f,
        };
        VEC3_CONSTRAIN_XY_LENGTH(axis, maxTilt);
        float angle = VEC3_XY_LENGTH(axis);
        VEC3_NORMALIZE(axis);
        fp_quaternion_t attSpYaw;
        quaternion_of_axis_angle(&attSpYaw, &axis, angle);

        fp_quaternion_t yawNed = {
            .w = cos_approx(Psi/2.f),
            .x = 0.f,
            .y = 0.f,
            .z = sin_approx(Psi/2.f),
        };

        // this is probaby the most expensive operation... can be half the cost if
        // optimized for .x = 0, .y = 0, unless compiler does that for us?
        indiRun.attSpNed = chain_quaternion(&yawNed, &attSpYaw);

        // convert throttle
        indiRun.spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        indiRun.spfSpBody.V.Z *= RC_SCALE_THROTTLE * (-indiRun.manualMaxUpwardsSpf);

        // get yaw rate
        indiRun.rateSpBodyCommanded = coordinatedYaw(DEGREES_TO_RADIANS(getSetpointRate(YAW)));

    } else {
        indiRun.controlAttitude = false;
        // acro
        indiRun.rateSpBodyCommanded.V.X = DEGREES_TO_RADIANS(getSetpointRate(ROLL));
        indiRun.rateSpBodyCommanded.V.Y = DEGREES_TO_RADIANS(getSetpointRate(PITCH));
        indiRun.rateSpBodyCommanded.V.Z = DEGREES_TO_RADIANS(getSetpointRate(YAW));

        // convert throttle
        indiRun.spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        indiRun.spfSpBody.V.Z *= RC_SCALE_THROTTLE * (-indiRun.manualMaxUpwardsSpf);
    }
}

#ifdef STM32H7
FAST_CODE
#endif
void getAlphaSpBody(timeUs_t current) {
    UNUSED(current);

    fp_quaternion_t attEstNed;
    getHoverAttitudeQuaternion(&attEstNed);

    fp_quaternion_t attEstNedInv = attEstNed;
    attEstNedInv.w = -attEstNed.w;

    // get rate setpoint from the pilot. If controlAttitude == true, this
    // setpoint will be mixed with the attitude command.
    indiRun.rateSpBody = indiRun.rateSpBodyCommanded;

    //fp_quaternion_t rateSpBodyUse = indiRun.rateSpBody;
    if (indiRun.controlAttitude) {
        // for better readibility and logs, flip setpoint quaternion if needed
        // the resulting quaterion is always exactly equivalent
        if (attEstNed.w * indiRun.attSpNed.w < 0.f)
            QUAT_SCALAR_MULT(indiRun.attSpNed, -1.f);

        // we decompose the error quaternion into first tilt, then yaw
        // q_e^B  =  q_yaw^B  *  q_tilt^B
        //
        // q_yaw = (w 0 0 z)
        // q_tilt = (w x y 0)
        indiRun.attErrBody = chain_quaternion(&attEstNedInv, &indiRun.attSpNed);

        // --- tilt component ---
        // get tilt as the rotation from the body z (0 0 1) to the target bodyZ 
        // which is the cross product of (0 0 1) x targetZBody:
        //
        // tiltAxis = (-targetZBody.V.Y  targetZBody.V.X  0)
        fp_vector_t targetZBody = quatRotMatCol(&indiRun.attErrBody, 2);
        float tiltAxisNorm = hypotf(targetZBody.V.X, targetZBody.V.Y);

        // get tilt axis without singularity traps at 0 and pi
        fp_vector_t tiltAxis = { .V.Z = 0.f };
        if (tiltAxisNorm > 1e-8f) {
            // error = error_angle * unit_vector_rotation_axis
            float itiltAxisNorm = 1.f / tiltAxisNorm;
            tiltAxis.V.X = -targetZBody.V.Y * itiltAxisNorm;
            tiltAxis.V.Y = targetZBody.V.X * itiltAxisNorm;
        } else {
            // just rotate around X, either almost left or right
            tiltAxis.V.X = (-targetZBody.V.Y > 0.f) ? 1.f : -1.f;
            tiltAxis.V.Y = 0.f;
        }

        // now we have the axis, but we still need the angle.
        // a.dot(b) = ||a|| * ||b|| * cos(theta) 
        //      but (0 0 1) and targetZBody are unit vectors
        //      therefore:  (0 0 1).dot(targetZBody) = targetZBody.Z
        //
        // and angle is  theta = acos(targetZBody.Z)
        // on interval 0 to pi (direction is given by tiltAxis)
        //
        // NOTE: some heuristic could be used here, because this is far from optimal
        // in cases where we have high angular rate and the most efficient way
        // to get to the setpoint is actually to continue through a flip, and
        // not counter steer initially
        float tiltErrorAngle = acos_approx(constrainf(targetZBody.V.Z, -1.f, 1.f));

        // finalize tilt error used to generate rate setpoint
        fp_vector_t tiltError = tiltAxis;
        VEC3_SCALAR_MULT(tiltError, tiltErrorAngle);

        // --- yaw component ---
        // we need (only) the w component of q_yaw to get yaw error angle
        // q_yaw^B  =  q_e^B  *  q_tilt^-B

        // setup q_tilt inverse first from ax ang rotation found above
        fp_quaternion_t q_tilt_inv;
        quaternion_of_axis_angle(&q_tilt_inv, &tiltAxis, -tiltErrorAngle);

        // get q_yaw via multiplication.
        // TODO: we only need w and sign(z). Surely there is somethign faster than dense quaternion mult
        fp_quaternion_t q_yaw = chain_quaternion(&indiRun.attErrBody, &q_tilt_inv);

        q_yaw.w = constrainf(q_yaw.w, -1.f, 1.f);
        float yawErrorAngle = 2.f*acos_approx(q_yaw.w);
        if (yawErrorAngle > M_PIf)
            yawErrorAngle -= 2.f*M_PIf; // make sure angleErr is [-pi, pi]

        // we still have to check if the vector compoenent is negative
        // this inverts the error angle
        if (q_yaw.z < 0.f)
            yawErrorAngle = -yawErrorAngle;

        // multiply with gains and mix existing rate setpoint
        indiRun.rateSpBody.V.X += indiRun.attGainsCasc.V.X * tiltError.V.X;
        indiRun.rateSpBody.V.Y += indiRun.attGainsCasc.V.Y * tiltError.V.Y;
        if (indiRun.trackAttitudeYaw)
            indiRun.rateSpBody.V.Z += indiRun.attGainsCasc.V.Z * yawErrorAngle;
        // else: just keep rateSpBody.V.Z that has been set

        // constrain to be safe
        VEC3_CONSTRAIN_XY_LENGTH(indiRun.rateSpBody, indiRun.attMaxTiltRate);
        indiRun.rateSpBody.V.Z = constrainf(indiRun.rateSpBody.V.Z, -indiRun.attMaxYawRate, indiRun.attMaxYawRate);
    }

    // limit to absolute max values
    indiRun.rateSpBody.V.X = constrainf(indiRun.rateSpBody.V.X, -indiRun.maxRateSp.V.X, indiRun.maxRateSp.V.X);
    indiRun.rateSpBody.V.Y = constrainf(indiRun.rateSpBody.V.Y, -indiRun.maxRateSp.V.Y, indiRun.maxRateSp.V.Y);
    indiRun.rateSpBody.V.Z = constrainf(indiRun.rateSpBody.V.Z, -indiRun.maxRateSp.V.Z, indiRun.maxRateSp.V.Z);

    // --- get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    fp_vector_t rateErr = indiRun.rateSpBody;
    VEC3_SCALAR_MULT_ADD(rateErr, -1.0f, indiRun.rate_f);

    // alphaSpBody = rateGains * rateErr
    indiRun.rateDotSpBody.V.X = indiRun.rateGains.V.X * rateErr.V.X;
    indiRun.rateDotSpBody.V.Y = indiRun.rateGains.V.Y * rateErr.V.Y;
    indiRun.rateDotSpBody.V.Z = indiRun.rateGains.V.Z * rateErr.V.Z;
}

#ifdef STM32H7
FAST_CODE
#endif
void getMotorCommands(timeUs_t current) {
    UNUSED(current);

    static float du[MAXU] = {0.f};
    static float rate_prev[XYZ_AXIS_COUNT] = {0.f, 0.f, 0.f};
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY) || defined(MOCKUP)
    static float omega_prev[MAXU] = {0.f};
#endif

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // rate
        indiRun.rateIMU.A[axis] = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[axis]);
        indiRun.rate_f.A[axis] = DEGREES_TO_RADIANS(gyro.gyroADCf[axis]);
        indiRun.spfIMU.A[axis] = acc.accADCafterRpm[axis] * acc.dev.acc_1G_rec * GRAVITYf;

        // get gyro derivative
        indiRun.rateDotIMU.A[axis] = indiRun.indiFrequency * (indiRun.rateIMU.A[axis] - rate_prev[axis]);
        rate_prev[axis] = indiRun.rateIMU.A[axis];

        // filter gyro derivative
        indiRun.rateDot_fs.A[axis] = biquadFilterApply(&indiRun.rateFilter[axis], indiRun.rateDotIMU.A[axis]);

        // get (filtered) accel
        indiRun.spf_fs.A[axis] = biquadFilterApply(&indiRun.spfFilter[axis], indiRun.spfIMU.A[axis]);
    }

#ifdef USE_LEARNER
    // rotate to hover!!
    fp_quaternion_t imu_to_hover;
    imu_to_hover = hoverAttitude; // copy, not pointer
    imu_to_hover.w *= -1.; // inverse
    rotate_vector_with_quaternion(&(indiRun.rate_f), &imu_to_hover);
    rotate_vector_with_quaternion(&(indiRun.rateDot_fs), &imu_to_hover);
    rotate_vector_with_quaternion(&(indiRun.spf_fs), &imu_to_hover);
#endif

    // get rotation speeds and accelerations from dshot, or fallback
    float omega_inv[MAXU];
    for (int i = 0; i < indiRun.actNum; i++) {

        // needed later as well, not just for fallback
        float invThresh = 0.1f * indiRun.actMaxOmega[i];
        omega_inv[i] = (fabsf(indiRun.omega_fs[i]) > invThresh) ? 1.f / indiRun.omega_fs[i] : 1.f / invThresh;
    }

    // get motor acceleration
    for (int i = 0; i < indiRun.actNum; i++) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY) || defined(MOCKUP)
        if (isDshotTelemetryActive() && indiRun.useRpmDotFeedback ) {
            indiRun.omegaDot_fs[i] = (indiRun.omega_fs[i] - omega_prev[i]) * indiRun.indiFrequency;
            omega_prev[i] = indiRun.omega_fs[i];
            // probably do some limiting here
        } else
#endif
        {
            indiRun.omegaDot_fs[i] = du[i] * indiRun.G2_scaler[i] * omega_inv[i];
        }
    }

    // horrible code! Fixme todo
    if (indiRun.bypassControl) { return; }


    // use INDI only when in the air, solve linearized global problem otherwise
    bool doIndi = (!isTouchingGround()) && ARMING_FLAG(ARMED);

    // compute pseudocontrol
    indiRun.dv[0] = 0.f;
    indiRun.dv[1] = 0.f;
    indiRun.dv[2] = indiRun.spfSpBody.V.Z - doIndi * indiRun.spf_fs.V.Z;
    indiRun.dv[3] = indiRun.rateDotSpBody.V.X - doIndi * indiRun.rateDot_fs.V.X;
    indiRun.dv[4] = indiRun.rateDotSpBody.V.Y - doIndi * indiRun.rateDot_fs.V.Y;
    indiRun.dv[5] = indiRun.rateDotSpBody.V.Z - doIndi * indiRun.rateDot_fs.V.Z;

    // add in G2 contributions G2 * omega_dot
    for (int j=0; j < 3; j++) {
        for (int i=0; i < indiRun.actNum; i++) {
            indiRun.dv[j+3] += doIndi * indiRun.actG2[j][i]*indiRun.omegaDot_fs[i];
        }
    }

    // compute pseudoinverse pinv(G1+G2)
    // TODO: use solver DONE
    float G1G2[MAXU*MAXV];
    for (int i=0; i < indiRun.actNum; i++) {
        for (int j=0; j < MAXV; j++) {
            G1G2[MAXV*i + j] = indiRun.actG1[j][i];
            if (j > 2)
                G1G2[MAXV*i + j] += indiRun.G2_scaler[i] * omega_inv[i] * indiRun.actG2[j-3][i];
        }
    }

    float gamma_used;
    float A_as[(MAXU+MAXV) * MAXU];
    float b_as[(MAXU+MAXV)];
    float du_as[MAXU];
    float du_min[MAXU];
    float du_max[MAXU];
    float du_pref[MAXU];

    for (int i=0; i < indiRun.actNum; i++) {
        // todo: what if negative u are possible?
        du_min[i]  = 0.f - doIndi * indiRun.uState_fs[i];
        du_max[i]  = indiRun.actLimit[i] - doIndi * indiRun.uState_fs[i];
        du_pref[i] = 0.f - doIndi * indiRun.uState_fs[i];
    }

    // setup problem
    float Wu_as[MAXU];
    for (int i = 0; i < indiRun.actNum; i++)
        Wu_as[i] = indiRun.wlsWu[i]; // because not const in setupWLS_A

    setupWLS_A(G1G2, indiRun.wlsWv, Wu_as, MAXV, indiRun.actNum, indiRun.wlsTheta, indiRun.wlsCondBound, A_as, &gamma_used);
    setupWLS_b(indiRun.dv, du_pref, indiRun.wlsWv, Wu_as, MAXV, indiRun.actNum, gamma_used, b_as);
    static int8_t Ws[MAXU];
    static activeSetExitCode as_exit_code = AS_SUCCESS;

    for (int i=0; i < indiRun.actNum; i++) {
      du_as[i] = (du_min[i] + du_max[i]) * 0.5f;
      // Assume warmstart is always desired and reset working set Ws only if 
      // if NAN errors were encountered
      if (as_exit_code >= AS_NAN_FOUND_Q)
        Ws[i] = 0;
    }

    // solve problem
    int iterations;
    int n_free;
#ifdef AS_RECORD_COST
    static float alloc_costs[AS_RECORD_COST_N] = {0.f};
#else
    static float alloc_costs[];
#endif

    as_exit_code = solveActiveSet(indiRun.wlsAlgo)(
      A_as, b_as, du_min, du_max, du_as, Ws, indiRun.wlsMaxIter, indiRun.actNum, MAXV,
      &iterations, &n_free, alloc_costs);

    //float G1G2_inv[MAXU][MAXV];
    // pseudoinverse or something?
    if (as_exit_code >= AS_NAN_FOUND_Q) {
        indiRun.nanCounter++;
    } else {
        if (ARMING_FLAG(ARMED))
            indiRun.nanCounter = 0;
    }

    if (indiRun.nanCounter > indiRun.wlsNanLimit) {
        disarm(DISARM_REASON_ALLOC_FAILURE);
    }

    // du = Ginv * dv and then constrain between 0 and 1
    for (int i=0; i < indiRun.actNum; i++) {
        // apply dgyro filters to sync with input
        // also apply gyro filters here?
        indiRun.uState_fs[i] = biquadFilterApply(&indiRun.uStateFilter[i], indiRun.uState[i]);
        indiRun.uState_fs[i] = constrainf(indiRun.uState_fs[i], 0.f, 1.f);

        if (as_exit_code < AS_NAN_FOUND_Q)
            indiRun.u[i] = constrainf(doIndi*indiRun.uState_fs[i] + du_as[i], 0.f, indiRun.actLimit[i]);// currentPidProfile->motor_output_limit * 0.01f);

        // apply lag filter to simulate spinup dynamics
        du[i] = indiRun.u[i] - indiRun.uState[i]; // actual du. SHOULD be identical to du_as, when doIndi

        indiRun.d[i] = indiLinearization(&indiRun.lin[i], indiRun.u[i]);
    }
}

// NB: this function MUST be invoked with the actual normalized motor commands
//     at the same rate as getMotorCommands would be called. EVEN IF NOT IN 
//     INDI MODE. Otherwise switching to INDI may result in a super high jerk.
void indiUpdateActuatorState( float* d ) {
    for (int i=0; i < indiRun.actNum; i++) {
        float u = indiOutputCurve( &indiRun.lin[i], d[i] );
        indiRun.uState[i] = pt1FilterApply( &indiRun.uLagFilter[i], u );

#if (defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)) || defined(MOCKUP)
        if (isDshotTelemetryActive() || getDshotTelemetry(i)) { // getDshotTelemetry triggers an update to DshotTelemitryActive, so the || makes sure we retry
            // to get to rad/s, multiply with erpm scaling (100), then divide by pole pairs and convert rpm to rad
            indiRun.omega[i] = indiRun.erpmToRads * getDshotTelemetry(i);
            indiRun.omega_fs[i] = MAX(0.f, biquadFilterApply(&indiRun.omegaFilter[i], indiRun.omega[i]));
        } else
#endif
        {
            // fallback option 1: fixed omega_hover
            indiRun.omega[i] = indiRun.actHoverOmega[i];
            indiRun.omega_fs[i] = indiRun.actHoverOmega[i];

            // fallback option 2: use actLag filter. TODO
        }
    }
}

// --- helpers --- //
#ifdef STM32H7
FAST_CODE
#endif
float getYawWithoutSingularity(void) {
    // get yaw from bodyX or bodyY axis expressed in inertial space, not from
    // eulers, which has singularity at pitch +-pi/2

    // first, get bodyX and bodyY in NED from the attitude DCM
    fp_vector_t bodyXNed = {
        .V.X = rMat.m[0][0],
        .V.Y = rMat.m[1][0],
        .V.Z = rMat.m[2][0]
    };
    fp_vector_t bodyYNed = {
        .V.X = rMat.m[0][1],
        .V.Y = rMat.m[1][1],
        .V.Z = rMat.m[2][1],
    };

    // second, find which one projects to the longest vector in the xy plane
    float bodyXProjLen = VEC3_XY_LENGTH(bodyXNed);
    float bodyYProjLen = VEC3_XY_LENGTH(bodyYNed);

    if (bodyXProjLen > bodyYProjLen) { // can never be close to 0 at the same time, from the geometry
        // third, get fwd axis as a xy coordinate system aligned with
        // either the bodyX or bodyY projection onto xy
        bodyXNed.V.X /= bodyXProjLen;
        bodyXNed.V.Y /= bodyXProjLen;
    } else {
        bodyXNed.V.X = bodyYNed.V.Y / bodyYProjLen;
        bodyXNed.V.Y = -bodyYNed.V.X / bodyYProjLen;
    }
    float yaw = atan2_approx(bodyXNed.V.Y, bodyXNed.V.X);

    return yaw;
}

#ifdef STM32H7
FAST_CODE
#endif
fp_vector_t coordinatedYaw(float yaw) {
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv;
    getHoverAttitudeQuaternion(&attEstNedInv);
    attEstNedInv.w *= -1.0f;

    fp_vector_t yawRateSpBody = quatRotMatCol(&attEstNedInv, 2);
    // todo: this doesnt need quatRotMatCol, it's simply the last col of the transpose of rMat
    VEC3_SCALAR_MULT(yawRateSpBody, yaw);

    return yawRateSpBody;
}

// init thrust linearization https://www.desmos.com/calculator/v9q7cxuffs
#ifdef STM32H7
FAST_CODE
#endif
void updateLinearization(actLin_t* lin, float k) {
    lin->k = constrainf(k, 0.025f, 0.7f);
    lin->A = 1.f / lin->k;
    lin->B = (sq(lin->k) - 2.f*lin->k + 1.f) / (4.f*sq(lin->k));
    lin->C = (lin->k - 1) / (2.f*lin->k);
}

#ifdef STM32H7
FAST_CODE
#endif
float indiLinearization(actLin_t* lin, float in) {
    if ((lin->A < 1.f) || (lin->B < 0.f))
        // no thrust lin requested/configured or misconfigured
        return in;

    if ((in <= 0.f) || (in >= 1.f))
        // input out of range
        return in;

    return sqrtf(lin->A*in + lin->B) + lin->C;
}

#ifdef STM32H7
FAST_CODE
#endif
float indiOutputCurve(actLin_t* lin, float in) {
    return lin->k*sq(in) + (1-lin->k)*in;
}

#endif // def USE_INDI

// TODO;
/* 
 * 1. DONE use FLIGHT_MODE to decide rc mode
 * 2. DONE incoorperate thrust linearization
 * 3. DONE deal with G2
 * 4. DONE logging of quat setpoint, quat attitude, omega setpoint, omega, alpha setpoint, alpha, motor setpoint
 * 5. DONE use actual settings built-ins
 * 6. NOPE rewrite with different quaternion structs
 * 7. DONE activeSetSolve
 * 8. DONE TO BE TESTED do not check min throttle for indi integrating, breakes down in pos_ctl
 * 9. DONE deal with what happens if neither pos not att are selected
 * 10. compiler macros for USE_ACC
 * 11. DONE REMOVE OMEGA_DOT_FEEDBACK compiler macro
 * 12. DONE REMOVE some PI compiler macros
 * 13. DONE analyse attitude mode wrt yaw/tilt trajectory
 * 14. DONE implement rate control and use rate config from PID
*/
