
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

#include "io/external_pos.h"
#include "io/beeper.h"

#include "pg/pg.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "att_ctl_init.h"
#include "att_ctl.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles, PG_INDI_PROFILE, 0);

indiRuntime_t indiRuntime;

#define RC_SCALE_THROTTLE 0.001f
#define RC_OFFSET_THROTTLE 1000.f
#define RC_MAX_SPF_Z -30.f

// refurbish this code somehow
#if (MAXU > AS_N_U) || (MAXV > AS_N_V)
#error "Sizes may be too much for ActiveSetCtlAlloc library"
#endif

/*
void indiInit(const pidProfile_t * pidProfile) {
    UNUSED(pidProfile);
    initIndiRuntime();

    // emulate parallel PD with cascaded (so we can limit velocity)
    attGainsCasc.V.X = attGains.V.X / rateGains.V.X;
    attGainsCasc.V.Y = attGains.V.Y / rateGains.V.Y;
    attGainsCasc.V.Z = attGains.V.Z / rateGains.V.Z;

    erpmToRad = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);

    // init states
    //nu = motorDeviceCount();
    for (int i = 0; i < nu; i++) {
        u_state[i] = 0.f;
        u_state_sync[i] = 0.f;
        omega[i] = 0.f;
    }
    for (int i = 0; i < MAXV; i++)
        dv[i] = 0.f;

    // indi G2 normalization constant 1 / (2 tau k)
    G2_normalizer = 1.f / (2.f * tauRpm * kThrust);



    // init alpha filters (just copy init values from pid_init)
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        dgyroNotch[axis] = pidRuntime.dtermNotch[axis];
        dgyroLowpass[axis] = pidRuntime.dtermLowpass[axis];
        dgyroLowpass2[axis] = pidRuntime.dtermLowpass2[axis];
    }

    for (int i = 0; i < nu; i++) {
        // init backup actuator state filters
        pt1FilterInit(&actLag[i], pt1FilterGain(1.f / (2.f * M_PIf * tauRpm), pidRuntime.dT));

        // rpm feedback filter. A bit handwavy, but using filter constant from
        // alpha lp seems most correct
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        erpmLowpass[i] = pidRuntime.dtermLowpass[0];
#endif

        // init indi filters on act state
        actNotch[i] = pidRuntime.dtermNotch[0];
        actLowpass[i] = pidRuntime.dtermLowpass[0];
        actLowpass2[i] = pidRuntime.dtermLowpass2[0];
    }
}
*/

void indiController(timeUs_t current) {
    indiRuntime_t* r = &indiRuntime;
    if (flightModeFlags
        && (!FLIGHT_MODE(CATAPULT_MODE) || catapultState == CATAPULT_DONE) 
        && (!FLIGHT_MODE(LEARNER_MODE) || learningQueryState == LEARNING_QUERY_DONE)) {
        // any flight mode but ACRO --> get attitude setpoint
        if ( !((++r->attExecCounter)%r->attRateDenom) ) {
            // rate limit attitude control
            getSetpoints(current);
            r->attExecCounter = 0;
        }
    } else {
        // function is cheap, let's go
        //stavrow here we get setpoints
        getSetpoints(current);
    }

    // for any flight mode:
    // 1. compute desired angular accelerations
    getAlphaSpBody(current);

    // allocation and INDI
    getMotor(current);
}

void getSetpoints(timeUs_t current) {
#if !defined(USE_CATAPULT) && !defined(USE_LEARNER)
    UNUSED(current)
#endif

    indiRuntime_t* r = &indiRuntime;

    // sets:
    // 1. at least one of attSpNed and/or rateSpBody
    // 2. spfSpBody
    // 3. controlAttitude
    // 4. trackAttitudeYaw

    r->attSpNed.qi = 1.f;
    r->attSpNed.qx = 0.f;
    r->attSpNed.qy = 0.f;
    r->attSpNed.qz = 0.f;
    r->spfSpBody.V.X = 0.f;
    r->spfSpBody.V.Y = 0.f;
    r->spfSpBody.V.Z = 0.f;
    r->rateSpBody.V.X = 0.f;
    r->rateSpBody.V.Y = 0.f;
    r->rateSpBody.V.Z = 0.f;
    r->bypassControl = false;
    r->controlAttitude = true;
    r->trackAttitudeYaw = false;

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
            && (catapultState != CATAPULT_IDLE)
            && (catapultState != CATAPULT_DONE)) {
        r->controlAttitude = controlAttitudeFromCat;
        r->attSpNed = attSpNedFromCat;
        r->spfSpBody = spfSpBodyFromCat;
        r->rateSpBody = rateSpBodyFromCat;
    } else
#endif
#ifdef USE_LEARNER
    if (FLIGHT_MODE(LEARNER_MODE)
            && (learningQueryState != LEARNING_QUERY_IDLE)
            && (learningQueryState != LEARNING_QUERY_DONE)) {
        r->bypassControl = true;
        for (int i=0; i < MAX_SUPPORTED_MOTORS; i++) {
            r->d[i] = outputFromLearningQuery[i];
        }
    } else
#endif
#ifdef USE_POS_CTL
    if (FLIGHT_MODE(POSITION_MODE) || FLIGHT_MODE(VELOCITY_MODE)) {
        r->attSpNed = attSpNedFromPos;
        r->spfSpBody = spfSpBodyFromPos;
        r->rateSpBody = rateSpBodyFromPos;
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
        float pitch = -getRcDeflection(PITCH);
        float maxTilt = r->manualMaxTilt;

        t_fp_vector axis = {
            .V.X = maxTilt*roll,
            .V.Y = maxTilt*pitch,
            .V.Z = 0.f,
        };
        VEC3_CONSTRAIN_XY_LENGTH(axis, maxTilt);
        float angle = VEC3_XY_LENGTH(axis);
        VEC3_NORMALIZE(axis);
        fp_quaternion_t attSpYaw;
        float_quat_of_axang(&attSpYaw, &axis, angle);

        fp_quaternion_t yawNed = {
            .qi = cos_approx(Psi/2.f),
            .qx = 0.f,
            .qy = 0.f,
            .qz = sin_approx(Psi/2.f),
        };

        // this is probaby the most expensive operation... can be half the cost if
        // optimized for .qx = 0, .qy = 0, unless compiler does that for us?
        r->attSpNed = quatMult(&yawNed, &attSpYaw);

        // convert throttle
        r->spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        r->spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;

        // get yaw rate
        r->rateSpBody = coordinatedYaw(DEGREES_TO_RADIANS(-getSetpointRate(YAW)));

        //todo: RC_MAX_SPF_Z replace by sum of Fz-row in G1
        //DONE todo: replace call to getAttSpNed by logic more suited to manual flying
        //      ie. just yaw * tilt?

    } else {
        r->controlAttitude = false;
        // acro
        r->rateSpBody.V.X = DEGREES_TO_RADIANS(getSetpointRate(ROLL));
        r->rateSpBody.V.Y = DEGREES_TO_RADIANS(-getSetpointRate(PITCH));
        r->rateSpBody.V.Z = DEGREES_TO_RADIANS(-getSetpointRate(YAW));

        // convert throttle
        r->spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        r->spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;
    }
}

void getAlphaSpBody(timeUs_t current) {
    UNUSED(current);

    indiRuntime_t* r = &indiRuntime;

    // get body rates in z-down, x-fwd frame
    t_fp_vector rateEstBody = {
        .V.X = DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]), 
        .V.Y = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_PITCH]),
        .V.Z = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_YAW]),
    };

    fp_quaternion_t attEstNed = {
        .qi = attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    fp_quaternion_t attEstNedInv = attEstNed;
    attEstNedInv.qi = -attEstNed.qi;

    //fp_quaternion_t rateSpBodyUse = r->rateSpBody;
    if (r->controlAttitude) {
        // for better readibility and logs, flip setpoint quaternion if needed
        // the resulting quaterion is always exactly equivalent
        if (attEstNed.qi * r->attSpNed.qi < 0.f)
            QUAT_SCALAR_MULT(r->attSpNed, -1.f);

        // we decompose the error quaternion into first tilt, then yaw
        // q_e^B  =  q_yaw^B  *  q_tilt^B
        //
        // q_yaw = (w 0 0 z)
        // q_tilt = (w x y 0)
        r->attErrBody = quatMult(&attEstNedInv, &r->attSpNed);

        // --- tilt component ---
        // get tilt as the rotation from the body z (0 0 1) to the target bodyZ 
        // which is the cross product of (0 0 1) x targetZBody:
        //
        // tiltAxis = (-targetZBody.V.Y  targetZBody.V.X  0)
        t_fp_vector targetZBody = quatRotMatCol(&r->attErrBody, 2);
        float tiltAxisNorm = hypotf(targetZBody.V.X, targetZBody.V.Y);

        // get tilt axis without singularity traps at 0 and pi
        t_fp_vector tiltAxis = { .V.Z = 0.f };
        if (tiltAxisNorm > 1e-8f) {
            // error = error_angle * unit_vector_rotation_axis
            float itiltAxisNorm = 1.f / tiltAxisNorm;
            tiltAxis.V.X = -targetZBody.V.Y * itiltAxisNorm;
            tiltAxis.V.Y = targetZBody.V.X * itiltAxisNorm;
        } else {
            // just rotate around X, either almost left or right
            tiltAxis.V.X = (-targetZBody.V.Y > 0.) ? 1.f : -1.f;
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
        t_fp_vector tiltError = tiltAxis;
        VEC3_SCALAR_MULT(tiltError, tiltErrorAngle);

        // --- yaw component ---
        // we need (only) the w component of q_yaw to get yaw error angle
        // q_yaw^B  =  q_e^B  *  q_tilt^-B

        // setup q_tilt inverse first from ax ang rotation found above
        fp_quaternion_t q_tilt_inv;
        float_quat_of_axang(&q_tilt_inv, &tiltAxis, -tiltErrorAngle);

        // get q_yaw via multiplication.
        // TODO: we only need qi and sign(qz). Surely there is somethign faster than dense quaternion mult
        fp_quaternion_t q_yaw = quatMult(&r->attErrBody, &q_tilt_inv);

        q_yaw.qi = constrainf(q_yaw.qi, -1.f, 1.f);
        float yawErrorAngle = 2.f*acos_approx(q_yaw.qi);
        if (yawErrorAngle > M_PIf)
            yawErrorAngle -= 2.f*M_PIf; // make sure angleErr is [-pi, pi]

        // we still have to check if the vector compoenent is negative
        // this inverts the error angle
        if (q_yaw.qz < 0.f)
            yawErrorAngle = -yawErrorAngle;

        // multiply with gains and mix existing rate setpoint
        r->rateSpBody.V.X += r->attGainsCasc.V.X * tiltError.V.X;
        r->rateSpBody.V.Y += r->attGainsCasc.V.Y * tiltError.V.Y;
        if (r->trackAttitudeYaw)
            r->rateSpBody.V.Z += r->attGainsCasc.V.Z * yawErrorAngle;
        // else: just keep rateSpBody.V.Z that has been set

        // constrain to be safe
        VEC3_CONSTRAIN_XY_LENGTH(r->rateSpBody, r->attMaxTiltRate);
        r->rateSpBody.V.Z = constrainf(r->rateSpBody.V.Z, -r->attMaxYawRate, r->attMaxYawRate);
    }

    // --- get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    t_fp_vector rateErr = r->rateSpBody;
    VEC3_SCALAR_MULT_ADD(rateErr, -1.0f, rateEstBody);

    // alphaSpBody = rateGains * rateErr
    r->rateDotSpBody.V.X = r->rateGains.V.X * rateErr.V.X;
    r->rateDotSpBody.V.Y = r->rateGains.V.Y * rateErr.V.Y;
    r->rateDotSpBody.V.Z = r->rateGains.V.Z * rateErr.V.Z;
}

#if !(defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)) && defined(USE_OMEGA_DOT_FEEDBACK)
    #undef USE_OMEGA_DOT_FEEDBACK
#endif

void getMotor(timeUs_t current) {
    UNUSED(current);

    indiRuntime_t* r = &indiRuntime;

    static float du[MAXU] = {0.f};
    static float gyro_prev[XYZ_AXIS_COUNT] = {0.f, 0.f, 0.f};
#ifdef USE_OMEGA_DOT_FEEDBACK
    static float omega_prev[MAXU] = {0.f};
#endif

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // get (filtered) gyro rate derivative
        r->rateDot.A[axis] = r->indiFrequency * DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[axis] - gyro_prev[axis]);
        gyro_prev[axis] = gyro.gyroADCf[axis];

        if ((axis == FD_PITCH) || (axis == FD_YAW))
            r->rateDot.A[axis] *= (-1.f);

        r->rateDot_fs.A[axis] = biquadFilterApply(&r->rateFilter[axis], r->rateDot.A[axis]);

        // get (filtered) accel
        r->spf_fs.A[axis] = biquadFilterApply(&r->spfFilter[axis], acc.accADCunfiltered[axis]);
    }


    // get rotation speeds and accelerations from dshot, or fallback
    float omega_inv[MAXU];
    for (int i = 0; i < r->actNum; i++) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (isDshotTelemetryActive()){ // || false) {
#ifdef USE_OMEGA_DOT_FEEDBACK
            omega_prev[i] = r->omega_fs[i];
#endif

            // to get to rad/s, multiply with erpm scaling (100), then divide by pole pairs and convert rpm to rad
            r->omega[i] = r->erpmToRads * getDshotTelemetry(i);
            r->omega_fs[i] = biquadFilterApply(&r->omegaFilter[i], r->omega[i]);
        } else
#endif
        {
            // fallback option 1: fixed omega_hover
            r->omega[i] = r->actHoverOmega[i];
            r->omega_fs[i] = r->actHoverOmega[i];

            // fallback option 2: use actLag filter. TODO
        }

        // needed later as well, not just for fallback
        omega_inv[i] = (fabsf(r->omega_fs[i]) > (0.5f * r->actHoverOmega[i])) ? 1.f / r->omega_fs[i] : 1.f / r->actHoverOmega[i];
    }

    // get motor acceleration
    for (int i = 0; i < r->actNum; i++) {
#if defined(USE_OMEGA_DOT_FEEDBACK) && defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (isDshotTelemetryActive()){ // || false) {
            r->omegaDot_fs[i] = (r->omega_fs[i] - omega_prev[i]) * r->indiFrequency;
            // probably do some limiting here
        } else
#endif
        {
            r->omegaDot_fs[i] = (r->actMaxT[i] * du[i]) * omega_inv[i] * r->G2_normalizer[i];
        }
    }

    // horrible code! Fixme todo
    if (r->bypassControl) { return; }


    // use INDI only when in the air, solve linearized global problem otherwise
    bool doIndi = (!isTouchingGround()) && ARMING_FLAG(ARMED);

    // compute pseudocontrol
    r->dv[0] = 0.f;
    r->dv[1] = 0.f;
    r->dv[2] = r->spfSpBody.V.Z - doIndi * GRAVITYf * (-acc.accADC[Z]) * acc.dev.acc_1G_rec;
    r->dv[3] = r->rateDotSpBody.V.X - doIndi * r->rateDot_fs.V.X;
    r->dv[4] = r->rateDotSpBody.V.Y - doIndi * r->rateDot_fs.V.Y;
    r->dv[5] = r->rateDotSpBody.V.Z - doIndi * r->rateDot_fs.V.Z;

    // add in G2 contributions G2 * omega_dot
    for (int j=0; j < 3; j++) {
        for (int i=0; i < r->actNum; i++) {
            r->dv[j+3] += doIndi * r->actG2[j][i]*r->omegaDot_fs[i] / r->actMaxT[i];
        }
    }

    // compute pseudoinverse pinv(G1+G2)
    // TODO: use solver DONE
    float G1G2[MAXU*MAXV];
    for (int i=0; i < r->actNum; i++) {
        for (int j=0; j < MAXV; j++) {
            G1G2[MAXV*i + j] = r->actG1[j][i];
            if (j > 2)
                G1G2[MAXV*i + j] += r->G2_normalizer[i] * omega_inv[i] * r->actG2[j-3][i];
        }
    }

    //float Wv_as[MAXV] = {sqrtf(1.f), sqrtf(1.f), sqrtf(100.f), sqrtf(100.f), sqrtf(100.f), sqrtf(1.f)};
    //float Wu_as[MAXU] = {1.f, 1.f, 1.f, 1.f};
    //float theta = 1e-4f;
    //float cond_bound = 1e9;
    //int imax = 1;
    //activeSetAlgoChoice as_choice = AS_QR;

    float gamma_used;
    float A_as[(MAXU+MAXV) * MAXU];
    float b_as[(MAXU+MAXV)];
    float du_as[MAXU];
    float du_min[MAXU];
    float du_max[MAXU];
    float du_pref[MAXU];

    for (int i=0; i < r->actNum; i++) {
        // todo: what if negative u are possible?
        du_min[i]  = 0.f - doIndi * r->uState[i];
        du_max[i]  = r->actLimit[i] - doIndi * r->uState[i];
        du_pref[i] = 0.f - doIndi * r->uState[i];
    }

    // setup problem
    float Wu_as[MAXU];
    for (int i = 0; i < r->actNum; i++)
        Wu_as[i] = r->wlsWu[i]; // because not const in setupWLS_A

    setupWLS_A(G1G2, r->wlsWv, Wu_as, MAXV, r->actNum, r->wlsTheta, r->wlsCondBound, A_as, &gamma_used);
    setupWLS_b(r->dv, du_pref, r->wlsWv, Wu_as, MAXV, r->actNum, gamma_used, b_as);
    static int8_t Ws[MAXU];
    static activeSetExitCode as_exit_code = AS_SUCCESS;

    for (int i=0; i < r->actNum; i++) {
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

    as_exit_code = solveActiveSet(r->wlsAlgo)(
      A_as, b_as, du_min, du_max, du_as, Ws, r->wlsMaxIter, r->actNum, MAXV,
      &iterations, &n_free, alloc_costs);

    //float G1G2_inv[MAXU][MAXV];
    // pseudoinverse or something?
    if (as_exit_code >= AS_NAN_FOUND_Q) {
        r->nanCounter++;
    } else {
        if (ARMING_FLAG(ARMED))
            r->nanCounter = 0;
    }

    if (r->nanCounter > 10) {
        disarm(DISARM_REASON_ALLOC_FAILURE);
    }

    // du = Ginv * dv and then constrain between 0 and 1
    for (int i=0; i < r->actNum; i++) {
        //float accumulate = G1G2_inv[i][0] * dv[0];
        //for (int j=1; j < nv; j++)
        //    accumulate += G1G2_inv[i][j] * dv[j];
        if (as_exit_code < AS_NAN_FOUND_Q)
            r->u[i] = constrainf(doIndi*r->uState_fs[i] + du_as[i], 0.f, r->actLimit[i]);// currentPidProfile->motor_output_limit * 0.01f);

        // apply lag filter to simulate spinup dynamics
        if (ARMING_FLAG(ARMED) || true) {
            du[i] = r->u[i] - r->uState[i]; // actual du. SHOULD be identical to du_as, when doIndi
            r->uState[i] = pt1FilterApply(&r->uLagFilter[i], r->u[i]);
        } else {
            du[i] = 0.f; // because we actually dont spin. if we don't do this, G2 term gets confused
            r->uState[i] = 0.f;
        }

        r->d[i] = indiLinearization(&r->lin[i], r->u[i]);

        // apply dgyro filters to sync with input
        // also apply gyro filters here?
        r->uState_fs[i] = biquadFilterApply(&r->uStateFilter[i], r->uState[i]);
    }
}

// --- helpers --- //
float getYawWithoutSingularity(void) {
    // get yaw from bodyX or bodyY axis expressed in inertial space, not from
    // eulers, which has singularity at pitch +-pi/2

    // first, get bodyX and bodyY in NED from the attitude DCM
    t_fp_vector bodyXNed = {
        .V.X =  rMat[0][0],
        .V.Y = -rMat[1][0],
        .V.Z = -rMat[2][0]
    };
    t_fp_vector bodyYNed = {
        .V.X = -rMat[0][1],
        .V.Y =  rMat[1][1],
        .V.Z =  rMat[2][1],
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

t_fp_vector coordinatedYaw(float yaw) {
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv = {
        .qi = -attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    t_fp_vector yawRateSpBody = quatRotMatCol(&attEstNedInv, 2);
    VEC3_SCALAR_MULT(yawRateSpBody, yaw);

    return yawRateSpBody;
}

// init thrust linearization https://www.desmos.com/calculator/v9q7cxuffs
void updateLinearization(actLin_t* lin, float k) {
    lin->k = constrainf(k, 0.025f, 0.7f);
    lin->A = 1.f / lin->k;
    lin->B = (sq(lin->k) - 2.f*lin->k + 1.f) / (4.f*sq(lin->k));
    lin->C = (lin->k - 1) / (2.f*lin->k);
}

float indiLinearization(actLin_t* lin, float in) {
    if ((lin->A < 1.f) || (lin->B < 0.f))
        // no thrust lin requested/configured or misconfigured
        return in;

    if ((in <= 0.f) || (in >= 1.f))
        // input out of range
        return in;

    return sqrtf(lin->A*in + lin->B) + lin->C;
}

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
 * 5. use actual settings built-ins
 * 6. NOPE rewrite with different quaternion structs
 * 7. DONE activeSetSolve
 * 8. TO BE TESTED do not check min throttle for indi integrating, breakes down in pos_ctl
 * 9. deal with what happens if neither pos not att are selected
 * 10. compiler macros for USE_ACC
 * 11. REMOVE OMEGA_DOT_FEEDBACK compiler macro
 * 12. REMOVE some PI compiler macros
 * 13. analyse attitude mode wrt yaw/tilt trajectory
 * 14. implement rate control and use rate config from PID
*/
