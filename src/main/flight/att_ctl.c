
#include "att_ctl.h"
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
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include <math.h>
#include "config/config.h"
#include "setupWLS.h"
#include "solveActiveSet.h"

#include "pg/pg.h"

#include <stdbool.h>

//PG_REGISTER_WITH_RESET_FN(indiConfig_t, indiConfig, PG_INDI_CONFIG, 1);
//void pgResetFn_indiConfig(indiConfig_t* indiConfig) {
//    RESET_CONFIG_2(indiConfig_t, indiConfig,
//        .attGains = {.V.X = 800.f, .V.Y = 800.f, .V.Z = 200.f}
//    )
//}

#define RC_SCALE_THROTTLE 0.001f
#define RC_OFFSET_THROTTLE 1000.f
#define RC_MAX_SPF_Z -30.f

fp_quaternion_t attSpNed = {.qi=1.f};
float zAccSpNed;
float yawRateSpNed;
bool controlAttitude = false;
bool trackAttitudeYaw = false;

fp_quaternion_t attErrBody = {.qi=1.f};
t_fp_vector rateSpBody = {0};
t_fp_vector rateSpBodyUse = {0};
t_fp_vector alphaSpBody = {0};
t_fp_vector yawRateSpBody = {0};
t_fp_vector spfSpBody = {0};

t_fp_vector attGains = {.V.X = 200.f, .V.Y = 200.f, .V.Z = 80.f};
t_fp_vector attGainsCasc;
t_fp_vector rateGains = {.V.X = 20.f, .V.Y = 20.f, .V.Z = 10.f};

float u[MAXU] = {0.f};
float u_output[MAXU] = {0.f};
float omega[MAXU] = {0.f};
float omega_dot[MAXU] = {0.f};
float omega_hover = 1900.f;
float u_state[MAXU];
float u_state_sync[MAXU];
float dv[MAXV];
int nu = 4;

float alpha[XYZ_AXIS_COUNT];
biquadFilter_t dgyroNotch[XYZ_AXIS_COUNT];
dtermLowpass_t dgyroLowpass[XYZ_AXIS_COUNT];
dtermLowpass_t dgyroLowpass2[XYZ_AXIS_COUNT];

float actTimeConst = 0.02f; // sec. Don't go below 0.01
pt1Filter_t actLag[MAXU];
biquadFilter_t actNotch[MAXU];
dtermLowpass_t actLowpass[MAXU];
dtermLowpass_t actLowpass2[MAXU];

uint8_t attRateDenom = 8; // do att control only every 8 invokations
uint8_t attExecCounter = 0;

float k_thrust  = 2.58e-7f;
float tau_rpm = 0.02f;
float Tmax = 4.5f;

#define ERPM_PER_LSB             100.0f
float erpmToRad;

// low pass for rpm sensing
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
dtermLowpass_t erpmLowpass[MAXU];
#endif

// refurbish this code somehow
#if (MAXU > AS_N_U) || (MAXV > AS_N_V)
#error "Sizes may be too much for ActiveSetCtlAlloc library"
#endif
static activeSetExitCode as_exit_code = AS_SUCCESS;
static uint8_t nanCounter = 0;

quadLin_t thrustLin = {.A = 0.f, .B = 0.f, .C = 0.f, .k = 0.f};

// todo: tie to rate profile?
float maxRateAttTilt = DEGREES_TO_RADIANS(800.f);
float maxRateAttYaw = DEGREES_TO_RADIANS(400.f);

void indiInit(const pidProfile_t * pidProfile) {
    UNUSED(pidProfile);

    // emulate parallel PD with cascaded (so we can limit velocity)
    attGainsCasc.V.X = attGains.V.X / rateGains.V.X;
    attGainsCasc.V.Y = attGains.V.Y / rateGains.V.Y;
    attGainsCasc.V.Z = attGains.V.Z / rateGains.V.Z;

    erpmToRad = ERPM_PER_LSB / 60.f / (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);

    // init states
    //nu = motorDeviceCount();
    for (int i = 0; i < nu; i++) {
        u_state[i] = 0.f;
        u_state_sync[i] = 0.f;
        omega[i] = 0.f;
    }
    for (int i = 0; i < MAXV; i++)
        dv[i] = 0.f;

    // init thrust linearization https://www.desmos.com/calculator/v9q7cxuffs
    float k_conf = pidProfile->thrustLinearization / 100.f;
    if ((k_conf > 0.025) && (k_conf < 0.7)) {
        thrustLin.k = k_conf;
        thrustLin.A = 1.f / thrustLin.k;
        thrustLin.B = (sq(thrustLin.k) - 2.f*thrustLin.k + 1.f) / (4.f*sq(thrustLin.k));
        thrustLin.C = (thrustLin.k - 1) / (2.f*thrustLin.k);
    }

    // init alpha filters (just copy init values from pid_init)
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        dgyroNotch[axis] = pidRuntime.dtermNotch[axis];
        dgyroLowpass[axis] = pidRuntime.dtermLowpass[axis];
        dgyroLowpass2[axis] = pidRuntime.dtermLowpass2[axis];
    }

    for (int i = 0; i < nu; i++) {
        // init backup actuator state filters
        pt1FilterInit(&actLag[i], pt1FilterGain(1.f / (2.f * M_PIf * actTimeConst), pidRuntime.dT));

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

void indiController(void) {
    if (flightModeFlags) {
        // any flight mode but ACRO --> get attitude setpoint
        if ( !((++attExecCounter)%attRateDenom) ) {
            // rate limit attitude control
            getSetpoints();
            attExecCounter = 0;
        }
    } else {
        // function is cheap, let's go
        getSetpoints();
    }

    // for any flight mode:
    // 1. compute desired angular accelerations
    getAlphaSpBody();

    // allocation and INDI
    getMotor();
}

void getSetpoints(void) {
    // sets:
    // 1. at least one of attSpNed and/or rateSpBody
    // 2. spfSpBody
    // 3. controlAttitude
    // 4. trackAttitudeYaw

    spfSpBody.V.X = 0.f;
    spfSpBody.V.Y = 0.f;
    spfSpBody.V.Z = 0.f;

    rateSpBody.V.X = 0.f;
    rateSpBody.V.Y = 0.f;
    rateSpBody.V.Z = 0.f;

    controlAttitude = true;
    trackAttitudeYaw = false;
    if (FLIGHT_MODE(POSITION_MODE) || FLIGHT_MODE(VELOCITY_MODE)) {
        //trackAttitudeYaw = FLIGHT_MODE(POSITION_MODE);

        // convert acc setpoint from position mode
        getAttSpNedFromAccSpNed(&accSpNed, &attSpNed, &(spfSpBody.V.Z));
        //rateSpBody = coordinatedYaw(yawRateSpFromOuter);
        if (FLIGHT_MODE(POSITION_MODE))
            rateSpBody.V.Z = yawRateSpFromOuter;
        else
            rateSpBody = coordinatedYaw(yawRateSpFromOuter);

    } else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get desired attitude setpoints from sticks

        // get proper yaw
        float Psi = getYawWithoutSingularity();

        // find tilt axis --> this can be done better! 
        // Now combined XY input gives bigger tilt angle before limiting,
        // which results in a sort of radial deadzone past the x*y=1 circle
        float roll = getRcDeflection(ROLL);
        float pitch = -getRcDeflection(PITCH);
        float maxTilt = DEGREES_TO_RADIANS(MAX_BANK_DEGREE);

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
        attSpNed = quatMult(yawNed, attSpYaw);

        // convert throttle
        spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;

        // get yaw rate
        rateSpBody = coordinatedYaw(DEGREES_TO_RADIANS(-getSetpointRate(YAW)));

        //todo: RC_MAX_SPF_Z replace by sum of Fz-row in G1
        //DONE todo: replace call to getAttSpNed by logic more suited to manual flying
        //      ie. just yaw * tilt?

    } else if (flightModeFlags) {
        trackAttitudeYaw = true;

        // unknown, but not acro flight mode, just command upright
        attSpNed.qi = 1.0f;
        attSpNed.qx = 0.f;
        attSpNed.qy = 0.f;
        attSpNed.qz = 0.f;

        spfSpBody.V.Z = 2.f; // low but downwards
    } else {
        controlAttitude = false;
        // acro
        rateSpBody.V.X = DEGREES_TO_RADIANS(getSetpointRate(ROLL));
        rateSpBody.V.Y = DEGREES_TO_RADIANS(-getSetpointRate(PITCH));
        rateSpBody.V.Z = DEGREES_TO_RADIANS(-getSetpointRate(YAW));

        // convert throttle
        spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;
    }
}

void getAlphaSpBody(void) {

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

    rateSpBodyUse = rateSpBody;
    if (controlAttitude) {
        // for better readibility and logs, flip setpoint quaternion if needed
        // the resulting quaterion is always exactly equivalent
        if (attEstNed.qi * attSpNed.qi < 0.f)
            QUAT_SCALAR_MULT(attSpNed, -1.f);

        // add in the error term based on attitude quaternion error
        // todo: fix this. Shortest-distance quat division is no bueno for position control with yaw tracking
        attErrBody = quatMult(attEstNedInv, attSpNed);

        // this would ideally be a normalize instead of constrain, but that is slow...
        attErrBody.qi = constrainf(attErrBody.qi, -1.f, +1.f);

        float angleErr = 2.f*acos_approx(attErrBody.qi);
        if (angleErr > M_PIf)
            angleErr -= 2.f*M_PIf; // make sure angleErr is [-pi, pi]
            // some heuristic could be used here, because this is far from optimal
            // in cases where we have high angular rate and the most efficient way
            // to get to the setpoint is actually to continue through a flip, and
            // not counter steer initially
            // NOTE: if you change this to not be [-pi, pi], then line 345 breaks!

        t_fp_vector attErrAxis = {.V.X = 1.f};
        float norm = sqrtf(1.f - attErrBody.qi*attErrBody.qi);
        // todo: nan happens here!
        if (norm > 1e-8f) {
            // procede as normal
            float normInv = 1.f / norm;
            attErrAxis.V.X = normInv * attErrBody.qx;
            attErrAxis.V.Y = normInv * attErrBody.qy;
            attErrAxis.V.Z = normInv * attErrBody.qz;
        } // else {
            // qi = +- 1 singularity. But this means the error angle is so small, 
            // that we don't care abuot this axis. So chose axis to remain 1,0,0
        // }

        // final error is angle times axis
        VEC3_SCALAR_MULT(attErrAxis, angleErr);

        // multiply with gains and add in
        rateSpBodyUse.V.X += attGainsCasc.V.X * attErrAxis.V.X;
        rateSpBodyUse.V.Y += attGainsCasc.V.Y * attErrAxis.V.Y;
        if (trackAttitudeYaw)
            rateSpBodyUse.V.Z += attGainsCasc.V.Z * attErrAxis.V.Z;
        // else: just keep rateSpBody.V.Z that has been set

        // constrain to be safe
        VEC3_CONSTRAIN_XY_LENGTH(rateSpBodyUse, maxRateAttTilt);
        rateSpBodyUse.V.Z = constrainf(rateSpBodyUse.V.Z, -maxRateAttYaw, maxRateAttYaw);
    }

    // --- get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    t_fp_vector rateErr = rateSpBodyUse;
    VEC3_SCALAR_MULT_ADD(rateErr, -1.0f, rateEstBody);

    // alphaSpBody = rateGains * rateErr
    alphaSpBody.V.X = rateGains.V.X * rateErr.V.X;
    alphaSpBody.V.Y = rateGains.V.Y * rateErr.V.Y;
    alphaSpBody.V.Z = rateGains.V.Z * rateErr.V.Z;
}

void getMotor(void) {
    // mix! And call writeMotors or whatever

    // to generate G1 and G2, see python code src/utils/indi/genGMc.py

    // TODO: just use activeSetSolve
    // TODO: Done. scale with currentPidProfile->motor_output_limit / 100.0f

    // FL, FR, RR, RL
    /*
    float Ginv[4][4] = {
        {-0.0254842f,   0.00042246f,  0.0007886f,   0.00246437f},
        {-0.0254842f,  -0.00042246f,  0.0007886f,  -0.00246437f},
        {-0.0254842f,  -0.00042246f, -0.0007886f,   0.00246437f},
        {-0.0254842f,   0.00042246f, -0.0007886f,  -0.00246437f},
    };
    */

    float G1[MAXV][4] = {
        {0.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 0.f},
        { -10.97752222f,  -10.97752222f,  -10.97752222f,  -10.97752222f},
        { -505.80871408f, -505.80871408f,  505.80871408f,  505.80871408f},
        { -305.98392703f,  305.98392703f, -305.98392703f,  305.98392703f},
        { -63.62310934f,   63.62310934f,   63.62310934f,  -63.62310934f},
    };

    float G2[MAXV][4] = {
        {0.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 0.f},
        {-0.00507791f, 0.00507791f,  0.00507791f, -0.00507791f},
    };

    // 1 / (2 tau k)
    float G2_normalizer = 176366843.f;

    static float du[MAXU] = {0.f};
    static float gyro_prev[XYZ_AXIS_COUNT] = {0.f, 0.f, 0.f};
    static float omega_prev[MAXU] = {0.f};

    // get (filtered) gyro rate derivative
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        alpha[axis] = pidRuntime.pidFrequency * DEGREES_TO_RADIANS(gyro.gyroADCf[axis] - gyro_prev[axis]);
        if ((axis == FD_PITCH) || (axis == FD_YAW))
            alpha[axis] *= (-1.f);
        alpha[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &dgyroNotch[axis], alpha[axis]);
        alpha[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &dgyroLowpass[axis], alpha[axis]);
        alpha[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &dgyroLowpass2[axis], alpha[axis]);
        gyro_prev[axis] = gyro.gyroADCf[axis];
    }

    // get rotation speeds and accelerations from dshot, or fallback
    float omega_inv[MAXU];
    for (int i = 0; i < nu; i++) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (isDshotTelemetryActive()) {
            omega_prev[i] = omega[i];

            // to get to rad/s, multiply with erpm scaling (100), then divide by pole pairs and convert rpm to rad
            omega[i] = pidRuntime.dtermLowpassApplyFn((filter_t *) &erpmLowpass[i], erpmToRad * getDshotTelemetry(i));
        } else
#endif
        {
            // fallback option 1: fixed omega_hover
            omega[i] = omega_hover;

            // fallback option 2: use actLag filter. TODO
        }

        // needed later as well, not just for fallback
        omega_inv[i] = (fabsf(omega[i]) > (0.5f * omega_hover)) ? 1.f / omega[i] : 1.f / omega_hover;
    }

    // use INDI only when in the air, solve global problem otherwise
    bool doIndi = !isTouchingGround();

    // get motor acceleration
    for (int i = 0; i < nu; i++) {
#if defined(USE_OMEGA_DOT_FEEDBACK) && defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (isDshotTelemetryActive()) {
            omega_dot[i] = (omega[i] - omega_prev[i]) * pidRuntime.pidFrequency;
            // probably do some limiting here
        } else
#endif
        {
            omega_dot[i] = (Tmax * du[i]) * omega_inv[i] * G2_normalizer;
        }
    }

    // compute pseudocontrol
    dv[0] = 0.f;
    dv[1] = 0.f;
    dv[2] = spfSpBody.V.Z - doIndi * 9.81f * (-acc.accADC[Z]) * acc.dev.acc_1G_rec;
    dv[3] = alphaSpBody.V.X - doIndi * alpha[FD_ROLL];
    dv[4] = alphaSpBody.V.Y - doIndi * alpha[FD_PITCH];
    dv[5] = alphaSpBody.V.Z - doIndi * alpha[FD_YAW];

    // add in G2 contributions G2 * omega_dot
    for (int j=0; j < MAXV; j++) {
        for (int i=0; i < nu; i++) {
            dv[j] += doIndi * G2[j][i]*omega_dot[i] / Tmax;
        }
    }

    // compute pseudoinverse pinv(G1+G2)
    // TODO: use solver DONE
    float G1G2[MAXU*MAXV];
    for (int i=0; i < nu; i++) {
        for (int j=0; j < MAXV; j++) {
            G1G2[MAXV*i + j] = G1[j][i] + G2_normalizer * omega_inv[i] * G2[j][i];
        }
    }

    float Wv_as[MAXV] = {sqrtf(1.f), sqrtf(1.f), sqrtf(100.f), sqrtf(100.f), sqrtf(100.f), sqrtf(1.f)};
    float Wu_as[MAXU] = {1.f, 1.f, 1.f, 1.f};
    float theta = 1e-4f;
    float cond_bound = 1e9;
    int imax = 1;
    activeSetAlgoChoice as_choice = AS_QR;

    float gamma_used;
    float A_as[(MAXU+MAXV) * MAXU];
    float b_as[(MAXU+MAXV)];
    float du_as[MAXU];
    float du_min[MAXU];
    float du_max[MAXU];
    float du_pref[MAXU];

    float motorMax = constrainf(50.f * 0.01f, 0.05f, 1.0f); // make parameter
    for (int i=0; i < nu; i++) {
        // todo: what if negative u are possible?
        du_min[i]  = 0.f - doIndi*u_state[i];
        du_max[i]  = motorMax - doIndi*u_state[i]; //todo exchange for some existing parameter? 
        du_pref[i] = 0.f - doIndi*u_state[i];
    }

    // setup problem
    setupWLS_A(G1G2, Wv_as, Wu_as, MAXV, nu, theta, cond_bound, A_as, &gamma_used);
    setupWLS_b(dv, du_pref, Wv_as, Wu_as, MAXV, nu, gamma_used, b_as);
    static int8_t Ws[MAXU];

    for (int i=0; i < nu; i++) {
      du_as[i] = (du_min[i] + du_max[i]) * 0.5;
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

    as_exit_code = solveActiveSet(as_choice)(
      A_as, b_as, du_min, du_max, du_as, Ws, imax, nu, MAXV,
      &iterations, &n_free, alloc_costs);

    //float G1G2_inv[MAXU][MAXV];
    // pseudoinverse or something?
    if (as_exit_code >= AS_NAN_FOUND_Q) {
        nanCounter++;
    } else {
        if (ARMING_FLAG(ARMED))
            nanCounter = 0;
    }

    if (nanCounter > 10) {
        disarm(DISARM_REASON_ALLOC_FAILURE);
    }

    // du = Ginv * dv and then constrain between 0 and 1
    for (int i=0; i < nu; i++) {
        //float accumulate = G1G2_inv[i][0] * dv[0];
        //for (int j=1; j < nv; j++)
        //    accumulate += G1G2_inv[i][j] * dv[j];
        if (as_exit_code < AS_NAN_FOUND_Q)
            u[i] = constrainf(doIndi*u_state_sync[i] + du_as[i], 0.f, motorMax);// currentPidProfile->motor_output_limit * 0.01f);

        // apply lag filter to simulate spinup dynamics
        if (ARMING_FLAG(ARMED) || true) {
            du[i] = u[i] - u_state[i]; // actual du. SHOULD be identical to du_as, when doIndi
            u_state[i] = pt1FilterApply(&actLag[i], u[i]);
        } else {
            du[i] = 0.f; // because we actually dont spin. if we don't do this, G2 term gets confused
            u_state[i] = 0.f;
        }

        u_output[i] = indiThrustLinearization(thrustLin, u[i]);

        // apply dgyro filters to sync with input
        // also apply gyro filters here?
        u_state_sync[i] = pidRuntime.dtermNotchApplyFn((filter_t *)&actNotch[i], u_state[i]);
        u_state_sync[i] = pidRuntime.dtermLowpassApplyFn((filter_t *)&actLowpass[i], u_state_sync[i]);
        u_state_sync[i] = pidRuntime.dtermLowpass2ApplyFn((filter_t *)&actLowpass2[i], u_state_sync[i]);
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

void getAttSpNedFromAccSpNed(t_fp_vector* accSpNed, fp_quaternion_t* attSpNed, float* fz) {
    /*
    * system of equations:
    * 
    * a^I = Rz(Psi) @ axang(alpha, axis) @ f^B   +   (0 0 G)'
    *   where:
    *       a^I: craft acceleration in inertial frame
    *       Psi: craft yaw
    *       alpha: tilt angle (assume positive)
    *       axis: tilt axis (tx ty 0), where (tx^2 + ty^2 == 1)
    *       f^B: body forces in body frame. Assume for MC: f^B == (0 0 fz)
    *       G: gravity (9.80665)
    * 
    * define    v^Y := Rz(Psi)^(-1) @ ( a^I - (0 0 G)' )   which results in:
    * 
    * v^Y = axang(alpha, axis) @ f^B  +  (0 0 G)'
    * 
    * We require to solve tilt/thrust setpoint. Ie: solve the following 
    * system for (fz alpha tx ty):
    *   1      ==   tx**2  +  ty**2
    *   vx^Y   ==   ty * sin(alpha) * fz
    *   vy^Y   ==  -tx * sin(alpha) * fz
    *   vz^Y   ==        cos(alpha) * fz
    * 
    * where we obtain the axang multplication from the last column of https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    * 
    * 
    * The solution of the system is given by:
    *   tx      =  sgn(vy) * 1 / sqrt( 1 + (vx / vy)^2 ) = vy / sqrt(vy^2 + vx^2)
    *   ty      =  - (vx * tx) / vy
    *   alpha   =  atan( sqrt(vx^2 + vy^2) / (-vz) )
    *   fz      =  vz / cos(alpha)
    *
    */

    //float Psi = (float) DECIDEGREES_TO_RADIANS(-attitude.values.yaw);
    float Psi = getYawWithoutSingularity();
    float cPsi = cos_approx(Psi);
    float sPsi = sin_approx(Psi);

    // a^I - g^I
    t_fp_vector aSp_min_g = {
        .V.X = accSpNed->V.X,
        .V.Y = accSpNed->V.Y,
        .V.Z = accSpNed->V.Z - 9.80665f,
    };

    // v^I = Rz(Psi)^(-1) @ aSp_min_g
    t_fp_vector v = {
        .V.X =  cPsi * aSp_min_g.V.X  +  sPsi * aSp_min_g.V.Y,
        .V.Y = -sPsi * aSp_min_g.V.X  +  cPsi * aSp_min_g.V.Y,
        .V.Z =                                                  aSp_min_g.V.Z,
    };

    t_fp_vector ax = {0};
    float vx2 = v.V.X*v.V.X;
    float vy2 = v.V.Y*v.V.Y;
    float XYnorm = sqrtf( vx2  +  vy2 );
    if ( XYnorm < 1e-4f ) {
        // 0.01% of a g
        // fall back logic, either ax = (+-1 0 0) or (0 +-1 0)
        // doesnt really matter since alpha will be tiny or close to pi
        ax.V.X = (vy2 >= vx2) ? ((v.V.Y >= 0.f) ? +1.f : -1.f) : 0.f;
        ax.V.Y = (vy2 <  vx2) ? ((v.V.X >= 0.f) ? -1.f : +1.f) : 0.f;
    } else {
        // base case
        if (vy2 > vx2) {
            ax.V.X  =  v.V.Y  /  XYnorm;
            ax.V.Y  =  - (v.V.X * ax.V.X) / v.V.Y;
        } else {
            ax.V.Y  =  - v.V.X  /  XYnorm;
            ax.V.X  =  - (v.V.Y * ax.V.Y) / v.V.X;
        }
    }

    float alpha = atan2_approx( XYnorm, -v.V.Z ); // norm is positive, so this is (0, M_PIf)
    alpha = constrainf(alpha, 0.f, DEGREES_TO_RADIANS(MAX_BANK_DEGREE)); //todo: make parameter

    // *fz = v.V.Z / cos_approx(alpha);
    float fz_target = v.V.Z / cos_approx(alpha);

    // attitude setpoint in the yaw frame
    fp_quaternion_t attSpYaw;
    float_quat_of_axang(&attSpYaw, &ax, alpha);

    // add in the yaw
    fp_quaternion_t yawNed = {
        .qi = cos_approx(Psi/2.f),
        .qx = 0.f,
        .qy = 0.f,
        .qz = sin_approx(Psi/2.f),
    };

    // this is probaby the most expensive operation... can be half the cost if
    // optimized for .qx = 0, .qy = 0, unless compiler does that for us?
    *attSpNed = quatMult(yawNed, attSpYaw);

    // discount thrust of we have not yet reached our attitude
    t_fp_vector zDesNed = quatRotMatCol(*attSpNed, 2);
    float zDotProd = 
        zDesNed.V.X * rMat[0][2]
        + zDesNed.V.Y * rMat[1][2] 
        + zDesNed.V.Z * rMat[2][2];

    *fz = fz_target * constrainf(zDotProd, 0.f, 1.f); // zDotProd is on [-1, +1]
}

t_fp_vector coordinatedYaw(float yaw) {
    // convert yawRateSpNed to Body
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv = {
        .qi = -attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    yawRateSpBody = quatRotMatCol(attEstNedInv, 2);
    VEC3_SCALAR_MULT(yawRateSpBody, yaw);

    return yawRateSpBody;
}

float indiThrustLinearization(quadLin_t lin, float in) {
    if ((lin.A < 1.f) || (lin.B < 0.f))
        // no thrust lin requested/configured or misconfigured
        return in;

    if ((in <= 0.f) || (in >= 1.f))
        // input out of range
        return in;

    return sqrtf(lin.A*in + lin.B) + lin.C;
}

float indiThrustCurve(quadLin_t lin, float in) {
    return lin.k*sq(in) + (1-lin.k)*in;
}

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
