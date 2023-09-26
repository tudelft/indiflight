
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

#define RC_SCALE_STICKS 0.002f
#define RC_SCALE_THROTTLE 0.001f
#define RC_OFFSET_THROTTLE 1000.f
#define RC_MAX_YAWRATE_DEG_S 300.f
#define RC_MAX_SPF_Z -30.f

fp_quaternion_t attSpNed = {.qi=1.f};
bool attTrackYaw = true;
float zAccSpNed;
float yawRateSpNed;

fp_quaternion_t attErrBody = {.qi=1.f};
t_fp_vector rateSpBody = {0};
t_fp_vector alphaSpBody = {0};
t_fp_vector yawRateSpBody = {0};
t_fp_vector spfSpBody = {0};

t_fp_vector attGains = {.V.X = 100.f, .V.Y = 100.f, .V.Z = 50.f};
t_fp_vector rateGains = {.V.X = 15.f, .V.Y = 15.f, .V.Z = 7.f};

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

quadLin_t thrustLin = {.A = 0.f, .B = 0.f, .C = 0.f, .k = 0.f};

void indiInit(const pidProfile_t * pidProfile) {
    UNUSED(pidProfile);

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
    getAlphaBody();
    getSpfSpBodyZ();

    // allocation and INDI
    getMotor();
}

void getAlphaBody(void) {

    // get body rates in z-down, x-fwd frame
    t_fp_vector rateEstBody = {
        .V.X = DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]), 
        .V.Y = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_PITCH]),
        .V.Z = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_YAW]),
    };

    // emulate parallel PD with cascaded (so we can limit velocity)
    t_fp_vector attGainsCasc = {
        .V.X = attGains.V.X / rateGains.V.X,
        .V.Y = attGains.V.Y / rateGains.V.Y,
        .V.Z = attGains.V.Z / rateGains.V.Z
    };

    // --- get body rates setpoint
    // initialize to yaw-rate (transform) from higher level controllers/sticks
    getYawRateSpBody();
    rateSpBody = yawRateSpBody;

    // add in the error term based on attitude quaternion error
    getAttErrBody(); // sets attErrBody

    float angleErr = 2.f*acos_approx(attErrBody.qi); // should never be above 1 or below -1
    if (angleErr > M_PIf)
        angleErr -= 2.f*M_PIf; // make sure angleErr is [-pi, pi]
        // some heuristic could be used here, because this is far from optimal
        // in cases where we have high angular rate and the most efficient way
        // to get to the setpoint is actually to continue through a flip, and
        // not counter steer initially

    t_fp_vector attErrAxis = {.V.X = 1.f};
    float norm = sqrtf(1.f - attErrBody.qi*attErrBody.qi);
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

    // multiply with gains and constrain
    VEC3_ELEM_MULT_ADD(rateSpBody, attGainsCasc, attErrAxis);
    VEC3_CONSTRAIN_XY_LENGTH(rateSpBody, DEGREES_TO_RADIANS(ATT_MAX_RATE_XY));
    rateSpBody.V.Z = constrainf(rateSpBody.V.Z, -DEGREES_TO_RADIANS(ATT_MAX_RATE_Z), DEGREES_TO_RADIANS(ATT_MAX_RATE_Z));

    // --- get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    t_fp_vector rateErr = rateSpBody;
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
    static int8_t as_exit_code = AS_SUCCESS;

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

    // du = Ginv * dv and then constrain between 0 and 1
    for (int i=0; i < nu; i++) {
        //float accumulate = G1G2_inv[i][0] * dv[0];
        //for (int j=1; j < nv; j++)
        //    accumulate += G1G2_inv[i][j] * dv[j];
        u[i] = constrainf(doIndi*u_state[i] + du_as[i], 0.f, motorMax);// currentPidProfile->motor_output_limit * 0.01f);

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
        u_state_sync[i] = pidRuntime.dtermNotchApplyFn((filter_t *)&actNotch[i], u_state[i]);
        u_state_sync[i] = pidRuntime.dtermLowpassApplyFn((filter_t *)&actLowpass[i], u_state_sync[i]);
        u_state_sync[i] = pidRuntime.dtermLowpass2ApplyFn((filter_t *)&actLowpass2[i], u_state_sync[i]);
    }
}

void getYawRateSpBody(void) {
    // get yawRateSpNed, unless assumed set by position controller
    if ( (!FLIGHT_MODE(POSITION_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)) ) {
        // human pilot, get from RC
        yawRateSpNed = -getSetpointRate(YAW) * RC_SCALE_STICKS;
        yawRateSpNed *= DEGREES_TO_RADIANS(RC_MAX_YAWRATE_DEG_S);
    }
    // no else... just assume that yawRateSpNed has been set properly by the posiotion controller

    // convert yawRateSpNed to Body
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv = {
        .qi = -attitude_q.w,
        .qx = attitude_q.x,
        .qy = attitude_q.y,
        .qz = attitude_q.z,
    };
    yawRateSpBody = quatRotMatCol(attEstNedInv, 2);
    VEC3_SCALAR_MULT(yawRateSpBody, yawRateSpNed);
}

void getSpfSpBodyZ(void) {
    // similar to getYawRateSpBody, decide which Ned z accel to use and transform

    if (FLIGHT_MODE(POSITION_MODE)) {
        // assume that zAccSpNed has been set by position controller
        spfSpBody.V.Z = (-9.81f + zAccSpNed);

        float cos_tilt_angle = getCosTiltAngle();
        if ((cos_tilt_angle < 0.5f) && (cos_tilt_angle > 0.f)) {
            // high tilt, but not inverted, limit divisor to 0.5
            cos_tilt_angle = 0.5f;
        } else if (cos_tilt_angle <= 0.f) {
            // inverted: disable throttle correction
            cos_tilt_angle = 1.0f;
        }

        spfSpBody.V.Z /= cos_tilt_angle;
        // note: this only gives offset free pos control, if spfSpBodyZ is reached 
        //       offset-free, ie through INDI
        // note2: if accSpNed.V.Z is non-zero, then accSpNed is over or undershot.
        //        Can we compromise on both at the same time somehow? or adjust 
        //        tilt angle setpoint based on accSpNed? There will be a system of 
        //        constrained nl equations that can be solved within bounds.

        // this should probably be in pos_ctl, because of the tradeoff... another allocation step?
    } else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get from RC directly
        spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;
    } else {
        // command a downwards setpoint (positive)
        spfSpBody.V.Z = 2.f;
    }
}

void getAttErrBody(void) {
    // get attitude estimate and inverse
    // todo: rewrite with the quaternion type defined in imu
    fp_quaternion_t attEstNed = {
        .qi = attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    fp_quaternion_t attEstNedInv = attEstNed;
    attEstNedInv.qi = -attEstNed.qi;

    // get error
    if (FLIGHT_MODE(POSITION_MODE)) {
        attErrBody = quatMult(attEstNedInv, attSpNed);
    } else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {

        // slow, but good code clarity
        t_fp_vector bodyXNed = quatRotMatCol(attEstNed, 0);
        float bodyXProjLen = VEC3_XY_LENGTH(bodyXNed);
        t_fp_vector bodyYNed = quatRotMatCol(attEstNed, 1);
        float bodyYProjLen = VEC3_XY_LENGTH(bodyYNed);

        t_fp_vector stickXNed = {0};
        t_fp_vector stickYNed = {0};
        if (bodyXProjLen > bodyYProjLen) { // can never be 0 together
            // stickX
            stickXNed.V.X = bodyXNed.V.X / bodyXProjLen;
            stickXNed.V.Y = bodyXNed.V.Y / bodyXProjLen;
            // stickY
            stickYNed.V.X = -stickXNed.V.Y;
            stickYNed.V.Y = +stickXNed.V.X;
        } else {
            // stickY
            stickYNed.V.X = bodyYNed.V.X / bodyYProjLen;
            stickYNed.V.Y = bodyYNed.V.Y / bodyYProjLen;
            // stickX
            stickXNed.V.X = +stickYNed.V.Y;
            stickXNed.V.Y = -stickYNed.V.X;
        }

        // find tilt axis --> this can be done better! 
        // Now combined XY input gives bigger tilt angle before limiting,
        // which results in a sort of radial deadzone past the x*y=1 circle
        float roll = getSetpointRate(ROLL) * RC_SCALE_STICKS;
        float pitch = getSetpointRate(PITCH) * RC_SCALE_STICKS;
#define RC_MAX_TILT_DEG 20.f
        float rc_scaler = tan_approx(DEGREES_TO_RADIANS(RC_MAX_TILT_DEG));
        t_fp_vector bodyZspNed = {
            .V.X = rc_scaler * (-stickXNed.V.X * pitch + -stickYNed.V.X * roll),
            .V.Y = rc_scaler * (-stickXNed.V.Y * pitch + -stickYNed.V.Y * roll),
            .V.Z = 1.
        };
        VEC3_CONSTRAIN_XY_LENGTH(bodyZspNed, rc_scaler); // limit total tilt
        VEC3_SCALAR_MULT(bodyZspNed, 1/VEC3_LENGTH(bodyZspNed)); // normalize

        // construct shortest possible rotation quaternion from BodyZ to 
        // BodyZ setpoint
        // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/halfAngle.htm

        t_fp_vector bodyZNed = quatRotMatCol(attEstNed, 2);
        float k_cos_theta = VEC3_DOT(bodyZNed, bodyZspNed);
        t_fp_vector axis;
        if (k_cos_theta <= -0.9999f) {
            // we are upside down with respect to the target z axis.
            // --> choose some orthogonal axis and rotate 180 degrees.
            attErrBody.qi = 0.f;

            // in particular choose cross product with the most orthogonal
            // coordinate axis for speed.
            float x = fabsf(bodyZNed.V.X);
            float y = fabsf(bodyZNed.V.Y);
            float z = fabsf(bodyZNed.V.Z);
            int most_ortho = (x < y) ? ((x < z) ? 0 : 1) : ((y < z) ? 1 : 2);
            // axis = normalize( cross(bodyZNed, most_ortho) )
            switch(most_ortho){
                case 0:
                    axis.V.X = 0.f;
                    axis.V.Y = +bodyZNed.V.Z;
                    axis.V.Z = -bodyZNed.V.Y;
                    break;
                case 1:
                    axis.V.X = -bodyZNed.V.Z;
                    axis.V.Y = 0.f;
                    axis.V.Z = +bodyZNed.V.X;
                    break;
                case 2:
                    axis.V.X = +bodyZNed.V.Y;
                    axis.V.Y = -bodyZNed.V.X;
                    axis.V.Z = 0.f;
                    break;
            }
            VEC3_NORMALIZE(axis);
            attErrBody.qx = axis.V.X;
            attErrBody.qy = axis.V.Y;
            attErrBody.qz = axis.V.Z;
        } else {
            // normal case, just follow http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/halfAngle.htm
            // scalar = k_cos_theta + 1
            // axis = cross(bodyZNed, bodyZSpNed)
            // normalize the quat
            attErrBody.qi = k_cos_theta + 1.f;
            VEC3_CROSS(axis, bodyZspNed, bodyZNed);
            axis = quatRotate(attEstNedInv, axis);
            attErrBody.qx = axis.V.X;
            attErrBody.qy = axis.V.Y;
            attErrBody.qz = axis.V.Z;
            QUAT_NORMALIZE(attErrBody);
        }
    } else {
        // not sure what happened, just command upright
        attSpNed.qi = 1.f;
        attSpNed.qx = 0.f;
        attSpNed.qy = 0.f;
        attSpNed.qz = 0.f;
        attErrBody = quatMult(attEstNedInv, attSpNed);
    }
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
 * 4. logging of quat setpoint, quat attitude, omega setpoint, omega, alpha setpoint, alpha, motor setpoint
 * 5. use actual settings built-ins
 * 6. rewrite with different quaternion structs
 * 7. DONE activeSetSolve
 * 8. do not check min throttle for indi integrating, breakes down in pos_ctl
 * 9. deal with what happens if neither pos not att are selected
 * 10. compiler macros for USE_ACC
 * 11. REMOVE OMEGA_DOT_FEEDBACK compiler macro
 * 12. REMOVE some PI compiler macros
*/
