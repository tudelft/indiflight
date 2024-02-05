
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

#include "io/external_pos.h"
#include "io/beeper.h"

#include "pg/pg.h"
#include "config/config.h"
#include "config/config_reset.h"
#include <string.h>

#include <stdbool.h>

PG_REGISTER_ARRAY_WITH_RESET_FN(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles, PG_INDI_PROFILE, 0);

void resetIndiProfile(indiProfile_t *indiProfile) {
    indiProfile_t *p = indiProfile;
    p->attGains[0] = 2000; // roll
    p->attGains[1] = 2000; // pitch
    p->attGains[2] = 1000; // yaw
    p->rateGains[0] = 200; // roll
    p->rateGains[1] = 200; // pitch
    p->rateGains[2] = 200; // yaw
    p->attMaxTiltRate = 800; // deg/s
    p->attMaxYawRate = 400; // deg/s
    p->manualUseCoordinatedYaw = true;
    p->manualMaxUpwardsSpf = 30;
    p->manualMaxTilt = 45; // degrees
    // ---- general INDI config
    p->useIncrement = true;
    p->useRpmDotFeedback = true;
    // ---- INDI actuator config
    for (int i = 0; i < MAXU; i++) {
        p->actHoverRpm[i] = 1500;
        p->actTimeConstMs[i] = 25;
        p->actPropConst[i] = 1e5;
        p->actMaxT[i] = 500;
        p->actNonlinearity[i] = 50;
        p->actLimit[i] = 100;
        p->actG1_fx[i] = 0;
        p->actG1_fy[i] = 0;
        p->actG1_fz[i] = 0;
        p->actG1_roll[i] = 0;
        p->actG1_pitch[i] = 0;
        p->actG1_yaw[i] = 0;
        p->actG2_roll[i] = 0;
        p->actG2_pitch[i] = 0;
        p->actG2_yaw[i] = 0;     // actuator rate effectiveness
        // ---- WLS config
        p->wlsWu[i] = 1;
        p->u_pref[i] = 0;
    }

    p->wlsWv[0] = 1; // fx
    p->wlsWv[1] = 1; // fy
    p->wlsWv[2] = 10; // fz
    p->wlsWv[3] = 10; // roll
    p->wlsWv[4] = 10; // pitch
    p->wlsWv[5] = 1; // yaw

    // ---- Filtering config
    p->imuSyncLp2Hz = 40;

    // -------- inaccessible parameters for now (will always be the values from the reset function)
    // ---- Att/Rate config
    p->attRateDenom = 8;
    // ---- WLS config
    p->useConstantG2 = false;
    p->useRpmFeedback = false;
    // ---- general INDI config
    p->useWls = true;
    p->wlsWarmstart = true;
    p->wlsMaxIter = 1;
    p->wlsAlgo = 1;
    p->wlsCondBound = 1 << 15;
    p->wlsTheta = 1;
    p->wlsNanLimit = 20;
}

void pgResetFn_indiProfiles(indiProfile_t *indiProfiles) {
    for (int i = 0; i < INDI_PROFILE_COUNT; i++) {
        resetIndiProfile(&indiProfiles[i]);
    }
}

indiRuntime_t indiRuntime;
void initIndiRuntime(void) {
    indiRuntime_t *r = &indiRuntime;
    const indiProfile_t *p = indiProfiles(systemConfig()->indiProfileIndex);
    // ---- Att/Rate config
    r->attGains.A[0] = p->attGains[0] * 0.1f;
    r->attGains.A[1] = p->attGains[1] * 0.1f;
    r->attGains.A[2] = p->attGains[2] * 0.1f;
    r->rateGains.A[0] = p->rateGains[0] * 0.1f;
    r->rateGains.A[1] = p->rateGains[1] * 0.1f;
    r->rateGains.A[2] = p->rateGains[2] * 0.1f;
    r->attMaxTiltRate = DEGREES_TO_RADIANS(p->attMaxTiltRate);
    r->attMaxYawRate  = DEGREES_TO_RADIANS(p->attMaxYawRate);
    r->attRateDenom = p->attRateDenom;
    r->manualUseCoordinatedYaw = (bool) p->manualUseCoordinatedYaw;
    r->manualMaxUpwardsSpf = (float) p->manualMaxUpwardsSpf;
    r->manualMaxTilt = DEGREES_TO_RADIANS(p->manualMaxTilt);
    // ---- general INDI config
    r->useIncrement = (bool) p->useIncrement;
    r->useConstantG2 = (bool) p->useConstantG2;
    r->useRpmFeedback = (bool) p->useRpmFeedback;
    r->useRpmDotFeedback = (bool) p->useRpmDotFeedback;
    // ---- INDI actuator config
    for (int i = 0; i < MAXU; i++) {
        r->actHoverOmega[i]   = p->actHoverRpm[i]*10.f / 60.f * 2.f * M_PIf;
        r->actTimeConstS[i]   = p->actTimeConstMs[i] * 1e-3f;
        r->actMaxT[i]         = p->actMaxT[i] * 0.01f;
        r->actNonlinearity[i] = p->actNonlinearity[i] * 0.01f;
        r->actLimit[i]        = p->actLimit[i] * 0.01f;
        r->actG1[0][i] = p->actG1_fx[i] * 0.01f;
        r->actG1[1][i] = p->actG1_fy[i] * 0.01f;
        r->actG1[2][i] = p->actG1_fz[i] * 0.01f;
        r->actG1[3][i] = p->actG1_roll[i]  * 0.1f;
        r->actG1[4][i] = p->actG1_pitch[i] * 0.1f;
        r->actG1[5][i] = p->actG1_yaw[i]   * 0.01f;
        r->actG2[0][i] = p->actG2_roll[i]  * 0.1f / p->actHoverRpm[i] * 0.1f;
        r->actG2[1][i] = p->actG2_pitch[i] * 0.1f / p->actHoverRpm[i] * 0.1f;
        r->actG2[2][i] = p->actG2_yaw[i]   * 0.1f / p->actHoverRpm[i] * 0.1f;
        // ---- WLS config
        r->wlsWu[i] = (float) p->wlsWu[i];
        r->u_pref[i] = p->u_pref[i] * 0.01f;
    }
    for (int i = 0; i < MAXV; i++)
        r->wlsWv[i] = (float) p->wlsWv[i];

    // ---- Filtering config
    r->imuSyncLp2Hz = (float) p->imuSyncLp2Hz;
    // ---- WLS config
    r->wlsAlgo = (activeSetAlgoChoice) p->wlsAlgo;
    r->useWls = (bool) p->useWls;
    r->wlsWarmstart = (bool) p->wlsWarmstart;
    r->wlsMaxIter = p->wlsMaxIter;
    r->wlsCondBound = p->wlsCondBound * 1e4f;
    r->wlsTheta = p->wlsTheta * 1e-4;
    r->wlsNanLimit = p->wlsNanLimit;
}

void changeIndiProfile(uint8_t profileIndex)
{
    if (profileIndex < INDI_PROFILE_COUNT) {
        systemConfigMutable()->indiProfileIndex = profileIndex;
    }

    initIndiRuntime();
}

#define RC_SCALE_THROTTLE 0.001f
#define RC_OFFSET_THROTTLE 1000.f
#define RC_MAX_SPF_Z -30.f

fp_quaternion_t attSpNed = {.qi=1.f};
float zAccSpNed;
float yawRateSpNed;
bool bypassControl = false;
bool controlAttitude = false;
bool trackAttitudeYaw = false;

fp_quaternion_t attErrBody = {.qi=1.f};
t_fp_vector rateSpBody = {0};
t_fp_vector rateSpBodyUse = {0};
t_fp_vector alphaSpBody = {0};
t_fp_vector yawRateSpBody = {0};
t_fp_vector spfSpBody = {0};

t_fp_vector attGains = {.V.X = 200.f, .V.Y = 200.f, .V.Z = 100.f};
t_fp_vector attGainsCasc;
t_fp_vector rateGains = {.V.X = 20.f, .V.Y = 20.f, .V.Z = 20.f};

float u[MAXU] = {0.f};
float u_output[MAXU] = {0.f};
float omegaUnfiltered[MAXU] = {0.f};
float omega[MAXU] = {0.f};
float omega_dot[MAXU] = {0.f};
float omega_hover = 500.f;
float u_state[MAXU];
float u_state_sync[MAXU];
float dv[MAXV];
int nu = 4;

float alpha[XYZ_AXIS_COUNT];
biquadFilter_t dgyroNotch[XYZ_AXIS_COUNT];
dtermLowpass_t dgyroLowpass[XYZ_AXIS_COUNT];
dtermLowpass_t dgyroLowpass2[XYZ_AXIS_COUNT];

pt1Filter_t actLag[MAXU];
biquadFilter_t actNotch[MAXU];
dtermLowpass_t actLowpass[MAXU];
dtermLowpass_t actLowpass2[MAXU];

uint8_t attRateDenom = 8; // do att control only every 8 invokations
uint8_t attExecCounter = 0;

#ifdef USE_INDI_PROFILE_X500
// x500 in sim, not very well tuned
static float G1[MAXV][MAXU] = {
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    { -5.351f,  -5.351f,  -5.351f,  -5.351f},
    {-69.9f , -69.9f ,  69.9f ,  69.9f},
    {-69.9f ,  69.9f , -69.9f ,  69.9f},
    { -9.60f,   9.60f,   9.60f,  -9.60f},
};

static float G2[MAXV][MAXU] = {
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {-0.079, 0.079,  0.079, -0.079},
};

static float kThrust  = 9.9e-6f;
static float tauRpm = 0.02f;
static float Tmax = 8.f;
#endif


// red props racequad
#ifdef USE_INDI_PROFILE_5INCHQUAD
static float G1[MAXV][MAXU] = {
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    {  -28.84615385f,   -28.84615385f,   -28.84615385f,   -28.84615385f},
    {-1040.82649651f, -1040.82649651,  1040.82649651f,  1040.82649651f},
    {-725.15631768f,  725.15631768f, -725.15631768f,  725.15631768f},
    { -84.64688606f,   84.64688606f,   84.64688606f,  -84.64688606f},
};

static float G2[MAXV][MAXU] = {
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {-0.04854287, 0.04854287,  0.04854287, -0.04854287},
};

// static float kThrust  = 1.89e-7f;
// static float tauRpm = 0.02f;
// static float Tmax = 4.2f;
static float kThrust  = 1.447e-6f; // results in the hardcoded G2 normalizer 17271808 used before
static float tauRpm = 0.02f;
static float Tmax = 15.8f;
#endif

#ifdef USE_INDI_PROFILE_CINERAT
// black props racequad. Also controls simulation model very well
static float G1[MAXV][MAXU] = {
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    {   0.f  ,    0.f  ,    0.f  ,    0.f},
    {  -10.5f,   -10.5f,   -10.5f,  -10.5f},
    { -400.f,   -400.f,   400.f,    400.f},
    { -260.f,    260.f,  -260.f,    260.f},
    { -51.f,      51.f,    51.f,    -51.f},
};

static float G2[MAXV][MAXU] = {
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {-0.0045f, 0.0045f,  0.0045f, -0.0045f},
};

static float kThrust  = 2.66e-7f;
static float tauRpm = 0.02f;
static float Tmax = 4.5f;
#endif

#ifdef USE_INDI_PROFILE_TRASHCAN
// trashcan 2S first estimate
static float G1[MAXV][MAXU] = {
    {   0.        ,    0.        ,    0.        ,    0.        },
    {   0.        ,    0.        ,    0.        ,    0.        },
    { -7.0       ,  -7.0       ,  -7.0       ,  -7.0       },
    {-250.43771272, -250.43771272,  250.43771272,  250.43771272},
    {-250.959669  ,  250.959669  , -250.959669  ,  250.959669  },
    { 70.74974316, -70.74974316, -70.74974316,  70.74974316}
};

static float G2[MAXV][MAXU] = {
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.0005, -0.0005,  -0.0005, 0.0005},
};

static float kThrust  = 8.1e-9f;
static float tauRpm = 0.03f;
static float Tmax = 0.5f;
#endif


static float G2_normalizer;

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
    initIndiRuntime();

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

    // indi G2 normalization constant 1 / (2 tau k)
    G2_normalizer = 1.f / (2.f * tauRpm * kThrust);


    // init thrust linearization https://www.desmos.com/calculator/v9q7cxuffs
    float k_conf = pidProfile->thrustLinearization / 100.f;
    if ((k_conf > 0.025f) && (k_conf < 0.7f)) {
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

void indiController(void) {
    if (flightModeFlags
        && (!FLIGHT_MODE(CATAPULT_MODE) || catapultState == CATAPULT_DONE) 
        && (!FLIGHT_MODE(LEARNAFTERCATAPULT_MODE) || learningState == LEARNING_DONE)) {
        // any flight mode but ACRO --> get attitude setpoint
        if ( !((++attExecCounter)%attRateDenom) ) {
            // rate limit attitude control
            getSetpoints();
            attExecCounter = 0;
        }
    } else {
        // function is cheap, let's go
        //stavrow here we get setpoints
        getSetpoints();
    }

    // for any flight mode:
    // 1. compute desired angular accelerations
    getAlphaSpBody();

    // allocation and INDI
    getMotor();
}

catapult_state_t catapultState = CATAPULT_DISABLED;

#ifdef USE_CATAPULT
void disableCatapult(void) {
    beeper(BEEPER_SILENCE);
    catapultState = CATAPULT_DISABLED;
}

PG_REGISTER_WITH_RESET_TEMPLATE(catapultConfig_t, catapultConfig, PG_CATAPULT_CONFIG, 0);
PG_RESET_TEMPLATE(catapultConfig_t, catapultConfig, 
    .altitude = 300,
    .xNed = 0,
    .yNed = 0,
    .rotationRoll = 400,
    .rotationPitch = 400,
    .rotationYaw = 400,
    .rotationTimeMs = 150,
    .upwardsAccel = 20,
);

catapultRuntime_t catapultRuntime;
void initCatapultRuntime(void) {
    catapultRuntime.altitude = catapultConfig()->altitude * 0.01f;
    catapultRuntime.xyNed[0] = catapultConfig()->xNed * 0.01f;
    catapultRuntime.xyNed[1] = catapultConfig()->yNed * 0.01f;
    catapultRuntime.rotationRate[0] = catapultConfig()->rotationRoll;
    catapultRuntime.rotationRate[1] = catapultConfig()->rotationPitch;
    catapultRuntime.rotationRate[2] = catapultConfig()->rotationYaw;
    catapultRuntime.rotationTimeUs = catapultConfig()->rotationTimeMs * 1e3;
    catapultRuntime.upwardsAccel = constrainf(catapultConfig()->upwardsAccel, 1.f, 40.f);
    // FIXME: singularity checks
    float h = catapultRuntime.altitude;
    float a = catapultRuntime.upwardsAccel;
    float g = GRAVITYf;
    float x = catapultRuntime.xyNed[0];
    float y = catapultRuntime.xyNed[1];
    float axis_x = y - posEstNed.V.Y;
    float axis_y = - (x - posEstNed.V.X);
    float distance = hypotf(axis_x, axis_y);
    float crashTime = 2.f*sqrtf(2*h/g);
    float vHorz = distance / crashTime;
    float fireTimeSec = sqrtf(2.f*h / (a + a*a/g));
    float vVertMax = a * fireTimeSec;
    float angleFromVertical = atan2f(vHorz, vVertMax);
    axis_x /= distance;
    axis_y /= distance;
    fp_quaternion_t att = {
        cosf(angleFromVertical / 2.f),
        axis_x * sinf(angleFromVertical / 2.f),
        axis_y * sinf(angleFromVertical / 2.f),
        0.f,
    };
    catapultRuntime.attitude = att;
    catapultRuntime.upwardsAccel /= cosf(angleFromVertical);

    //float fireTimeSec = sqrtf( (sqrtf(g*g + 8.f*a*h*g) - g) / (2.f*a*a) );
    catapultRuntime.fireTimeUs = 1e6 * fireTimeSec;
}

void runCatapultStateMachine(float * spfSpBodyZ, t_fp_vector * rateSpBody) {
#define CATAPULT_DELAY_TIME 1000000 // 1sec
#define CATAPULT_MAX_LAUNCH_TIME 500000 // 0.5sec
#define CATAPULT_ALTITUDE 4.f
#define CATAPULT_ACCELERATION 20.f // must be positive, else segfault!
#define CATAPULT_ROTATION_TIME 150000 // 150ms
    static timeUs_t launchTime = 0;
    static timeUs_t cutoffTime = 0;

    bool disableConditions = !FLIGHT_MODE(POSITION_MODE)
                || (extPosState == EXT_POS_NO_SIGNAL)
                || (posSetpointState == EXT_POS_NO_SIGNAL)
#if defined(USE_EKF) && false
                || !ekf_is_healthy() // implement this!
#endif
                ;
    bool enableConditions = !disableConditions && !ARMING_FLAG(ARMED);

    *spfSpBodyZ = 0.;
    rateSpBody->V.X = 0.;
    rateSpBody->V.Y = 0.;
    rateSpBody->V.Z = 0.;

doMore:
    switch (catapultState) {
        case CATAPULT_DISABLED:
            launchTime = 0;
            cutoffTime = 0;

            if (enableConditions)
                catapultState = CATAPULT_WAITING_FOR_ARM;

            break;
        case CATAPULT_WAITING_FOR_ARM:
            beeper(BEEPER_THROW_TO_ARM);
            if (ARMING_FLAG(ARMED)) {
                launchTime = micros() + CATAPULT_DELAY_TIME;
                catapultState = CATAPULT_DELAY; goto doMore;
            }
            if (disableConditions)
                catapultState = CATAPULT_DISABLED;
            break;
        case CATAPULT_DELAY:
            if (cmpTimeUs(micros(), launchTime) > 0) {
                float a = CATAPULT_ACCELERATION;
                float h = CATAPULT_ALTITUDE;
                float g = GRAVITYf;
                float fireTimeSec = sqrtf( (sqrtf(g*g + 8.f*a*h*g) - g) / (2.f*a*a) );
                timeDelta_t fireTimeUs = 1e6 * fireTimeSec;
                cutoffTime = micros() + ( fireTimeUs > CATAPULT_MAX_LAUNCH_TIME ? CATAPULT_MAX_LAUNCH_TIME : fireTimeUs );
                catapultState = CATAPULT_LAUNCHING; goto doMore;
            }
            break;
        case CATAPULT_LAUNCHING:
            if (cmpTimeUs(micros(), cutoffTime) <= 0) {
                *spfSpBodyZ = -(CATAPULT_ACCELERATION + GRAVITYf);
            } else { 
                catapultState = CATAPULT_ROTATING; goto doMore;
            }
            break;
        case CATAPULT_ROTATING:
            if ( cmpTimeUs(micros(), cutoffTime) <= CATAPULT_ROTATION_TIME ) {
                rateSpBody->V.X = DEGREES_TO_RADIANS(400.f);
                rateSpBody->V.Y = DEGREES_TO_RADIANS(400.f);
                rateSpBody->V.Z = DEGREES_TO_RADIANS(400.f);
            } else {
                beeper(BEEPER_SILENCE);
                catapultState = CATAPULT_DONE;
            }
            break;
        case CATAPULT_DONE:
            break;
    }

}
#endif

learning_state_t learningState = LEARNING_DISABLED;

#ifdef USE_LEARN_AFTER_CATAPULT

#define LEARNING_ACT 4
#define LEARNING_DELAY_TIME 250000 // 250ms
#define LEARNING_STEP_TIME 50000 // 50ms
#define LEARNING_RAMP_TIME 100000 // 100ms
#define LEARNING_OVERLAP_TIME 50000 // 50ms, suggested not more than STEP_TIME
#define LEARNING_STEP_AMPLITUDE 0.35f
#define LEARNING_RAMP_AMPLITUDE 0.70f
#define LEARNING_TOTAL_QUERY_TIME ((LEARNING_STEP_TIME+LEARNING_RAMP_TIME)*LEARNING_ACT - LEARNING_OVERLAP_TIME*(LEARNING_ACT-1))
STATIC_ASSERT(LEARNING_OVERLAP_TIME <= LEARNING_STEP_TIME, "LEARNING_OVERLAP_TIME larger than LEARNING_STEP_TIME untested");
STATIC_ASSERT(LEARNING_TOTAL_QUERY_TIME + LEARNING_DELAY_TIME < 1000000, "Dangerously high total learning time"); // 1 sec

#define LEARNING_GYRO_MAX 1600.f // deg/s

void disableLearning(void) {
    if (catapultState != CATAPULT_DONE)
        learningState = LEARNING_DISABLED; // only reset if catapult is also reset
}

static void resetMotorQueryState(motor_state_t* motorStates, int motor, bool enable) {
    if ( motor >= LEARNING_ACT ) return;

    motor_state_t* p = motorStates + motor;
    if (enable) {
        p->startTime = micros();
        p->queryState = QUERY_STEP;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            p->minGyro[axis] = gyro.gyroADCf[axis] + (-LEARNING_GYRO_MAX - gyro.gyroADCf[axis]) / (LEARNING_ACT - motor);
            p->maxGyro[axis] = gyro.gyroADCf[axis] + (+LEARNING_GYRO_MAX - gyro.gyroADCf[axis]) / (LEARNING_ACT - motor);
        }
    } else {
        p->queryState = QUERY_ZERO;
    } // do nothing if enable, but already enabled
}

void runLearningStateMachine(void) {
    static timeUs_t startAt;
    static motor_state_t motorStates[LEARNING_ACT];

    for (int i=0; i < LEARNING_ACT; i++)  u_output[i] = 0.f;

doMore:
    switch (learningState) {
        case LEARNING_DISABLED:
            startAt = micros() + LEARNING_DELAY_TIME;
            learningState = LEARNING_DELAY; goto doMore;
        case LEARNING_DELAY:
            if (cmpTimeUs(micros(), startAt) > 0) {

                resetMotorQueryState(motorStates, 0, true);
                for (int motor = 1; motor < LEARNING_ACT; motor++)
                    resetMotorQueryState(motorStates, motor, false);

                learningState = LEARNING_ACTIVE; goto doMore;
            }
            break;
        case LEARNING_ACTIVE:
            // invoke learning here!
            for (int motor = 0; motor < LEARNING_ACT; motor++) {
                if (motorStates[motor].queryState == QUERY_ZERO) { break; } // no more motors possible

                // trigger next motor, if overlap time reached
                timeDelta_t time_in_query = cmpTimeUs(micros(), motorStates[motor].startTime);
                if ( (motor+1 < LEARNING_ACT)
                        && ( (time_in_query + LEARNING_OVERLAP_TIME) > (LEARNING_STEP_TIME + LEARNING_RAMP_TIME) ) )
                    if (motorStates[motor+1].queryState == QUERY_ZERO)
                        resetMotorQueryState(motorStates, motor + 1, true);

                // protect somewhat against gyro overrun, probably wont work because of filter delays
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    float val = gyro.gyroADCf[axis];
                    if ( (val > motorStates[motor].maxGyro[axis]) || (val < motorStates[motor].minGyro[axis]) ) {
                        motorStates[motor].queryState = QUERY_DONE;
                    }
                }

doMoreMotors:
                switch (motorStates[motor].queryState) {
                    case QUERY_ZERO: { break; } // should never happen
                    case QUERY_STEP:
                        if (cmpTimeUs(micros(), motorStates[motor].startTime + LEARNING_STEP_TIME) > 0) {
                            motorStates[motor].queryState = QUERY_RAMP; goto doMoreMotors;
                        }
                        u_output[motor] = LEARNING_STEP_AMPLITUDE;
                        break;
                    case QUERY_RAMP: {
                        timeDelta_t time_in_ramp = cmpTimeUs(micros(), motorStates[motor].startTime + LEARNING_STEP_TIME);
                        if (time_in_ramp > LEARNING_RAMP_TIME) {
                            motorStates[motor].queryState = QUERY_DONE; goto doMoreMotors;
                        }
                        u_output[motor] = LEARNING_RAMP_AMPLITUDE * ( 1.f - ( ((float) time_in_ramp) / ((float) LEARNING_RAMP_TIME) ) );
                        break;
                    }
                    case QUERY_DONE: { break; }
                }
            }

            bool allMotorsDone = true;
            for (int motor = 0; motor < LEARNING_ACT; motor++)
                allMotorsDone &= (motorStates[motor].queryState == QUERY_DONE);

            // 10ms grace period, then cutoff, even if learning is not done, because that means error in the state machine
            if (allMotorsDone || (cmpTimeUs(micros(), startAt) > (LEARNING_TOTAL_QUERY_TIME + 10000)) ) {
                learningState = LEARNING_DONE; goto doMore;
            }

            break;
        case LEARNING_DONE: { break; }
    }

    for (int motor = 0; motor < LEARNING_ACT; motor++) {
        u_output[motor] = constrainf(u_output[motor], 0.f, 1.f);
        u[motor] = u_output[motor]; // for logging purposes. TODO: also log du
    }
}
#endif

void getSetpoints(void) {
    // sets:
    // 1. at least one of attSpNed and/or rateSpBody
    // 2. spfSpBody
    // 3. controlAttitude
    // 4. trackAttitudeYaw

    attSpNed.qi = 1.f;
    attSpNed.qx = 0.f;
    attSpNed.qy = 0.f;
    attSpNed.qz = 0.f;

    spfSpBody.V.X = 0.f;
    spfSpBody.V.Y = 0.f;
    spfSpBody.V.Z = 0.f;

    rateSpBody.V.X = 0.f;
    rateSpBody.V.Y = 0.f;
    rateSpBody.V.Z = 0.f;

    bypassControl = false;
    controlAttitude = true;
    trackAttitudeYaw = false;

    // flight mode handling for setpoints
    if (FLIGHT_MODE(CATAPULT_MODE) && (catapultState != CATAPULT_DONE)) {
#ifdef USE_CATAPULT
        // this part is never reached anyway ifndef USE_CATAPULT because the flight mode is disabled
        controlAttitude = false;
        runCatapultStateMachine(&(spfSpBody.V.Z), &rateSpBody);
    } else if (FLIGHT_MODE(CATAPULT_MODE) && FLIGHT_MODE(LEARNAFTERCATAPULT_MODE) && (catapultState == CATAPULT_DONE) && (learningState != LEARNING_DONE)) {
#ifdef USE_LEARN_AFTER_CATAPULT
        bypassControl = true;
        runLearningStateMachine();
#endif
#endif
    } else if (FLIGHT_MODE(POSITION_MODE) || FLIGHT_MODE(VELOCITY_MODE)) {
        //trackAttitudeYaw = FLIGHT_MODE(POSITION_MODE);

        // convert acc setpoint from position mode
        //getAttSpNedFromAccSpNed(&accSpNedFromPos, &attSpNed, &(spfSpBody.V.Z));
        //rateSpBody = coordinatedYaw(yawRateSpFromOuter);
        //if (FLIGHT_MODE(POSITION_MODE))
        //    rateSpBody.V.Z = yawRateSpFromOuter;
        //else
        //    rateSpBody = coordinatedYaw(yawRateSpFromOuter);

        attSpNed = attSpNedFromPos;
        spfSpBody = spfSpBodyFromPos;
        rateSpBody = rateSpBodyFromPos;

    } else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get desired attitude setpoints from sticks

        // get proper yaw
        float Psi = getYawWithoutSingularity();

        // find tilt axis --> this can be done better! 
        // Now combined XY input gives bigger tilt angle before limiting,
        // which results in a sort of radial deadzone past the x*y=1 circle
        float roll = getRcDeflection(ROLL);
        float pitch = -getRcDeflection(PITCH);
        float maxTilt = indiRuntime.manualMaxTilt;

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
        attSpNed = quatMult(&yawNed, &attSpYaw);

        // convert throttle
        spfSpBody.V.Z = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        spfSpBody.V.Z *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;

        // get yaw rate
        rateSpBody = coordinatedYaw(DEGREES_TO_RADIANS(-getSetpointRate(YAW)));

        //todo: RC_MAX_SPF_Z replace by sum of Fz-row in G1
        //DONE todo: replace call to getAttSpNed by logic more suited to manual flying
        //      ie. just yaw * tilt?

    //} else if (flightModeFlags) {
    //    trackAttitudeYaw = true;

    //    // unknown, but not acro flight mode, just command upright
    //    attSpNed.qi = 1.0f;
    //    attSpNed.qx = 0.f;
    //    attSpNed.qy = 0.f;
    //    attSpNed.qz = 0.f;

    //    spfSpBody.V.Z = 2.f; // low but downwards
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
        attErrBody = quatMult(&attEstNedInv, &attSpNed);

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

#if !(defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)) && defined(USE_OMEGA_DOT_FEEDBACK)
    #undef USE_OMEGA_DOT_FEEDBACK
#endif

void getMotor(void) {
    static float du[MAXU] = {0.f};
    static float gyro_prev[XYZ_AXIS_COUNT] = {0.f, 0.f, 0.f};
#ifdef USE_OMEGA_DOT_FEEDBACK
    static float omega_prev[MAXU] = {0.f};
#endif

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
        if (isDshotTelemetryActive()){ // || false) {
#ifdef USE_OMEGA_DOT_FEEDBACK
            omega_prev[i] = omega[i];
#endif

            // to get to rad/s, multiply with erpm scaling (100), then divide by pole pairs and convert rpm to rad
            omegaUnfiltered[i] = erpmToRad * getDshotTelemetry(i);
            omega[i] = pidRuntime.dtermLowpassApplyFn((filter_t *) &erpmLowpass[i], omegaUnfiltered[i]);
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

    // get motor acceleration
    for (int i = 0; i < nu; i++) {
#if defined(USE_OMEGA_DOT_FEEDBACK) && defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (isDshotTelemetryActive()){ // || false) {
            omega_dot[i] = (omega[i] - omega_prev[i]) * pidRuntime.pidFrequency;
            // probably do some limiting here
        } else
#endif
        {
            omega_dot[i] = (Tmax * du[i]) * omega_inv[i] * G2_normalizer;
        }
    }

    // horrible code! Fixme todo
    if (bypassControl) { return; }


    // use INDI only when in the air, solve linearized global problem otherwise
    bool doIndi = (!isTouchingGround()) && ARMING_FLAG(ARMED);

    // compute pseudocontrol
    dv[0] = 0.f;
    dv[1] = 0.f;
    dv[2] = spfSpBody.V.Z - doIndi * GRAVITYf * (-acc.accADC[Z]) * acc.dev.acc_1G_rec;
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

    float motorMax = constrainf(currentPidProfile->motor_output_limit * 0.01f, 0.05f, 1.0f); // make parameter
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

t_fp_vector coordinatedYaw(float yaw) {
    // convert yawRateSpNed to Body
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv = {
        .qi = -attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    yawRateSpBody = quatRotMatCol(&attEstNedInv, 2);
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
