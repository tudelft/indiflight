
#include "common/maths.h"
#include <stdbool.h>

#include "platform.h"
#include "flight/pid.h"
#include "common/maths.h"
#include "solveActiveSet.h"

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
    int16_t attGains[3]; // attitude error to rotational accel gain * 10
    int16_t rateGains[3]; // rate error to rotational accel gain * 10
    uint16_t attMaxTiltRate; // max tilt rate in deg/s
    uint16_t attMaxYawRate; // max yaw rate in deg/s
    uint8_t attUseSpfAttenuation;  // bool: enable correcting thrust setpoint if target attitude has not yet been reached
    uint8_t attCoordinatedYaw;     // bool: coordinate yaw in manual flight in Angle/Horizon mode
    uint8_t manualMaxUpwardsSpfZ; // maximum upwards specific force in manual flight in N/kg. 255 means max of the platform
    uint8_t manualMaxTilt; // in manual flight in deg (0, 180)
    uint8_t autoMaxTilt;   // in automatic flight in deg (0, 180)
    // ---- general INDI config
    uint8_t useIncrement;          // bool: use incremental law. NDI (or more precisely linDI) otherwise
    uint8_t useRpmDotFeedback;     // bool: make use of dshot rpm derivative data in feedback loop if available
    // ---- INDI actuator config
    uint16_t actHoverRpm[MAXU];     // approximate rpm/10 in hover flight. FIXME: make an estimator for this
    uint8_t actTimeConst[MAXU];     // time constant for actuator spool up in ms
    uint32_t actPropConst[MAXU];    // propeller constant in N / (rad/s)^2 * 1e11
    uint16_t actMaxT[MAXU];         // max motor thrust in N*100 (centinewton)
    uint8_t actNonlinearity[MAXU];  // motor nonlinearity percentage between (0, 100)
    uint8_t actLimit[MAXU];         // limit motor output in percent (100 full power)
    int16_t actG1_fx[MAXU];      // actuator effectiveness (N/kg) / u * 100
    int16_t actG1_fy[MAXU];      // actuator effectiveness * 100
    int16_t actG1_fz[MAXU];      // actuator effectiveness * 100
    int16_t actG1_roll[MAXU];    // actuator effectiveness (Nm/(kg m^2)) / u * 10
    int16_t actG1_pitch[MAXU];   // actuator effectiveness * 10 
    int16_t actG1_yaw[MAXU];     // actuator effectiveness * 100
    int16_t actG2_roll[MAXU];    // actuator rate effectiveness (Nm/(kg m^2)) / (rad/s/s) * hoverRpm * 10
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
    // ---- WLS config
    uint8_t useConstantG2;         // bool: do not adapt spinup terms based on rpm data, if available and useWls is true
    uint8_t useRpmFeedback;        // bool: make use of dshot rpm data in feedback loop if available. FIXME: actually implement this
    // ---- general INDI config
    uint8_t useWls;              // bool: enable Wls in favour of static pinv
    uint8_t wlsWarmstart;        // bool: use warmstarting of wls
    uint8_t wlsMaxIter;          // wls iteration limit per loop. Keep to 1 usually, if warmstarting is used
    uint8_t wlsAlgo;             // 0: QR, but slow. 1: QR, 2: Chol
    uint16_t wlsCondBound;       // bound condition number to wlsCondBound * 1e4
    uint16_t wlsTheta;           // objective segragation * 1e-4
    uint8_t wlsNanLimit;         // disarm because of repeating failures in wls. Keep low FIXME: make this fallback to pinv
} indiProfile_t;

typedef struct indiRuntime_s {
    // ---- INDI config
    t_fp_vector attGains;
    t_fp_vector rateGains;
    float attTiltRateLimit;
    float attYawRateLimit;
    float maxUpwardsSpfZManual; // -1 means sum(G1[2, :])
    float maxBankDegreeManual;
    float maxBankDegreeAuto;
    float omegaHover;
    float actTimeConst;
    uint8_t attRateDenom;
    bool attUseSpfzDiscounting;
    bool attCoordinateYaw;
    bool useIncrement;
    bool useOmegaFeedback;
    bool useOmegaDotFeedback;
    //float kThrust;
    //float tauRpm;
    float Tmax;
    float kLin;
    float motorOutputLimit;
    // ---- Filtering config

    // ---- WLS config
    activeSetAlgoChoice wlsAlgo;
    float u_pref[MAXU];
    bool useWls;
    bool wlsWarmstart;
    bool useConstantG2;
    float G2Normalizer; // 1/(2 * tauRpm * kThrust)
    int wlsMaxIter;
    float wlsCondBound;
    float wlsTheta;
    float wlsWv2[MAXV];
    float wlsWu2[MAXU];
    uint16_t wlsNanLimit;
    float wlsG1[MAXV][MAXU];
    float wlsG2[MAXV][MAXU];
} indiRuntime_t;

#define INDI_PROFILE_COUNT 3
PG_DECLARE_ARRAY(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles);
void resetIndiProfile(indiProfile_t *profile);

// linearization
typedef struct quadLin_s {
    float A;
    float B;
    float C;
    float k;
} quadLin_t;

typedef struct catapultConfig_s {
    uint16_t altitude;        // in cm
    int16_t xNed;             // in cm
    int16_t yNed;             // in cm
    uint16_t rotationRoll;    // in deg/s
    uint16_t rotationPitch;   // in deg/s
    uint16_t rotationYaw;     // in deg/s
    uint16_t rotationTimeMs;  // in ms
    uint8_t upwardsAccel;     // in m/s/s
} catapultConfig_t;

PG_DECLARE(catapultConfig_t, catapultConfig);

typedef struct catapultRuntime_s {
    float altitude;
    float xyNed[2];
    float rotationRate[3];
    timeDelta_t rotationTimeUs;
    timeDelta_t fireTimeUs;
    float upwardsAccel;
    fp_quaternion_t attitude;
} catapultRuntime_t;
extern catapultRuntime_t catapultRuntime;

void initCatapultRuntime(void);

#define MAX_BANK_DEGREE_MANUAL 40.f

#ifndef MAX_BANK_DEGREE_AUTO
#define MAX_BANK_DEGREE_AUTO 50.f
#endif

extern fp_quaternion_t attSpNed;
extern t_fp_vector rateSpBodyUse;
extern t_fp_vector alphaSpBody;
extern t_fp_vector spfSpBody;
extern float zAccSpNed;
extern float yawRateSpNed;
extern bool attTrackYaw;
extern float dv[MAXV];
extern float u[MAXU];
extern float u_state[MAXU];
extern float u_state_sync[MAXU];
extern float u_output[MAXU];
extern float omegaUnfiltered[MAXU];
extern float omega[MAXU];
extern float omega_dot[MAXU];
extern float alpha[XYZ_AXIS_COUNT];

void indiInit(const pidProfile_t * pidProfile);
void indiController(void);
float indiThrustLinearization(quadLin_t lin, float in);
float indiThrustCurve(quadLin_t lin, float in);

float getYawWithoutSingularity(void);
void getAttSpNedFromAccSpNed(t_fp_vector* accSpNed, fp_quaternion_t* attSpNed, float* fz);
t_fp_vector coordinatedYaw(float yaw);
void getSetpoints(void);
void getAlphaSpBody(void);
void getMotor(void);

typedef enum catapult_state_s {
    CATAPULT_DISABLED = -1,
    CATAPULT_WAITING_FOR_ARM = 0,
    CATAPULT_DELAY,
    CATAPULT_LAUNCHING,
    CATAPULT_ROTATING,
    CATAPULT_DONE,
} catapult_state_t;

extern catapult_state_t catapultState;

#ifdef USE_CATAPULT
void runCatapultStateMachine(float * spfSpBodyZ, t_fp_vector * rateSpBody);
void disableCatapult(void);
#endif

typedef enum learning_state_e {
    LEARNING_DISABLED = -1,
    LEARNING_DELAY = 0,
    LEARNING_ACTIVE,
    LEARNING_DONE
} learning_state_t;

extern learning_state_t learningState;

#ifdef USE_LEARN_AFTER_CATAPULT
typedef enum query_state_e {
    QUERY_ZERO = 0,
    QUERY_STEP,
    QUERY_RAMP,
    QUERY_DONE
} query_state_t;

typedef struct motor_stats_s {
    timeUs_t startTime;
    float minGyro[3];
    float maxGyro[3];
    query_state_t queryState;
} motor_state_t;


void runLearningStateMachine(void);
void disableLearning(void);
#endif
