
#include "common/maths.h"
#include <stdbool.h>

#include "platform.h"
#include "flight/pid.h"
#include "common/maths.h"
#include "solveActiveSet.h"

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
    uint16_t attMaxTiltRate; // max tilt rate in deg/s
    uint16_t attMaxYawRate; // max yaw rate in deg/s
    uint8_t manualUseCoordinatedYaw;     // bool: coordinate yaw in manual flight in Angle/Horizon mode
    uint8_t manualMaxUpwardsSpf; // maximum upwards specific force in manual flight in N/kg. 255 means max of the platform
    uint8_t manualMaxTilt; // in manual flight in deg (0, 180)
    // ---- general INDI config
    uint8_t useIncrement;          // bool: use incremental law. NDI (or more precisely linDI) otherwise
    uint8_t useRpmDotFeedback;     // bool: make use of dshot rpm derivative data in feedback loop if available
    // ---- INDI actuator config
    uint16_t actHoverRpm[MAXU];     // approximate rpm/10 in hover flight. FIXME: make an estimator for this
    uint8_t actTimeConstMs[MAXU];     // time constant for actuator spool up in ms
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

typedef struct indiRuntime_s {
    // ---- Att/Rate config
    t_fp_vector attGains;
    t_fp_vector rateGains;
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
    // ---- INDI actuator config
    float actHoverOmega[MAXU];   // rad/s
    float actTimeConstS[MAXU];   // sec
    float actMaxT[MAXU];         // N
    float actNonlinearity[MAXU]; // - 
    float actLimit[MAXU];        // 
    float actG1[MAXV][MAXU];
    float actG2[3][MAXU];
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
} indiRuntime_t;

#define INDI_PROFILE_COUNT 3
PG_DECLARE_ARRAY(indiProfile_t, INDI_PROFILE_COUNT, indiProfiles);
void resetIndiProfile(indiProfile_t *profile);

extern indiRuntime_t indiRuntime;
void initIndiRuntime(void);

void changeIndiProfile(uint8_t profileIndex);

// linearization
typedef struct quadLin_s {
    float A;
    float B;
    float C;
    float k;
} quadLin_t;

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
void indiController(timeUs_t current);
float indiThrustLinearization(quadLin_t lin, float in);
float indiThrustCurve(quadLin_t lin, float in);

float getYawWithoutSingularity(void);
t_fp_vector coordinatedYaw(float yaw);
void getSetpoints(timeUs_t current);
void getAlphaSpBody(timeUs_t current);
void getMotor(timeUs_t current);

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
