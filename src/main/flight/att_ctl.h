
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

typedef struct indiConfig_s {
    // ---- INDI config
    t_fp_vector attGains;
    t_fp_vector rateGains;
    float maxNegativeSpfZ;
    float maxBankDegree;
    float omegaHover;
    float actTimeConst;
    bool useIncrement;
    bool useOmegaFeedback;
    bool useOmegaDotFeedback;
    bool useConstantG2;
    float G2Normalizer; // 1/(2 * tauRpm * kThrust)
    //float kThrust;
    //float tauRpm;
    float Tmax;
    // ---- WLS config
    activeSetAlgoChoice wlsAlgo;
    bool useWls;
    bool wlsWarmstart;
    int wlsMaxIter;
    float wlsCondBound;
    float wlsTheta;
    float wlsWv2[MAXV];
    float wlsWu2[MAXU];
    float u_pref[MAXU];
    float wlsG1[MAXV][MAXU];
    float wlsG2[MAXV][MAXU];
    float Ginv[MAXU][MAXV];
} indiConfig_t;

PG_DECLARE(indiConfig_t, indiConfig);

// linearization
typedef struct quadLin_s {
    float A;
    float B;
    float C;
    float k;
} quadLin_t;

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

void runCatapultStateMachine(float * spfSpBodyZ, t_fp_vector * rateSpBody);
void disableCatapult();
