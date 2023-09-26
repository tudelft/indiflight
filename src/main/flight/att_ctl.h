
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

#define MAX_BANK_DEGREE 50.f
#define ATT_MAX_RATE_XY 500.f
#define ATT_MAX_RATE_Z 150.f

extern fp_quaternion_t attSpNed;
extern t_fp_vector rateSpBody;
extern t_fp_vector alphaSpBody;
extern t_fp_vector spfSpBody;
extern float zAccSpNed;
extern float yawRateSpNed;
extern bool attTrackYaw;
extern float dv[MAXV];
extern float u[MAXU];
extern float u_state[MAXU];
extern float u_output[MAXU];
extern float omega[MAXU];
extern float omega_dot[MAXU];
extern float alpha[XYZ_AXIS_COUNT];

void indiInit(const pidProfile_t * pidProfile);
void indiController(void);
float indiThrustCurve(quadLin_t lin, float in);
float indiThrustLinearization(quadLin_t lin, float in);

void getYawRateSpBody(void);
void getAttErrBody(void);
void getAlphaBody(void);
void getSpfSpBodyZ(void);
void getMotor(void);
