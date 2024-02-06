
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