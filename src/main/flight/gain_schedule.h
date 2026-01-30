

//// Definition of structures for gainscheduling
// Structure definition for GainSchedule
typedef struct {
    const float *xAxis;
    int xSize;
    const float *yAxis;
    int ySize;
    const float *table;   // gain table stored row-major (y-major)
} GainSchedule;

// Polynomial structures
typedef struct {
    const float *coeffs; // [a0, a1, ..., an]
    int order;
} Poly1D;

typedef struct {
    const float *coeffs; // flattened row-major: coeff[j*(xOrder+1) + i]
    int xOrder;
    int yOrder;
} Poly2D;

float lerp(float x0, float x1, float y0, float y1, float x);
float interpolate1D(const float *xAxis, const float *gains, int size, float x);
float interpolate2D(const float *xAxis, int xSize, const float *yAxis, int ySize, 
                     const float **gainTable, float x, float y);

float evalPoly1D(const Poly1D *p, float x);
float evalPoly2D(const Poly2D *p, float x, float y);

extern const Poly1D kEtaPoly1D;
extern const Poly1D kOmegaPoly1D;
extern const Poly1D ffKPoly1D;
extern const Poly1D ffPolePoly1D;

extern const Poly2D kEtaPoly2D;
extern const Poly2D kOmegaPoly2D;
extern const Poly2D ffKPoly2D;
extern const Poly2D ffPolePoly2D;

extern const GainSchedule kEta1D;
extern const GainSchedule kOmega1D;
extern const GainSchedule ffK1D;
extern const GainSchedule ffPole1D;
extern const float ffZeroGain1D;

extern const GainSchedule kEta2D;
extern const GainSchedule kOmega2D;
extern const GainSchedule ffK2D;
extern const GainSchedule ffPole2D;

typedef enum {
    GAIN_SCHEDULE_1D_INTERP = 0,    // 1D interpolation (table lookup)
    GAIN_SCHEDULE_2D_INTERP = 1,    // 2D interpolation (table lookup)
    GAIN_SCHEDULE_1D_POLY = 2,      // 1D polynomial
    GAIN_SCHEDULE_2D_POLY = 3       // 2D polynomial
} GainScheduleType;
