#include <stdio.h>
#include "gain_schedule.h"

// 1D Polynomials
// static const float kEtaCoeffs1D[] = {2125, -297.4, 13.31};
static const float kEtaCoeffs1D[] = {12.8, -277.6, 1942.1};
const Poly1D kEtaPoly1D = {kEtaCoeffs1D, 2};

// static const float kOmegaCoeffs1D[] = {2427, -418.3, 25.47}; 
static const float kOmegaCoeffs1D[] = {25.7, -426.8, 2505.3};
const Poly1D kOmegaPoly1D = {kOmegaCoeffs1D, 2};

// static const float ffKCoeffs1D[] = {1811, -129.1, 3.952};  
static const float ffKCoeffs1D[] = {3.952, -129.1, 1811};
const Poly1D ffKPoly1D = {ffKCoeffs1D, 2};

// static const float ffPoleCoeffs1D[] = {9819, -757.2, 25.83};
static const float ffPoleCoeffs1D[] = {25.83, -757.2, 9819};
const Poly1D ffPolePoly1D = {ffPoleCoeffs1D, 2};

// 2D Polynomials (NOT DONE YET)
static const float kEtaCoeffs2D[] = {0.5, 0.1, 0.2, 0.05}; // 0.5 + 0.1x + 0.2y + 0.05xy
const Poly2D kEtaPoly2D = {kEtaCoeffs2D, 1, 1};

static const float kOmegaCoeffs2D[] = {1.0, 0.05, 0.03, 0.01}; // 1.0 + 0.05x + 0.03y + 0.01xy
const Poly2D kOmegaPoly2D = {kOmegaCoeffs2D, 1, 1};

static const float ffKCoeffs2D[] = {2.0, 0.3, -0.1, 0.02}; // 2.0 + 0.3x - 0.1y + 0.02xy
const Poly2D ffKPoly2D = {ffKCoeffs2D, 1, 1};

static const float ffPoleCoeffs2D[] = {10.0, -0.5, 0.2, -0.01}; // 10.0 - 0.5x + 0.2y - 0.01xy
const Poly2D ffPolePoly2D = {ffPoleCoeffs2D, 1, 1};


// // 1D Gain interpolation
// Replace tauAxis1D and all gain arrays with your new values:
static const float tauAxis1D[] = {
    0.0100, 0.0118, 0.0136, 0.0154, 0.0172, 0.0190, 0.0208, 0.0226, 0.0244, 0.0262,
    0.0279, 0.0297, 0.0315, 0.0333, 0.0351, 0.0369, 0.0387, 0.0405, 0.0423, 0.0441,
    0.0459, 0.0477, 0.0495, 0.0513, 0.0531, 0.0549, 0.0567, 0.0585, 0.0603, 0.0621,
    0.0638, 0.0656, 0.0674, 0.0692, 0.0710, 0.0728, 0.0746, 0.0764, 0.0782, 0.0800
};

static const float ffPoleGains1D[] = {
    -8.3559, -8.7817, -8.8356, -8.9186, -9.0507, -9.2595, -9.5368, -9.8384, -9.9942, -10.0798,
    -10.0645, -9.9412, -9.7790, -9.5905, -9.3502, -9.1569, -8.9626, -8.7166, -8.2506, -8.2339,
    -8.0707, -7.8682, -7.6724, -7.4569, -7.2543, -7.1018, -6.8848, -6.7875, -6.5450, -6.4022,
    -6.2401, -6.0995, -5.9630, -5.8334, -5.7101, -5.5906, -5.4770, -5.4100, -5.2614, -5.1611
};
const GainSchedule ffPole1D = {tauAxis1D, 40, NULL, 0, ffPoleGains1D};

static const float ffKGains1D[] = {
    1.0020, 1.0156, 1.0394, 1.0695, 1.1006, 1.1320, 1.1654, 1.2022, 1.2293, 1.2568,
    1.2819, 1.2972, 1.3105, 1.3216, 1.3257, 1.3367, 1.3456, 1.3469, 1.3411, 1.3506,
    1.3539, 1.3555, 1.3557, 1.3514, 1.3477, 1.3458, 1.3402, 1.3412, 1.3345, 1.3328,
    1.3293, 1.3272, 1.3250, 1.3232, 1.3219, 1.3203, 1.3185, 1.3196, 1.3156, 1.3144
};
const GainSchedule ffK1D = {tauAxis1D, 40, NULL, 0, ffKGains1D};

static const float kEtaGains1D[] = {
    11.2283, 10.5637, 9.8886, 9.2239, 8.6245, 8.1030, 7.6422, 7.2278, 6.8775, 6.5506,
    6.2484, 5.9848, 5.7398, 5.5137, 5.3102, 5.1113, 4.9288, 4.7653, 4.6094, 4.4627,
    4.3264, 4.1981, 4.0772, 3.9677, 3.8636, 3.7644, 3.6699, 3.5800, 3.4942, 3.4121,
    3.3339, 3.2587, 3.1870, 3.1185, 3.0527, 2.9894, 2.9286, 2.8702, 2.8138, 2.7598
};
const GainSchedule kEta1D = {tauAxis1D, 40, NULL, 0, kEtaGains1D};

static const float kOmegaGains1D[] = {
    22.0160, 21.3117, 20.7699, 20.2099, 19.5787, 18.9042, 18.2270, 17.5723, 16.9213, 16.3142,
    15.7429, 15.1945, 14.6800, 14.1953, 13.7286, 13.3068, 12.9090, 12.5187, 12.1549, 11.8120,
    11.4858, 11.1768, 10.8827, 10.5953, 10.3218, 10.0624, 9.8156, 9.5814, 9.3579, 9.1444,
    8.9408, 8.7454, 8.5593, 8.3809, 8.2099, 8.0455, 7.8875, 7.7356, 7.5892, 7.4487
};
const GainSchedule kOmega1D = {tauAxis1D, 40, NULL, 0, kOmegaGains1D};

// // Tau axis (independent variable)
// static const float tauAxis1D[] = {0.0100, 0.0137, 0.0174, 0.0211, 0.0247, 0.0284, 0.0321, 0.0358, 0.0395, 0.0432, 0.0468, 0.0505, 0.0542, 0.0579, 0.0616, 0.0653, 0.0689, 0.0726, 0.0763, 0.0800};

// // K_eta gains
// static const float kEtaGains1D[] = {11.2290, 9.8429, 8.5680, 7.5807, 6.8054, 6.1790, 5.6648, 5.2292, 4.8604, 4.5388, 4.2581, 4.0129, 3.8002, 3.6077, 3.4333, 3.2741, 3.1288, 2.9959, 2.8732, 2.7598};
// const GainSchedule kEta1D = {tauAxis1D, 20, NULL, 0, kEtaGains1D};

// // K_Omega gains
// static const float kOmegaGains1D[] = {22.0141, 20.7435, 19.5069, 18.1144, 16.7913, 15.5944, 14.5235, 13.5817, 12.7382, 11.9897, 11.3215, 10.7143, 10.1565, 9.6536, 9.1995, 8.7857, 8.4081, 8.0623, 7.7435, 7.4487};
// const GainSchedule kOmega1D = {tauAxis1D, 20, NULL, 0, kOmegaGains1D};

// // FfK gains
// // static const float ffKGains1D[] = {1.4257, 1.1786, 0.9499, 0.8178, 0.7270, 0.6547, 0.5998, 0.5513, 0.5120, 0.4784, 0.4493, 0.4222, 0.4000, 0.3800, 0.3619, 0.3445, 0.3299, 0.3157, 0.3027, 0.2909};
// static const float ffKGains1D[] = {
//     0.9976, 1.0486, 1.1238, 1.1856, 1.2414, 1.3065, 1.3337, 1.3328, 1.3436, 1.3523,
//     1.3547, 1.3547, 1.3452, 1.3384, 1.3335, 1.3280, 1.3240, 1.3208, 1.3185, 1.3150
// };
// const GainSchedule ffK1D = {tauAxis1D, 20, NULL, 0, ffKGains1D};

// // FfP gains
// // static const float ffPoleGains1D[] = {21.3741, 15.7076, 10.2703, 7.8107, 6.4457, 5.5352, 4.9430, 4.3996, 4.0140, 3.7101, 3.5083, 3.2405, 3.0345, 2.8568, 2.7215, 2.5710, 2.4727, 2.3579, 2.2507, 2.1492};
// static const float ffPoleGains1D[] = {
//     10.8998, 10.9410, 10.4700, 9.7684, 8.4949, 7.5988, 6.7349, 6.0434, 5.4834, 5.0136
// };
// static const float tauAxis1DPole[] ={
//     0.0100, 0.0178, 0.0256, 0.0333, 0.0411, 0.0489, 0.0567, 0.0644, 0.0722, 0.0800
// }; // Accidentally used too little, using this only for testing
// const GainSchedule ffPole1D = {tauAxis1DPole, 20, NULL, 0, ffPoleGains1D};

// const float ffZeroGain1D = -5.0113; // constant zero for feedforward zero



// 2D Gain interpolation (NOT DONE YET)
static const float kEtaXAxis2D[] = {0, 2, 4};
static const float kEtaYAxis2D[] = {0, 1};
static const float kEtaGains2D[] = {0.3, 0.5, 0.7,   0.4, 0.6, 0.8}; // row-major
const GainSchedule kEta2D = {kEtaXAxis2D, 3, kEtaYAxis2D, 2, kEtaGains2D};

static const float kOmegaXAxis2D[] = {0, 3, 6};
static const float kOmegaYAxis2D[] = {0, 2};
static const float kOmegaGains2D[] = {0.6, 1.0, 1.4,   0.8, 1.2, 1.6}; // row-major
const GainSchedule kOmega2D = {kOmegaXAxis2D, 3, kOmegaYAxis2D, 2, kOmegaGains2D};

static const float ffKXAxis2D[] = {0, 1, 2};
static const float ffKYAxis2D[] = {0, 3};
static const float ffKGains2D[] = {1.2, 1.8, 2.4,   1.5, 2.1, 2.7}; // row-major
const GainSchedule ffK2D = {ffKXAxis2D, 3, ffKYAxis2D, 2, ffKGains2D};

static const float ffPoleXAxis2D[] = {0, 2, 4};
static const float ffPoleYAxis2D[] = {0, 1};
static const float ffPoleGains2D[] = {7.0, 9.0, 11.0,   8.0, 10.0, 12.0}; // row-major
const GainSchedule ffPole2D = {ffPoleXAxis2D, 3, ffPoleYAxis2D, 2, ffPoleGains2D};

float lerp(float x0, float x1, float y0, float y1, float x)
{
    if (x1 == x0) return y0;
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

// Function for 1D gain scheduling
float interpolate1D(const float *xAxis, const float *gains, int size, float x)
{
    // Clamp gains. TODO: make bounds a limit / give error?
    if (x <= xAxis[0]) return gains[0];
    if (x >= xAxis[size-1]) return gains[size-1];

    // Find interval
    int i;
    for (i = 0; i < size-1; i++) {
        if (xAxis[i] <= x && x <= xAxis[i+1]) {
            return lerp(xAxis[i], xAxis[i+1], gains[i], gains[i+1], x);
        }
    }
    return gains[size-1]; // should not reach here
}

// 2D gain scheduling (bilinear interpolation)
float interpolate2D(const float *xAxis, int xSize,
                     const float *yAxis, int ySize,
                     const float **gainTable,
                     float x, float y)
{
    // Clamp gains. TODO: make bounds a limit / give error?
    if (x <= xAxis[0]) x = xAxis[0];
    if (x >= xAxis[xSize-1]) x = xAxis[xSize-1];
    if (y <= yAxis[0]) y = yAxis[0];
    if (y >= yAxis[ySize-1]) y = yAxis[ySize-1];

    // Find x index
    int i;
    for (i = 0; i < xSize-1; i++) {
        if (xAxis[i] <= x && x <= xAxis[i+1]) break;
    }

    // Find y index
    int j;
    for (j = 0; j < ySize-1; j++) {
        if (yAxis[j] <= y && y <= yAxis[j+1]) break;
    }

    // Corners of cell
    float x0 = xAxis[i];
    float x1 = xAxis[i+1];
    float y0 = yAxis[j];
    float y1 = yAxis[j+1];

    float Q11 = gainTable[j][i];
    float Q21 = gainTable[j][i+1];
    float Q12 = gainTable[j+1][i];
    float Q22 = gainTable[j+1][i+1];

    // Bilinear interpolation
    float R1 = lerp(x0, x1, Q11, Q21, x);
    float R2 = lerp(x0, x1, Q12, Q22, x);
    return lerp(y0, y1, R1, R2, y);
}


float evalPoly1D(const Poly1D *p, float x)
{
    float result = p->coeffs[p->order];

    // Horner's method for polynomial evaluation
    for (int i = p->order - 1; i >= 0; i--) {
        result = result * x + p->coeffs[i];
    }
    return result;
}

float evalPoly2D(const Poly2D *p, float x, float y)
{
    float result = 0.0;
    for (int j = 0; j <= p->yOrder; j++) {
        // polynomial in x for each power of y
        float temp = p->coeffs[j * (p->xOrder + 1) + p->xOrder];
        for (int i = p->xOrder - 1; i >= 0; i--) {
            temp = temp * x + p->coeffs[j * (p->xOrder + 1) + i];
        }
        // accumulate with y^j
        float yPow = 1.0;
        for (int k = 0; k < j; k++) yPow *= y;
        result += temp * yPow;
    }
    return result;
}