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
// Updated tau axis (independent variable)
static const float tauAxis1D[] = {
    0.0100, 0.0124, 0.0148, 0.0172, 0.0197, 0.0221, 0.0245, 0.0269, 0.0293, 0.0317,
    0.0341, 0.0366, 0.0390, 0.0414, 0.0438, 0.0462, 0.0486, 0.0510, 0.0534, 0.0559,
    0.0583, 0.0607, 0.0631, 0.0655, 0.0679, 0.0703, 0.0728, 0.0752, 0.0776, 0.0800
};

// Updated kEta gains
static const float kEtaGains1D[] = {
    11.2813, 10.3256, 9.4252, 8.6072, 7.9217, 7.3442, 6.8523, 6.4245, 6.0486, 5.7154,
    5.4176, 5.1507, 4.9086, 4.6890, 4.4870, 4.3040, 4.1343, 3.9821, 3.8419, 3.7115,
    3.5888, 3.4738, 3.3658, 3.2641, 3.1678, 3.0768, 2.9914, 2.9099, 2.8331, 2.7602
};
const GainSchedule kEta1D = {tauAxis1D, 30, NULL, 0, kEtaGains1D};

// Updated kOmega gains
static const float kOmegaGains1D[] = {
    21.9912, 21.1220, 20.3892, 19.5539, 18.6463, 17.7433, 16.8781, 16.0713, 15.3218, 14.6292,
    13.9881, 13.3950, 12.8481, 12.3391, 11.8711, 11.4311, 11.0222, 10.6336, 10.2661, 9.9247,
    9.6046, 9.3049, 9.0239, 8.7590, 8.5092, 8.2729, 8.0509, 7.8391, 7.6392, 7.4495
};
const GainSchedule kOmega1D = {tauAxis1D, 30, NULL, 0, kOmegaGains1D};

// Updated ffPole (pFf) gains
static const float ffPoleGains1D[] = {
    -8.7723, -8.8351, -8.9011, -9.0603, -9.3615, -9.7481, -10.0061, -10.0710, -9.9689, -9.7649,
    -9.5024, -9.2087, -8.9137, -8.6072, -8.3238, -8.0365, -7.7663, -7.4870, -7.2162, -6.9629,
    -6.7278, -6.5092, -6.3034, -6.1096, -5.9288, -5.7584, -5.5980, -5.4449, -5.3004, -5.1638
};
const GainSchedule ffPole1D = {tauAxis1D, 30, NULL, 0, ffPoleGains1D};

// Updated ffK (kFf) gains
static const float ffKGains1D[] = {
    0.9978, 1.0234, 1.0597, 1.1013, 1.1443, 1.1898, 1.2318, 1.2665, 1.2924, 1.3117,
    1.3257, 1.3352, 1.3431, 1.3475, 1.3523, 1.3538, 1.3556, 1.3519, 1.3470, 1.3416,
    1.3373, 1.3341, 1.3305, 1.3270, 1.3245, 1.3225, 1.3204, 1.3181, 1.3160, 1.3142
};
const GainSchedule ffK1D = {tauAxis1D, 30, NULL, 0, ffKGains1D};



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