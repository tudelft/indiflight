/*
 * Machine parameters for the f2c'd CLAPACK (slamch_, dlamch_), replacing
 * lib/main/clapack/INSTALL/{s,d}lamch.c.
 *
 * The f2c versions find the parameters at run time with floating point loops
 * (slamc1_/slamc2_) that rely on exact IEEE semantics. With -ffast-math (the
 * MOCKUP build uses -Ofast) these loops never terminate, so the first QR
 * factorization in the learner (sgeqp3_ in calculateHoverAttitude) hung the
 * simulation. This uses <float.h> instead, like LAPACK 3.2 and later.
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <float.h>

#include "f2c.h"

// LAPACK assumes rounding (rnd = 1), so eps is half the spacing at 1.
// No floating point literals: the ARM build uses -fsingle-precision-constant
#define LAMCH_BODY(EPSILON, TINY, HUGE, RADIX, MANT_DIG, MIN_EXP, MAX_EXP) \
    const double eps = (double) (EPSILON) / 2;                                 \
    double sfmin = (double) (TINY);                                              \
    const double small = 1 / (double) (HUGE);                                   \
    if (small >= sfmin) {                                               \
        /* use small plus a bit, to avoid overflow when computing 1/sfmin */ \
        sfmin = small * ((double) 1 + eps);                                     \
    }                                                                   \
    switch (cmach[0]) {                                                 \
        case 'E': case 'e': return eps;                                 \
        case 'S': case 's': return sfmin;                               \
        case 'B': case 'b': return (RADIX);                             \
        case 'P': case 'p': return eps * (double) (RADIX);                       \
        case 'N': case 'n': return (MANT_DIG);                          \
        case 'R': case 'r': return 1;                                  \
        case 'M': case 'm': return (MIN_EXP);                           \
        case 'U': case 'u': return (double) (TINY);                              \
        case 'L': case 'l': return (MAX_EXP);                           \
        case 'O': case 'o': return (double) (HUGE);                              \
        default: return 0;                                             \
    }

doublereal slamch_(char *cmach)
{
    LAMCH_BODY(FLT_EPSILON, FLT_MIN, FLT_MAX, FLT_RADIX, FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP)
}

doublereal dlamch_(char *cmach)
{
    LAMCH_BODY(DBL_EPSILON, DBL_MIN, DBL_MAX, FLT_RADIX, DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP)
}

// a + b, kept for callers that use it to defeat extended precision registers
doublereal slamc3_(real *a, real *b)
{
    return *a + *b;
}

doublereal dlamc3_(doublereal *a, doublereal *b)
{
    return *a + *b;
}
