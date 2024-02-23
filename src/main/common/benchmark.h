
#include <stdint.h>
#include "platform.h"
#include <math.h>

#pragma once

#define BENCH_REPEAT 100
#define BENCH_N 4 // size of the matrices, max 10

#if (BENCH_N*BENCH_N > BENCH_REPEAT)
    #error "BENCH_N*BENCH_N must be <= BENCH_REPEAT"
#endif

#define STATIC_INLINE __attribute__((always_inline)) static inline

#define BENCH_PREPARE_SCALAR_MULT(_type, _typename) \
    volatile _type _typename##a = 23; \
    volatile _type _typename##b = 5; \
    volatile _type _typename##c = 0; \
    (void)(_typename##c);

STATIC_INLINE void scalar_mult_i16(const int16_t* a, int16_t* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] * a[n - i - 1];
}

STATIC_INLINE void scalar_mult_i32(const int32_t* a, int32_t* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] * a[n - i - 1];
}

STATIC_INLINE void scalar_mult_f32(const float* a, float* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] * a[n - i - 1];
}

STATIC_INLINE void scalar_mult_f64(const double* a, double* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] * a[n - i - 1];
}

STATIC_INLINE void scalar_div_f32(const float* a, float* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] / a[n - i - 1];
}

STATIC_INLINE void scalar_div_f64(const double* a, double* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = a[i] / a[n - i - 1];
}

STATIC_INLINE void scalar_sqrt_f32(const float* a, float* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = sqrtf(a[i]);
}

STATIC_INLINE void scalar_sqrt_f64(const double* a, double* b, int n) {
    for (int i = 0; i < n; i++)
        b[i] = sqrt(a[i]);
}

typedef struct dtype_counts_s {
    uint32_t i16;
    uint32_t i32;
    uint32_t f32;
    uint32_t f64;
} dtype_counters_t;

enum {
    BENCH_TEST_SCALAR_MULT = 0,
    BENCH_TEST_SCALAR_DIV,
    BENCH_TEST_SCALAR_SQRT,
    BENCH_TEST_MATRIX_MULT,
    BENCH_TEST_QR_DECOMP,
    BENCH_TEST_COUNT,
};

enum {
    BENCH_DRIVER_NAIVE = 0,
    BENCH_DRIVER_HANDWRITTEN_4,
    BENCH_DRIVER_HANDWRITTEN_8,
    BENCH_DRIVER_HANDWRITTEN_16,
    BENCH_DRIVER_CMSYS,
    BENCH_DRIVER_OPENBLAS,
    BENCH_DRIVER_COUNT,
};

extern dtype_counters_t benchCounters[BENCH_TEST_COUNT][BENCH_DRIVER_COUNT];

void benchmark_harness(void);
