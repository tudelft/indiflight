// Stand-in for the generated lib/main/ekf_c_code_generation/build/ekf_calc.h,
// so blackbox_header_unittest builds without that submodule. Only what
// blackbox.c and flight/ekf.h use.

#pragma once

#define N_STATES 16
#define N_INPUTS 6
#define N_PROC_NOISES 12
#define N_MEASUREMENTS 7

float *ekf_get_X(void);
