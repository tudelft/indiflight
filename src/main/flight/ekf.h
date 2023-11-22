
#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#define N_STATES 15
#define N_INPUTS 6
#define N_MEASUREMENTS 6

void ekf_init(float X0[N_STATES], float P_diag0[N_STATES]);
float* ekf_get_state(void);
void ekf_get_Pdiag(float P_diag[N_STATES]);
void ekf_predict(float U[N_INPUTS], float dt);
void ekf_update(float Z[N_MEASUREMENTS]);

#endif
