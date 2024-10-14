#include "drone_model.h"

float ax_modeled = 0;
float ay_modeled = 0;
float az_modeled = 0;

void compute_modeled_acceleration(float vbx, float vby, float vbz,
                          float omega0, float omega1, float omega2, float omega3,
                          float p, float q, float r) {
    // let compiler know that we dont use p,q,r,vbz
    (void) p;
    (void) q;
    (void) r;
    (void) vbz;
    // estimated parameters based on data from dummy drone
    // 'k_w': -1.94e-06, 'k_x': -5.67e-05, 'k_y': -6.59e-05,
    static float k_x = -5.67e-05;
    static float k_y = -6.59e-05;
    static float k_omega = -1.94e-06;

    ax_modeled = k_x*vbx*(omega0+omega1+omega2+omega3);
    ay_modeled = k_y*vby*(omega0+omega1+omega2+omega3);
    az_modeled = k_omega*(omega0*omega0+omega1*omega1+omega2*omega2+omega3*omega3);
}
void get_modeled_acceleration(float* ax, float* ay, float* az) {
    *ax = ax_modeled;
    *ay = ay_modeled;
    *az = az_modeled;
}