#ifndef DRONE_MODEL_H
#define DRONE_MODEL_H

void compute_modeled_acceleration(float vbx, float vby, float vbz,
                          float omega0, float omega1, float omega2, float omega3,
                          float p, float q, float r
);

void get_modeled_acceleration(float* ax, float* ay, float* az);

#endif // DRONE_MODEL_H