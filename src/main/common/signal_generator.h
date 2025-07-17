// #ifndef SIGNAL_GENERATOR_H
// #define SIGNAL_GENERATOR_H

#include <stdbool.h>
#include "platform.h"
#include "time.h"
#include "maths.h"

#include "common/time.h" // Assuming you use this for time management

extern int current_signal_mode;

typedef enum {
    SIGNAL_MODE_OFF,
    SIGNAL_MODE_IMPULSE,
    SIGNAL_MODE_DOUBLET,
    SIGNAL_MODE_STEP
} signal_mode_t;

// Function to initialize the signal mode
void set_signal_mode(signal_mode_t mode, float signalAmplitude, float signalPeriod, timeUs_t currentTimeUs);

// Function to generate the signal for the current time
float generate_signal(timeUs_t currentTimeUs, timeUs_t signalStartTime);

fp_quaternion_t create_step_quaternion(float angle, char axis);
void normalize_quaternion(fp_quaternion_t* q);
fp_quaternion_t multiply_quaternions(const fp_quaternion_t* q1, const fp_quaternion_t* q2);
