#include "signal_generator.h"
#include <math.h>

int current_signal_mode = SIGNAL_MODE_OFF;
static float signal_amplitude = 0.0f;
static float signal_period = 0.0f;
static timeUs_t signal_start_time = 0;

void set_signal_mode(signal_mode_t mode, timeUs_t currentTimeUs) {
    current_signal_mode = mode;
    signal_start_time = currentTimeUs;

    // Set specific parameters for each mode
    switch (mode) {
        case SIGNAL_MODE_IMPULSE:
            signal_amplitude = M_PI / 36.0f;  // 5 degrees
            signal_period = 0.1f;  // 0.1 seconds
            break;
        case SIGNAL_MODE_DOUBLET:
            signal_amplitude = M_PI / 36.0f;  // 5 degrees
            signal_period = 1.0f;  // 1 second
            break;
        case SIGNAL_MODE_OFF:
        default:
            signal_amplitude = 0.0f;
            signal_period = 0.0f;
            break;
    }
}

float generate_signal(timeUs_t currentTimeUs) {
    if (current_signal_mode == SIGNAL_MODE_OFF) {
        return 0.0f;
    }

    float elapsedTime = (currentTimeUs - signal_start_time) * 1e-6f; // Convert to seconds

    if (current_signal_mode == SIGNAL_MODE_IMPULSE) {
        // Impulse signal: a single pulse at the start
        return (elapsedTime < signal_period) ? signal_amplitude : 0.0f;
    }

    if (current_signal_mode == SIGNAL_MODE_DOUBLET) {
        // Doublet signal
        float cycleTime = fmod(elapsedTime, signal_period); // Get time within current cycle
        if (cycleTime < signal_period / 4) {
            return signal_amplitude; // Positive pulse
        } else if (cycleTime < signal_period / 2) {
            return -signal_amplitude; // Negative pulse
        } else {
            return 0.0f; // No pulse
        }
    }

    return 0.0f; // Default to no signal
}



fp_quaternion_t create_step_quaternion(float angle, char axis) {
    fp_quaternion_t quat = QUATERNION_INITIALIZE;
    
    float half_angle = angle / 2.0f;
    float sin_half_angle = sin_approx(half_angle);
    
    switch (axis) {
        case 'x':
            quat.w = cos_approx(half_angle);
            quat.x = sin_half_angle;
            quat.y = 0.0f;
            quat.z = 0.0f;
            break;
        case 'y':
            quat.w = cos_approx(half_angle);
            quat.x = 0.0f;
            quat.y = sin_half_angle;
            quat.z = 0.0f;
            break;
        case 'z':
            quat.w = cos_approx(half_angle);
            quat.x = 0.0f;
            quat.y = 0.0f;
            quat.z = sin_half_angle;
            break;
        default:
            // Handle the error case where an invalid axis is passed.
            break;
    }

    return quat;
}

// Helper function to normalize a quaternion
void normalize_quaternion(fp_quaternion_t* q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

// Helper function to multiply two quaternions
fp_quaternion_t multiply_quaternions(const fp_quaternion_t* q1, const fp_quaternion_t* q2) {
    fp_quaternion_t result;
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    return result;
}


