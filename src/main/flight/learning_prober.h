/*
 * Generate sequence of input commands to learn UAV dynamics
 *
 * Copyright 2025 Till Blaha (Delft University of Technology)
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

#pragma once

#include "common/time.h"
#include "config/config.h"

// --- config
typedef struct proberConfig_s {
    uint8_t type;  // learning_prober_config_type_t
    // --- common parameters
    uint16_t actMask;
    uint16_t preDelayMs;  // delay before starting the prober
    uint16_t postDelayMs;  // delay after finishing the prober
    uint16_t gyroMax;  // stepramps: end an actuator's excitation early if the gyro leaves this range, deg/s. 0: off
    // --- specific parameters
    // steps
    uint16_t steps_stepMs;  // duration of each step in ms
    uint16_t steps_overlapMs;  // overlap between steps in ms
    uint8_t steps_amp; // amplitude of each step in percent (0-100)
    // stepramps
    uint16_t stepramps_stepMs;  // duration of each step in ms
    uint16_t stepramps_rampMs;  // duration of each ramp in ms
    uint16_t stepramps_overlapMs;  // overlap between steps in ms
    uint8_t stepramps_stepAmp; // amplitude of each step in percent (0 - 100)
    uint8_t stepramps_rampAmp; // amplitude of each ramp in percent (0 - 100)
    // multisine
    uint16_t multisine_fundamental;  // fundamental frequency in Hz
    uint8_t multisine_sinesPerActuator;  // number of sine waves per actuator
    uint8_t multisine_optimizePhase;  // randomize phase angles
    uint16_t multisine_timeMs; // total time for probing in 
    uint8_t multisine_amp; // amplitude of the sine waves in percent (0-100)
    // noise
    uint16_t noise_sampleTimeUs;  // sample time in us
    uint8_t noise_amp;  // amplitude of the noise in percent (0-100)
} proberConfig_t;

PG_DECLARE(proberConfig_t, proberConfig);

typedef enum {
    PROBER_STEPS,
    PROBER_STEPRAMPS,
//     PROBER_MULTISINE,
    PROBER_NOISE,
    PROBER_ORTHO,
} prober_type_t;

// for each of the types, make one runtime struct. then a union of those structs
typedef struct prober_runtime_steps_s {
    int currentStepIndex;  // index of the current step in the sequence
    timeUs_t nextStepTimeUs;  // when to apply the next step
} prober_runtime_steps_t;

typedef enum {
    PROBER_STEPRAMP_ZERO = 0,
    PROBER_STEPRAMP_STEP,
    PROBER_STEPRAMP_RAMP,
    PROBER_STEPRAMP_DONE
} prober_stepramp_state_t;

typedef struct prober_runtime_stepramps_s {
    int numSelected;            // number of motors excited
    int act[MAXU];              // actuator index of the k-th excited motor
    timeUs_t startTimeUs[MAXU];
    float minGyro[MAXU][3];     // deg/s
    float maxGyro[MAXU][3];
    prober_stepramp_state_t state[MAXU];
} prober_runtime_stepramps_t;

typedef struct prober_runtime_multisine_s {
    float motorPhases[MAX_SUPPORTED_MOTORS];  // current phase angle of the sine wave
    float servoPhases[MAX_SUPPORTED_SERVOS];
} prober_runtime_multisine_t;

typedef struct prober_runtime_noise_s {
    timeUs_t nextSampleTimeUs;  // when to apply the next noise sample
    float currentNoiseValue;  // current noise value
} prober_runtime_noise_t;

typedef struct prober_runtime_s {
    prober_type_t type;
    int numActuators; // number of actuators (motors + servos)
    timeUs_t initTimeUs;      // when init was called
    timeUs_t safetyTimeoutUs; // when to stop probing even if not finished
    timeUs_t genStartTimeUs;     // first call to actual probe generator
    timeUs_t genFinishTimeUs;    // last call to actual probe generator
    // state variables
    bool isInitialized;
    bool isGenRunning;
    bool isGenFinished;
    bool isFinished;
    // union of type-specific runtime data
    union {
        prober_runtime_steps_t steps;
        prober_runtime_stepramps_t stepramps;
        prober_runtime_multisine_t multisine;
        prober_runtime_noise_t noise;
    } sub;
    float output[MAXU];  // output values for each motor during probing
    // servo
    bool isInitializedServo;
    bool isGenRunningServo;
    bool isGenFinishedServo;
    timeUs_t lastStepTimeUs;
} prober_runtime_t;

extern prober_runtime_t proberRuntime;

void initProber(timeUs_t currentTimeUs);
void resetProber(void);
void updateProber(timeUs_t currentTimeUs);
void initServoProber(timeUs_t currentTimeUs);
void updateServoProber(timeUs_t currentTimeUs);

void initProberDebugPin(void);
