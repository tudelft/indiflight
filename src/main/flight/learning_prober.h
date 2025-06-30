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
    uint8_t numMotors;
    uint8_t numServos;
    uint16_t preDelayMs;  // delay before starting the prober
    uint16_t postDelayMs;  // delay after finishing the prober
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
    PROBER_MULTISINE,
    PROBER_NOISE,
} prober_type_t;

// for each of the types, make one runtime struct. then a union of those structs
typedef struct prober_runtime_steps_s {
    int currentStepIndex;  // index of the current step in the sequence
    timeUs_t nextStepTimeUs;  // when to apply the next step
} prober_runtime_steps_t;

typedef struct prober_runtime_stepramps_s {
    int currentStepIndex;  // index of the current step in the sequence
    timeUs_t nextStepTimeUs;  // when to apply the next step
    timeUs_t nextRampTimeUs;  // when to apply the next ramp
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
    float motorOutput[MAX_SUPPORTED_MOTORS];  // output values for each motor during probing
    float servoOutput[MAX_SUPPORTED_SERVOS];  // output values for each servo during probing
} prober_runtime_t;

extern prober_runtime_t proberRuntime;

void initProber(timeUs_t currentTimeUs);
void updateProber(timeUs_t currentTimeUs);
