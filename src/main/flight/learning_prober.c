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

#include "common/maths.h"
//#include <math.h>
#include <stdlib.h>
// #include "common/axis.h"
// #include "common/filter.h"
// #include "common/rng.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#include "drivers/time.h"
#include "common/time.h"
// #include "fc/runtime_config.h"
// #include "fc/core.h"
#include "pg/pg_ids.h"

// #include "sensors/gyro.h"
// #include "sensors/acceleration.h"
// #include "sensors/boardalignment.h"
// #include "flight/ahrs.h"
// #include "flight/indi.h"
// #include "flight/indi_init.h"
// #include "flight/catapult.h"
// #include "flight/pos_ctl.h"
// #include "flight/throw.h"
// #include "ekf_calc.h"   // for dirty override
// 
// #include "f2c.h"
// #include "clapack.h"

#include <stdbool.h>

#include "learning_prober.h"

#ifdef USE_LEARNER

PG_REGISTER_WITH_RESET_TEMPLATE(proberConfig_t, proberConfig, PG_PROBER_CONFIG, 1);
PG_RESET_TEMPLATE(proberConfig_t, proberConfig,
    // .type = (uint8_t) PROBER_STEPS,
    .type = (uint8_t) PROBER_MULTISINE,
    .numMotors = 4,
    .numServos = 0,
    .preDelayMs = 250,
    .postDelayMs = 250,
    .steps_stepMs = 50,
    .steps_overlapMs = 0,
    .steps_amp = 35,
    .stepramps_stepMs = 50,
    .stepramps_rampMs = 100,
    .stepramps_overlapMs = 50,
    .stepramps_stepAmp = 35,
    .stepramps_rampAmp = 70,
    .multisine_fundamental = 10,
    .multisine_sinesPerActuator = 3,
    .multisine_optimizePhase = false,
    .multisine_timeMs = 500,
    .multisine_amp = 20,
    .noise_sampleTimeUs = 10000,
    .noise_amp = 20,
);

prober_runtime_t proberRuntime;

static void setZeroOutputs(void) {
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        proberRuntime.motorOutput[i] = 0.0f;
    }
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        proberRuntime.servoOutput[i] = 0.0f;
    }
}

static void initSteps(void) {
    const proberConfig_t* config = proberConfig();
    proberRuntime.sub.steps.currentStepIndex = 0;
    proberRuntime.sub.steps.nextStepTimeUs = proberRuntime.genStartTimeUs + config->steps_stepMs * 1000;
    proberRuntime.safetyTimeoutUs += config->steps_stepMs * 1000 * config->numMotors;
}

static void initMultisine(void) {
    const proberConfig_t* config = proberConfig();
    proberRuntime.safetyTimeoutUs += config->multisine_timeMs * 1000;

    for (int i=0; i < config->numMotors; i++) {
        // proberRuntime.sub.multisine.motorPhases[i] = (float)rand() / RAND_MAX * 2.0f * M_PIf; // random phase between 0 and 2π
        proberRuntime.sub.multisine.motorPhases[i] = i * 0.5f * M_PIf; // random phase between 0 and 2π
    }
    for (int i=0; i < config->numServos; i++) {
        proberRuntime.sub.multisine.servoPhases[i] = 0.0f;
    }
}

static void updateSteps(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();

    setZeroOutputs();

    if (proberRuntime.sub.steps.currentStepIndex < config->numMotors) {

        proberRuntime.motorOutput[proberRuntime.sub.steps.currentStepIndex] =
            0.01f * config->steps_amp;  // Convert to float output

        if (currentTimeUs >= proberRuntime.sub.steps.nextStepTimeUs) {
            // Prepare for the next step
            proberRuntime.sub.steps.currentStepIndex++;
            proberRuntime.sub.steps.nextStepTimeUs += config->steps_stepMs * 1000;
        }
    } else {
        // All steps done, set finished flag
        proberRuntime.isGenFinished = true;
        proberRuntime.genFinishTimeUs = currentTimeUs;
    }
}

static void updateMultisine(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();

    if (cmpTimeUs(currentTimeUs, proberRuntime.genStartTimeUs) < config->multisine_timeMs * 1000) {
        setZeroOutputs();
        // Generate multisine output
        for (int i = 0; i < config->numMotors; i++) {
            // Calculate the sine wave output for each motor
            float dt_s = 0.000001f * (currentTimeUs - proberRuntime.genStartTimeUs);
            float phase = 2.f*M_PIf * dt_s * config->multisine_fundamental + proberRuntime.sub.multisine.motorPhases[i];
            // Wrap the phase to keep it within [0, 2π]
            while (phase >= 2 * M_PIf) {
                phase -= 2 * M_PIf;
            }
            proberRuntime.motorOutput[i] = (0.01f*config->multisine_amp) + 0.01f * config->multisine_amp * sinf(phase);
        }
    } else {
        // All steps done, set finished flag
        proberRuntime.isGenFinished = true;
        proberRuntime.genFinishTimeUs = currentTimeUs;
    }
}

#define PROBER_SAFETY_TIME_MAX ((timeUs_t) 1000000) // 1 sec

void initProber(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();
    proberRuntime.type = (prober_type_t) config->type;

    proberRuntime.initTimeUs = currentTimeUs;
    proberRuntime.safetyTimeoutUs = currentTimeUs + 100000 + 1000 * (config->preDelayMs + config->postDelayMs);
    proberRuntime.genStartTimeUs = currentTimeUs + config->preDelayMs * 1000;

    switch (proberRuntime.type) {
        case PROBER_STEPS:
            // Initialize steps prober runtime
            initSteps();
            break;
        case PROBER_STEPRAMPS:
            // Initialize stepramps prober runtime
            break;
        case PROBER_MULTISINE:
            // Initialize multisine prober runtime
            initMultisine();
            break;
        case PROBER_NOISE:
            // Initialize noise prober runtime
            break;
        default:
            // Handle unknown prober type
            break;
    }

    proberRuntime.isInitialized = true;
    proberRuntime.isGenFinished = false;
    proberRuntime.isFinished = false;

    setZeroOutputs();
}

void updateProber(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();

    if (!proberRuntime.isInitialized
            || cmpTimeUs(currentTimeUs, proberRuntime.initTimeUs) <= config->preDelayMs * 1000
            || proberRuntime.isGenFinished
            || proberRuntime.isFinished) {
        if (proberRuntime.isGenFinished && (cmpTimeUs(currentTimeUs, proberRuntime.genFinishTimeUs) > config->postDelayMs * 1000)) {
            proberRuntime.isFinished = true;
            proberRuntime.isInitialized = false;
        }
        goto zeroAndReturn;
    }

    if (cmpTimeUs(currentTimeUs, proberRuntime.safetyTimeoutUs) > 0) {
        proberRuntime.isFinished = true;
        proberRuntime.isInitialized = false;
        goto zeroAndReturn;
    }

    // Update logic based on the prober type
    switch (proberRuntime.type) {
        case PROBER_STEPS:
            // Handle steps prober logic
            updateSteps(currentTimeUs);
            break;
        case PROBER_STEPRAMPS:
            // Handle stepramps prober logic
            break;
        case PROBER_MULTISINE:
            // Handle multisine prober logic
            updateMultisine(currentTimeUs);
            break;
        case PROBER_NOISE:
            // Handle noise prober logic
            break;
        default:
            // Handle unknown prober type
            break;
    }

    proberRuntime.isGenRunning = true;
    return;

zeroAndReturn:
    setZeroOutputs();
    proberRuntime.isGenRunning = false;
    return;  // Prober is not initialized or already finished, no updates needed
}

#endif
