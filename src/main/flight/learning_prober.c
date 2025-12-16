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

#include "flight/learner.h"
#include "learning_prober.h"
#include "common/ortho_signals.h"
#include "common/filter.h"
#include "flight/indi.h"
#include "flight/servos.h"
#include "drivers/light_led.h"

#ifdef USE_LEARNER

PG_REGISTER_WITH_RESET_TEMPLATE(proberConfig_t, proberConfig, PG_PROBER_CONFIG, 0);
PG_RESET_TEMPLATE(proberConfig_t, proberConfig,
    // .type = (uint8_t) PROBER_STEPS,
    .type = (uint8_t) PROBER_ORTHO,
    .actMask = 0xFFFF, // all motors and servos
    // .numMotors = 2,
    // .numServos = 2,
    .preDelayMs = 100,
    .postDelayMs = 0,
    .steps_stepMs = 100,
    .steps_overlapMs = 0,
    .steps_amp = 35,
    .stepramps_stepMs = 50,
    .stepramps_rampMs = 100,
    .stepramps_overlapMs = 50,
    .stepramps_stepAmp = 35,
    .stepramps_rampAmp = 70,
    .multisine_fundamental = 15,
    .multisine_sinesPerActuator = 2,
    .multisine_optimizePhase = false,
    .multisine_timeMs = 467,
    .multisine_amp = 10,
    .noise_sampleTimeUs = 10000,
    .noise_amp = 20,
);

prober_runtime_t proberRuntime;

static void setZeroOutputs(void) {
    for (int i = 0; i < MAXU; i++) {
        proberRuntime.output[i] = 0.0f;
    }
}

static void initSteps(void) {
    const proberConfig_t* config = proberConfig();
    proberRuntime.sub.steps.currentStepIndex = 0;
    proberRuntime.sub.steps.nextStepTimeUs = proberRuntime.genStartTimeUs + config->steps_stepMs * 1000;
    proberRuntime.safetyTimeoutUs += config->steps_stepMs * 1000 * (learnRun.numActuators);
}

// static void initMultisine(void) {
//     const proberConfig_t* config = proberConfig();
//     proberRuntime.safetyTimeoutUs += config->multisine_timeMs * 1000;
// 
//     for (int i=0; i < config->numMotors; i++) {
//         // proberRuntime.sub.multisine.motorPhases[i] = (float)rand() / RAND_MAX * 2.0f * M_PIf; // random phase between 0 and 2π
//         proberRuntime.sub.multisine.motorPhases[i] = i * 0.5f * M_PIf;
//     }
//     for (int i=0; i < config->numServos; i++) {
//         proberRuntime.sub.multisine.servoPhases[i] = 0.0f;
//     }
// }

static void updateSteps(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();
    int totalSteps = learnRun.numActuators;

    setZeroOutputs();

    int act = 0, counter = 0;
    while (act < indiRun.actNum) {
        if (config->actMask & (1 << (act++))) {
            if (counter++ >= proberRuntime.sub.steps.currentStepIndex) {
                // If we have reached the total number of steps, break
                break;
            }
        }
    }

    act--;

    if (proberRuntime.sub.steps.currentStepIndex < totalSteps) {
        float output = 0.01f * config->steps_amp;

        if (proberRuntime.sub.steps.currentStepIndex < learnRun.numActuators) {
            if (act==0) {
                proberRuntime.output[act] = output;
            } else if (act==1) {
                proberRuntime.output[act] = output;
                proberRuntime.output[act+1] = 0.4f;  // trigger servo
            } else if (act==2) {
                proberRuntime.output[act] = 0;
                proberRuntime.output[act-2] = output;
                proberRuntime.output[act+1] = 0.4f;  // trigger other servo servo
            } else if (act==3) {
                proberRuntime.output[act] = 0;
                proberRuntime.output[act-2] = output;
            }
        }

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

// static void updateMultisine(timeUs_t currentTimeUs) {
//     const proberConfig_t* config = proberConfig();
// 
//     if (cmpTimeUs(currentTimeUs, proberRuntime.genStartTimeUs) < config->multisine_timeMs * 1000) {
//         setZeroOutputs();
//         // Generate multisine output
//         for (int i = 0; i < config->numMotors; i++) {
//             // Calculate the sine wave output for each motor
//             proberRuntime.output[i] = 0.015f*config->multisine_amp;
//             float dt_s = 0.000001f * (currentTimeUs - proberRuntime.genStartTimeUs);
//             for (int j = 1; j <= config->multisine_sinesPerActuator; j++) {
//                 float phase = 2.f*M_PIf * dt_s * j * config->multisine_fundamental + proberRuntime.sub.multisine.motorPhases[i];
//                 // Wrap the phase to keep it within [0, 2π]
//                 while (phase >= 2 * M_PIf) {
//                     phase -= 2 * M_PIf;
//                 }
//                 proberRuntime.output[i] += 0.01f * config->multisine_amp * sinf(phase);
//             }
//         }
//     } else {
//         // All steps done, set finished flag
//         proberRuntime.isGenFinished = true;
//         proberRuntime.genFinishTimeUs = currentTimeUs;
//     }
// }

static void updateOrtho(timeUs_t currentTimeUs) {
    timeDelta_t timeSinceStartUs = cmpTimeUs(currentTimeUs, proberRuntime.genStartTimeUs);
    float t = 0.000001f * timeSinceStartUs; // convert to seconds

    setZeroOutputs();
    if (!orthoSignalGenerate(t, proberRuntime.output, learnRun.numActuators)) {
        // All steps done, set finished flag
        proberRuntime.isGenFinished = true;
        proberRuntime.genFinishTimeUs = currentTimeUs;
    }
}

#define PROBER_SAFETY_TIME_MAX ((timeUs_t) 1000000) // 1 sec

// #define DEBUG_SERVOS 1

#ifdef DEBUG_SERVOS

#include "drivers/exti.h"
#include "drivers/io_types.h"
#include "drivers/exti.h"

static IO_t signalPin;

void initProberDebugPin(void) {
    // see accgyro_mpu.c:mpuIntextiInit() for this code
    signalPin = IOGetByTag(IO_TAG(PE6));
    IOInit(signalPin, OWNER_CAMERA_CONTROL, 0);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(signalPin),
        .Mode = GPIO_MODE_OUTPUT_PP, // internal push pull
        .Speed = GPIO_SPEED_FREQ_HIGH, // high slew rate, i guess
        .Pull = GPIO_NOPULL, // chatGPT says so
    };
    if (signalPin) {
        HAL_GPIO_Init(IO_GPIO(signalPin), &init);
    }
}
#else
void initProberDebugPin(void) {
}
#endif

void initProber(timeUs_t currentTimeUs) {
    const proberConfig_t* config = proberConfig();
    proberRuntime.type = (prober_type_t) config->type;

    // get number of actuators
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
//         case PROBER_MULTISINE:
//             // Initialize multisine prober runtime
//             initMultisine();
//             break;
        case PROBER_NOISE:
            // Initialize noise prober runtime
            break;
        case PROBER_ORTHO:
            proberRuntime.safetyTimeoutUs += 700000;  // add half a second // TODO THIS IS BEUN AF
            // Initialize orthogonal signal prober runtime not needed
            // TODO: do properly
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

static pt1Filter_t servoFeedbackFilter[MAXU];

void initServoProber(timeUs_t currentTimeUs) {
    for (int i = 0; i < indiRun.actNum; i++) {
        if (indiRun.actType[i] == INDI_ACT_TYPE_SERVO) {
            pt1FilterInit(&servoFeedbackFilter[i], pt1FilterGain(1.f / (2.f * M_PIf * 0.005f), indiRun.dT)); // 32Hz
        }
    }
    proberRuntime.isInitializedServo = true;
    proberRuntime.isGenRunningServo = false;
    proberRuntime.lastStepTimeUs = currentTimeUs;
}

#define LEARNER_SERVO_TIMESTEP_US (250000) // 250ms
#define LEARNER_SERVO_NUM_STEPS (5)

void updateServoProber(timeUs_t currentTimeUs) {
    if (!proberRuntime.isInitializedServo) {
        return;
    }


    static int stepIndex = 0;
    proberRuntime.isGenRunningServo = true;

    float steps[LEARNER_SERVO_NUM_STEPS] = {0.2f, -0.2f, 0.0f, 0.2f, 0.0f};
    // filter the servo_feedback returns
    float servoRadFiltered[MAXU];
    static float servoRadFilteredPrev[MAXU] = {0.0f};
    static float delay_thresh = 0.f;
    static float tau_thresh = 0.f;
    int s = 0;
    for (int i = 0; i < indiRun.actNum; i++) {
        if (indiRun.actType[i] == INDI_ACT_TYPE_SERVO) {
            float servoRadRaw = DEGREES_TO_RADIANS( 0.01f * ((float)servo_feedback[s++]) );
            servoRadFiltered[i] = pt1FilterApply(&servoFeedbackFilter[i], servoRadRaw);

            // detect stuff 
            float divisor = proberRuntime.output[i];
            if (divisor < 1e-3f && divisor > -1e-3f) {
                divisor = 1e-3f; // prevent div by zero
            }


            switch (stepIndex)
            {
                case 0:
                    // first step, record maximum
                    actRls[i].x[0] = servoRadFiltered[i] / divisor;
                    break;
                case 1:
                    // second step, record max
                    actRls[i].x[1] = (-actRls[i].x[0] * divisor + servoRadFiltered[i]) * 0.5f;
                    break;
                case 2:
                    // back to center, wait for things to settle, record last value as threshold for delay detection
                    delay_thresh = servoRadFiltered[i] + 0.5f*0.01745f; // 1deg deadzone
                    tau_thresh = 0.632f * (actRls[i].x[0]*steps[0] - actRls[i].x[1]);
                    break;
                case 3:
                    // rising edge
                    if (servoRadFilteredPrev[i] < delay_thresh && servoRadFiltered[i] >= delay_thresh) {
                        actRls[i].x[2] = 1e-5f * cmpTimeUs(currentTimeUs, proberRuntime.lastStepTimeUs);
                    }
                    if (servoRadFilteredPrev[i] < tau_thresh && servoRadFiltered[i] >= tau_thresh) {
                        actRls[i].x[3] = 1e-5f * cmpTimeUs(currentTimeUs, proberRuntime.lastStepTimeUs) - actRls[i].x[2];
                    }
                    break;
                case 4:
                    // back to center
                    break;
                default:
                    break;
            }
            servoRadFilteredPrev[i] = servoRadFiltered[i];

            proberRuntime.output[i] = steps[stepIndex];

            // add ramp to investigate delay TODO
            // proberRuntime.output[i] += 0.5f * 1e-6f * (float) (currentTimeUs - proberRuntime.lastStepTimeUs);
            proberRuntime.output[i] = constrainf(proberRuntime.output[i], -0.4f, 0.4f);
        }
    }

    timeDelta_t delta = cmpTimeUs(currentTimeUs, proberRuntime.lastStepTimeUs);
    if (delta > LEARNER_SERVO_TIMESTEP_US) {
        if (stepIndex >= LEARNER_SERVO_NUM_STEPS) {
            // finished
            stepIndex = 0;
            proberRuntime.isGenFinishedServo = true;
            proberRuntime.isGenRunningServo = false;
            proberRuntime.isInitializedServo = false;
            return;
        }

#ifdef DEBUG_SERVOS
        LED1_TOGGLE;
        if (signalPin) {
            IOToggle(signalPin);
        }
#endif

        proberRuntime.lastStepTimeUs = currentTimeUs;
        stepIndex++;
    }
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
            goto zeroAndReturn;
        } else if (proberRuntime.isGenFinished) {
            // set zero outputs, but keep prober running
            setZeroOutputs();
            proberRuntime.isGenRunning = true;
            return;  // Prober is finished, no updates needed
        } else {
            goto zeroAndReturn;
        }

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
//         case PROBER_MULTISINE:
//             // Handle multisine prober logic
//             updateMultisine(currentTimeUs);
//             break;
        case PROBER_NOISE:
            // Handle noise prober logic
            break;
        case PROBER_ORTHO:
            // Handle orthogonal signal prober logic
            updateOrtho(currentTimeUs);
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
