/*
 * INDI based attitude and rate controller for UAV -- runtime initialization
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
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


#include "platform.h"

#ifdef USE_INDI

#include "sensors/gyro.h"
#include "common/filter.h"
#include "flight/rpm_filter.h"
#include "flight/mixer_init.h"

#include "indi.h"
#include "indi_init.h"

void pgResetFn_indiProfiles(indiProfile_t *indiProfiles) {
    for (int i = 0; i < INDI_PROFILE_COUNT; i++) {
        resetIndiProfile(&indiProfiles[i]);
    }
}

void changeIndiProfile(uint8_t profileIndex)
{
    if (profileIndex < INDI_PROFILE_COUNT) {
        systemConfigMutable()->indiProfileIndex = profileIndex;
    }

    initIndiRuntime();
}

void resetIndiProfile(indiProfile_t *indiProfile) {
    indiProfile->attGains[0] = 2000; // roll
    indiProfile->attGains[1] = 2000; // pitch
    indiProfile->attGains[2] = 1000; // yaw
    indiProfile->rateGains[0] = 200; // roll
    indiProfile->rateGains[1] = 200; // pitch
    indiProfile->rateGains[2] = 200; // yaw
    indiProfile->attMaxTiltRate = 800; // deg/s
    indiProfile->attMaxYawRate = 400; // deg/s
    indiProfile->manualUseCoordinatedYaw = true;
    indiProfile->manualMaxUpwardsSpf = 30;
    indiProfile->manualMaxTilt = 45; // degrees
    for (int i = 0; i < 5; i++) {
        indiProfile->feedforwardCoefs[i] = 0;
    }
    // ---- general INDI config
    indiProfile->useIncrement = true;
    indiProfile->useAccelForSpfz = true;
    indiProfile->useRpmDotFeedback = true;
    indiProfile->useMotorLeadLag = false;
    indiProfile->useAttLeadLag = false;

    // ---- INDI actuator config
    indiProfile->actNum = 4;
    for (int i = 0; i < MAXU; i++) {
        indiProfile->actTimeConstMs[i] = 25;
        indiProfile->actHoverRpm[i] = 20000;
        indiProfile->actMaxRpm[i] = 40000;
        indiProfile->actNonlinearity[i] = 50;
        indiProfile->actMin[i] = 0;
        indiProfile->actMax[i] = 100;
        indiProfile->actIsMotor[i] = true;
        indiProfile->actIsServo[i] = false;
        indiProfile->actG1_fx[i] = 0;
        indiProfile->actG1_fy[i] = 0;
        indiProfile->actG1_fz[i] = 0;
        indiProfile->actG1_roll[i] = 0;
        indiProfile->actG1_pitch[i] = 0;
        indiProfile->actG1_yaw[i] = 0;
        indiProfile->actG2_roll[i] = 0;
        indiProfile->actG2_pitch[i] = 0;
        indiProfile->actG2_yaw[i] = 0;     // actuator rate effectiveness
        indiProfile->actTimeConstTrueMs[i] = 25; // true time constant in seconds
        indiProfile->actTimeConstDesMs[i] = 25; // desired time constant in seconds
        // ---- WLS config
        indiProfile->wlsWu[i] = 1;
        indiProfile->u_pref[i] = 0;
    }

    indiProfile->wlsWv[0] = 1; // fx
    indiProfile->wlsWv[1] = 1; // fy
    indiProfile->wlsWv[2] = 50; // fz
    indiProfile->wlsWv[3] = 50; // roll
    indiProfile->wlsWv[4] = 50; // pitch
    indiProfile->wlsWv[5] = 5; // yaw

    // ---- Tailsitter
    indiProfile->tails_use_scheduled = false;
    indiProfile->tails_use_sine = true;
    indiProfile->tails_cxw = 0;
    indiProfile->tails_cyw = 0;
    indiProfile->tails_czw = 0;
    indiProfile->tails_clw = 0;
    indiProfile->tails_cmw = 0;
    indiProfile->tails_cnw = 0;
    indiProfile->tails_cnwd = 0;
    indiProfile->tails_cxd = 0;
    indiProfile->tails_cmd = 0;
    indiProfile->tails_cnd = 0;

    // ---- Filtering config
    indiProfile->imuSyncLp2Hz = 15;

    // -------- inaccessible parameters for now (will always be the values from the reset function)
    // ---- Att/Rate config
    indiProfile->attRateDenom = 1;
    // ---- WLS config
    indiProfile->useConstantG2 = false;
    indiProfile->useRpmFeedback = false;
    indiProfile->maxRateSp[0] = 1800.f;          // maximum rate setpoint in deg/s
    indiProfile->maxRateSp[1] = 1800.f;          // maximum rate setpoint in deg/s
    indiProfile->maxRateSp[2] = 1800.f;          // maximum rate setpoint in deg/s
    // ---- general INDI config
    indiProfile->useWls = true;
    indiProfile->wlsWarmstart = true;
    indiProfile->wlsMaxIter = 1;
    indiProfile->wlsAlgo = 1;
    indiProfile->wlsCondBound = 1 << 15;
    indiProfile->wlsTheta = 1;
    indiProfile->wlsNanLimit = 20;

    // ---- gain scheduling
    indiProfile->useGainScheduling = false;
    indiProfile->gainSchedulingType = 0; // 0: None, 1: 1D interp, 2: 2D interp, 3: 1D poly, 4: 2D poly
    indiProfile->gainScheduleFf = false; // Whether to take feedforward values from gain-scheduling
    
    // ---- attitude injection
    indiProfile->injectAttSp = false; // whether to inject attitude setpoints
    indiProfile->attSpInjectionStarted = false;
    indiProfile->attSpInjectionType = 0; // 0: None, 1: Impulse, 2: Doublet, 3: Step
    indiProfile->attSpInjectionAmplitude = 0; // amplitude of injection in degrees * 10
    indiProfile->attSpInjectionDuration = 0;  // duration of injection in ms
    indiProfile->attSpInjectionDurationDown = 0; // for doublet, duration of negative pulse in ms

    #ifdef INJECT_INPUT_DISTURBANCE
    // ---- input disturbance injection
    indiProfile->injectInputDist = false; // whether to inject input disturbance
    indiProfile->inputDistAmplitude = 0; // amplitude of input disturbance in percent
    for (int i = 0; i < MAXU; i++) {
        indiProfile->applyDistToMotor[i] = false; // whether to apply disturbance to motor
    }
    #endif
}

void initIndiRuntimeParameters(void) {
    const indiProfile_t *p = indiProfiles(systemConfig()->indiProfileIndex);

    // ---- Att/Rate config
    indiRun.attGains.A[0] = p->attGains[0] * 0.1f;
    indiRun.attGains.A[1] = p->attGains[1] * 0.1f;
    indiRun.attGains.A[2] = p->attGains[2] * 0.1f;
    indiRun.rateGains.A[0] = MAX(1U, p->rateGains[0]) * 0.1f;
    indiRun.rateGains.A[1] = MAX(1U, p->rateGains[1]) * 0.1f;
    indiRun.rateGains.A[2] = MAX(1U, p->rateGains[2]) * 0.1f;
    indiRun.attMaxTiltRate = DEGREES_TO_RADIANS(p->attMaxTiltRate);
    indiRun.attMaxYawRate  = DEGREES_TO_RADIANS(p->attMaxYawRate);
    indiRun.attRateDenom = p->attRateDenom;
    indiRun.manualUseCoordinatedYaw = (bool) p->manualUseCoordinatedYaw;
    indiRun.manualMaxTilt = DEGREES_TO_RADIANS(p->manualMaxTilt);
    // ---- general INDI config
    indiRun.useIncrement = (bool) p->useIncrement;
    indiRun.useAccelForSpfz = (bool) p->useAccelForSpfz;
    indiRun.useConstantG2 = (bool) p->useConstantG2;
    indiRun.useRpmFeedback = (bool) p->useRpmFeedback;
    indiRun.useRpmDotFeedback = (bool) p->useRpmDotFeedback;
    indiRun.maxRateSp.A[0] = (float) DEGREES_TO_RADIANS(p->maxRateSp[0]);          // maximum rate setpoint in deg/s
    indiRun.maxRateSp.A[1] = (float) DEGREES_TO_RADIANS(p->maxRateSp[1]);          // maximum rate setpoint in deg/s
    indiRun.maxRateSp.A[2] = (float) DEGREES_TO_RADIANS(p->maxRateSp[2]);          // maximum rate setpoint in deg/s
    // ---- INDI actuator config
    indiRun.actNum = MIN(p->actNum, MAXU);
    for (int i = 0; i < MAXU; i++) {
        indiRun.actHoverOmega[i] = ((float) MAX(0U, p->actHoverRpm[i])) / SECONDS_PER_MINUTE * 2.f * M_PIf;
        float maxRpm = (float) MAX(0U, p->actMaxRpm[i]);
        indiRun.actMaxOmega[i]  = maxRpm / SECONDS_PER_MINUTE * 2.f * M_PIf;
        indiRun.actMaxOmega2[i] = sq( indiRun.actMaxOmega[i] );
        indiRun.actTimeConstS[i] = MAX(1UL, p->actTimeConstMs[i]) * 1e-3f;
        indiRun.actNonlinearity[i] = constrainu(p->actNonlinearity[i], 0, 100) * 0.01f;
        indiRun.actIsMotor[i] = (bool) p->actIsMotor[i];
        indiRun.actIsServo[i] = (bool) p->actIsServo[i];
        if (indiRun.actIsMotor[i]) {
            unsigned int tmp = MIN(currentPidProfile->motor_output_limit, p->actMax[i]);
            indiRun.actMin[i] = 0.f; // hardcode for safety!
            indiRun.actMax[i] = constrainu(tmp, 0, 100) * 0.01f;
        } else {
            indiRun.actMin[i] = constrain(p->actMin[i], -100, 100) * 0.01f;
            indiRun.actMax[i] = constrain(p->actMax[i], -100, 100) * 0.01f;
        }
        indiRun.actG1[0][i] = p->actG1_fx[i] * 0.01f;
        indiRun.actG1[1][i] = p->actG1_fy[i] * 0.01f;
        indiRun.actG1[2][i] = p->actG1_fz[i] * 0.01f;
        indiRun.actG1[3][i] = p->actG1_roll[i]  * 0.1f;
        indiRun.actG1[4][i] = p->actG1_pitch[i] * 0.1f;
        indiRun.actG1[5][i] = p->actG1_yaw[i]   * 0.1f;
        indiRun.actG2[0][i] = p->actG2_roll[i]  * 1e-5f;
        indiRun.actG2[1][i] = p->actG2_pitch[i] * 1e-5f;
        indiRun.actG2[2][i] = p->actG2_yaw[i]   * 1e-5f;
        indiRun.G2_scaler[i] = 0.5f * indiRun.actMaxOmega2[i] / indiRun.actTimeConstS[i];
        // ---- WLS config
        indiRun.wlsWu[i] = (float) p->wlsWu[i];
        indiRun.u_pref[i] = p->u_pref[i] * 0.01f;
        indiRun.useMotorLeadLag = (bool) p->useMotorLeadLag;
        indiRun.actTimeConstTrueS[i] = MAX(1UL, p->actTimeConstTrueMs[i]) * 1e-3f;
        indiRun.actTimeConstDesS[i] = MAX(1UL, p->actTimeConstDesMs[i]) * 1e-3f;
    }
    //indiRun.actG1[0][0] = NAN; // FIXME: crashtesting
    for (int i = 0; i < MAXV; i++)
        indiRun.wlsWv[i] = (float) p->wlsWv[i];

    if (p->manualMaxUpwardsSpf == 255) {
        indiRun.manualMaxUpwardsSpf = 0;
        for (int i = 0; i < indiRun.actNum; i++) {
            indiRun.manualMaxUpwardsSpf += -indiRun.actG1[2][i];
        }
    } else {
        indiRun.manualMaxUpwardsSpf = (float) p->manualMaxUpwardsSpf;
    }

    // ---- tailsitter
    indiRun.tailsUseScheduled = (bool) p->tails_use_scheduled;
    if (indiRun.tailsUseScheduled) {
        // enforce motor/servo assignment: first 2 motor, then 2 servos
        for (int m=0; m<indiRun.actNum; m++) {indiRun.actIsMotor[m]=(m<2) ? true : false;}
        for (int s=0; s<indiRun.actNum; s++) {indiRun.actIsServo[s]=(s>=2 && s<4) ? true : false;}
    }
    indiRun.tailsUseSine = (bool) p->tails_use_sine;
    indiRun.tailsD0[0]= DEGREES_TO_RADIANS( ((float) p->tails_d0[0]) * 1e-2f );
    indiRun.tailsD0[1]= DEGREES_TO_RADIANS( ((float) p->tails_d0[1]) * 1e-2f );
    indiRun.tailsCxw  = ((float) p->tails_cxw)  * 1e-9f;
    indiRun.tailsCyw  = ((float) p->tails_cyw)  * 1e-9f;
    indiRun.tailsCzw  = ((float) p->tails_czw)  * 1e-9f;
    indiRun.tailsClw  = ((float) p->tails_clw)  * 1e-8f;
    indiRun.tailsCmw  = ((float) p->tails_cmw)  * 1e-8f;
    indiRun.tailsCnw  = ((float) p->tails_cnw)  * 1e-8f;
    indiRun.tailsCnwd = ((float) p->tails_cnwd) * 1e-5f;
    indiRun.tailsCxd  = ((float) p->tails_cxd)  * 1e-8f;
    indiRun.tailsCmd  = ((float) p->tails_cmd)  * 1e-8f;
    indiRun.tailsCnd  = ((float) p->tails_cnd)  * 1e-8f;

    // ---- Filtering config
    indiRun.imuSyncLp2Hz = (float) constrainu(p->imuSyncLp2Hz, 1, 5e5 / gyro.targetLooptime);
    // ---- WLS config
    indiRun.wlsAlgo = (activeSetAlgoChoice) p->wlsAlgo;
    indiRun.useWls = (bool) p->useWls;
    indiRun.wlsWarmstart = (bool) p->wlsWarmstart;
    indiRun.wlsMaxIter = p->wlsMaxIter;
    indiRun.wlsCondBound = p->wlsCondBound * 1e4f;
    indiRun.wlsTheta = p->wlsTheta * 1e-4;
    indiRun.wlsNanLimit = p->wlsNanLimit;

    // actuators
    indiRun.erpmToRads = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);
    for (int i = 0; i < indiRun.actNum; i++) {
        if (indiRun.actIsMotor[i]) {
            updateLinearization(&indiRun.lin[i], indiRun.actNonlinearity[i]);
        } else if (indiRun.actIsServo[i]) {
            indiRun.lin[i].k = 0.f;
            indiRun.lin[i].A = 0.f;
        }
    }

    // gains
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        indiRun.attGainsCasc.A[axis] = indiRun.attGains.A[axis] / indiRun.rateGains.A[axis]; // attitude gains simulating parallel PD
    }

    for (int i = 0; i < 5; i++) {
        indiRun.feedforwardCoefs[i] = p->feedforwardCoefs[i] * 1e-6f;
        indiRun.attCoefs[i] = p->attCoefs[i] * 1e-6f;
        indiRun.rateCoefs[i] = p->rateCoefs[i] * 1e-6f;
    }
    // ---- housekeeping
    indiRun.dT = gyro.targetLooptime * 1e-6f; // target looptime in S
    indiRun.indiFrequency = 1.0f / indiRun.dT; // target looptime in S

    // ---- control law selection
    indiRun.bypassControl = false; // no control at all. u and d are unmodified by loop
    indiRun.controlAttitude = true; // attempt to reach tilt given by attSpNed
    indiRun.trackAttitudeYaw = false; // also attempt to reach yaw given by attSpNedj
    indiRun.useAttLeadLag = p->useAttLeadLag; // use lead-lag filter for attitude control

    // ---- gain scheduling
    indiRun.useGainScheduling = p->useGainScheduling;
    indiRun.gainSchedulingType = p->gainSchedulingType;
    indiRun.gainScheduleFf = p->gainScheduleFf;

    // ---- attitude injection parameters
    #ifdef INJECT_ATTITUDE_SETPOINTS
    indiRun.injectAttSp = p->injectAttSp;
    indiRun.attSpInjectionStarted = false;

    switch (p->attSpInjectionType) {
        case 0: // None
            indiRun.attSpInjectionType = SIGNAL_MODE_OFF;
            break;
        case 1: // Impulse
            indiRun.attSpInjectionType = SIGNAL_MODE_IMPULSE;
            break;
        case 2: // Doublet
            indiRun.attSpInjectionType = SIGNAL_MODE_DOUBLET;
            break;
        case 3: // Step
            indiRun.attSpInjectionType = SIGNAL_MODE_STEP;
            break;
        default:
            indiRun.attSpInjectionType = SIGNAL_MODE_OFF;
            break;
    }

    indiRun.attSpInjectionAmplitude = DEGREES_TO_RADIANS(p->attSpInjectionAmplitude) * 0.1f; // convert to radians and scale
    indiRun.attSpInjectionDuration = p->attSpInjectionDuration * 1e-3f; // convert to seconds
    indiRun.attSpInjectionDurationDown = p->attSpInjectionDurationDown * 1e-3f; // convert to seconds
    #endif

    #ifdef INJECT_INPUT_DISTURBANCE
    indiRun.injectInputDist = p->injectInputDist;
    indiRun.inputDistAmplitude = constrain(p->inputDistAmplitude, 0, 100) * 0.01f; // Convert to motor setpoint
    for (int i = 0; i < MAXU; i++) {
        indiRun.applyDistToMotor[i] = p->applyDistToMotor[i];
    }
    #endif
}

void initIndiRuntime(void) {
    // set parameters from profile
    initIndiRuntimeParameters();

    // reset runtime values

    // ---- runtime values -- actuators
    for (int i = 0; i < indiRun.actNum; i++) {
        indiRun.d[i] = 0.f; // command issued to the actuators on [-1, 1] scale
        indiRun.u[i] = 0.f; // control variable proportional to output force, but on [-1, 1], for motors [0, 1]
        indiRun.uState[i] = 0.f; // estimated force state of the actuators [-1, 1]
        indiRun.uState_fs[i] = 0.f; // sync-filtered estimated force state
        indiRun.omega[i] = 0.f; // unfiltered motor speed rad/s
        indiRun.omega_fs[i] = 0.f; // sync-filtered motor speed rad/s
        //indiRun.omegaDot[i] = 0.f; // unfiltered motor rate rad/s/s
        indiRun.omegaDot_fs[i] = 0.f; // sync-filtered motor rate rad/s/s
    }
    // ---- runtime values -- axes
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        indiRun.attGainsCasc.A[axis] = indiRun.attGains.A[axis] / indiRun.rateGains.A[axis]; // attitude gains simulating parallel PD
        indiRun.rateSpBody.A[axis] = 0.; // rate setpoint in body coordinates
        indiRun.rateDotSpBody.A[axis] = 0.; // rate derivative setpoint in body coordinates
        indiRun.spfSpBody.A[axis] = 0.; // specific force setpoint in body coordinates
        indiRun.rateIMU.A[axis] = 0.; // unfiltered-filtered gyro in rad/s
        indiRun.rate_f.A[axis] = 0.; // unfiltered-filtered gyro in rad/s
        //indiRun.rate_fs.A[axis] = 0.; // sync-filtered gyro in rad/s
        indiRun.rateDotIMU.A[axis] = 0.; // unfiltered gyro derivative in rad/s/s
        indiRun.rateDot_fs.A[axis] = 0.; // sync-filtered gyro derivative in rad/s/s
        indiRun.spfIMU.A[axis] = 0.; // unfiltered accelerometer (specific force) in N/kg
        indiRun.spf_fs.A[axis] = 0.; // sync-filtered accelerometer (specific force) in N/kg
    }
    indiRun.attSpNed = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    indiRun.attErrBody = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    for (int j = 0; j < MAXV; j++) {
        indiRun.dv[j] = 0.f; // delta-pseudo controls in N/kg and Nm/(kgm^2)
    }

    // ---- housekeeping
    indiRun.attExecCounter = 0; // count executions (wrapping)
    indiRun.nanCounter = 0; // count times consecutive nans appear in allocation

    // ---- filters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        biquadFilterInitLPF(&indiRun.rateFilter[axis], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&indiRun.spfFilter[axis], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        if (indiRun.gainScheduleFf) {
            biquadFilterInitZeroPole(&indiRun.feedforwardFilter[axis], 0.0f, 0.0f, 0.0f, gyro.targetLooptime); // wait until gains scheduled for initializing filter fully
        } else {
            biquadFilterInitLeadLag(&indiRun.feedforwardFilter[axis], indiRun.feedforwardCoefs[0], indiRun.feedforwardCoefs[1], indiRun.feedforwardCoefs[2], indiRun.feedforwardCoefs[3], indiRun.feedforwardCoefs[4]);
        }
        biquadFilterInitLeadLag(&indiRun.rateLlFilter[axis], indiRun.rateCoefs[0], indiRun.rateCoefs[1], indiRun.rateCoefs[2], indiRun.rateCoefs[3], indiRun.rateCoefs[4]);
        biquadFilterInitLeadLag(&indiRun.attLlFilter[axis], indiRun.attCoefs[0], indiRun.attCoefs[1], indiRun.attCoefs[2], indiRun.attCoefs[3], indiRun.attCoefs[4]);
    }
    for (int i = 0; i < indiRun.actNum; i++) {
        pt1FilterInit(&indiRun.uLagFilter[i], pt1FilterGain(1.f / (2.f * M_PIf * indiRun.actTimeConstS[i]), indiRun.dT)); // to simulate spinup
        biquadFilterInitLPF(&indiRun.uStateFilter[i], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&indiRun.omegaFilter[i], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        if (indiRun.useMotorLeadLag) {
            // initialize motor lead-lag filter
            // b0 = 1, b1 = 0, b2 = 0, a1 = -exp(-dT/tauEst), a2 = exp(-dT/tauDes)
            biquadFilterInitMotorLeadLag(&indiRun.motorLeadLagFilter[i], indiRun.actTimeConstTrueS[i], indiRun.actTimeConstDesS[i], gyro.targetLooptime);
        }
    }
    #ifdef INJECT_ATTITUDE_SETPOINTS
    indiRun.attSpInjectionStartTime = 0;
    #endif 
}

#endif // ifdef USE_INDI
