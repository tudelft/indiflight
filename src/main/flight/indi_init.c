
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
    // ---- general INDI config
    indiProfile->useIncrement = true;
    indiProfile->useRpmDotFeedback = true;
    // ---- INDI actuator config
    indiProfile->actNum = 4;
    for (int i = 0; i < MAXU; i++) {
        indiProfile->actHoverRpm[i] = 1500;
        indiProfile->actTimeConstMs[i] = 25;
        indiProfile->actPropConst[i] = 1e5;
        indiProfile->actMaxT[i] = 500;
        indiProfile->actNonlinearity[i] = 50;
        indiProfile->actLimit[i] = 100;
        indiProfile->actG1_fx[i] = 0;
        indiProfile->actG1_fy[i] = 0;
        indiProfile->actG1_fz[i] = 0;
        indiProfile->actG1_roll[i] = 0;
        indiProfile->actG1_pitch[i] = 0;
        indiProfile->actG1_yaw[i] = 0;
        indiProfile->actG2_roll[i] = 0;
        indiProfile->actG2_pitch[i] = 0;
        indiProfile->actG2_yaw[i] = 0;     // actuator rate effectiveness
        // ---- WLS config
        indiProfile->wlsWu[i] = 1;
        indiProfile->u_pref[i] = 0;
    }

    indiProfile->wlsWv[0] = 1; // fx
    indiProfile->wlsWv[1] = 1; // fy
    indiProfile->wlsWv[2] = 10; // fz
    indiProfile->wlsWv[3] = 10; // roll
    indiProfile->wlsWv[4] = 10; // pitch
    indiProfile->wlsWv[5] = 1; // yaw

    // ---- Filtering config
    indiProfile->imuSyncLp2Hz = 40;

    // -------- inaccessible parameters for now (will always be the values from the reset function)
    // ---- Att/Rate config
    indiProfile->attRateDenom = 8;
    // ---- WLS config
    indiProfile->useConstantG2 = false;
    indiProfile->useRpmFeedback = false;
    // ---- general INDI config
    indiProfile->useWls = true;
    indiProfile->wlsWarmstart = true;
    indiProfile->wlsMaxIter = 1;
    indiProfile->wlsAlgo = 1;
    indiProfile->wlsCondBound = 1 << 15;
    indiProfile->wlsTheta = 1;
    indiProfile->wlsNanLimit = 20;
}

void initIndiRuntime(void) {
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
    indiRun.useConstantG2 = (bool) p->useConstantG2;
    indiRun.useRpmFeedback = (bool) p->useRpmFeedback;
    indiRun.useRpmDotFeedback = (bool) p->useRpmDotFeedback;
    indiRun.actNum = MIN(p->actNum, MAXU);
    // ---- INDI actuator config
    for (int i = 0; i < MAXU; i++) {
        float hoverRpm = MAX(1U, p->actHoverRpm[i]) * 10.f;
        indiRun.actHoverOmega[i]   = hoverRpm / SECONDS_PER_MINUTE * 2.f * M_PIf;
        indiRun.actTimeConstS[i]   = MAX(1UL, p->actTimeConstMs[i]) * 1e-3f;
        indiRun.actPropConst[i]    = MAX(1UL, p->actPropConst[i]) * 1e-11f;
        indiRun.actMaxT[i]         = MAX(1UL, p->actMaxT[i]) * 0.01f;
        indiRun.actNonlinearity[i] = constrainu(p->actNonlinearity[i], 0, 100) * 0.01f;
        unsigned int tmp = MIN(currentPidProfile->motor_output_limit, indiRun.actLimit[i]);
        indiRun.actLimit[i] = constrainu(tmp, 0, 100) * 0.01f;
        indiRun.actG1[0][i] = p->actG1_fx[i] * 0.01f;
        indiRun.actG1[1][i] = p->actG1_fy[i] * 0.01f;
        indiRun.actG1[2][i] = p->actG1_fz[i] * 0.01f;
        indiRun.actG1[3][i] = p->actG1_roll[i]  * 0.1f;
        indiRun.actG1[4][i] = p->actG1_pitch[i] * 0.1f;
        indiRun.actG1[5][i] = p->actG1_yaw[i]   * 0.01f;
        indiRun.actG2[0][i] = p->actG2_roll[i] / hoverRpm * 0.1f;
        indiRun.actG2[1][i] = p->actG2_pitch[i] / hoverRpm * 0.1f;
        indiRun.actG2[2][i] = p->actG2_yaw[i] / hoverRpm * 0.1f;
        // ---- WLS config
        indiRun.wlsWu[i] = (float) p->wlsWu[i];
        indiRun.u_pref[i] = p->u_pref[i] * 0.01f;
    }
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

    // ---- runtime values -- actauators
    for (int i = 0; i < indiRun.actNum; i++) {
        indiRun.d[i] = 0.f; // command issued to the actuators on [-1, 1] scale
        indiRun.u[i] = 0.f; // control variable proportional to output force, but on [-1, 1], for motors [0, 1]
        indiRun.uState[i] = 0.f; // estimated force state of the actuators [-1, 1]
        indiRun.uState_fs[i] = 0.f; // sync-filtered estiamted force state
        updateLinearization(&indiRun.lin[i], indiRun.actNonlinearity[i]);
        indiRun.omega[i] = 0.f; // unfiltered motor speed rad/s
        indiRun.omega_fs[i] = 0.f; // sync-filtered motor speed rad/s
        //indiRun.omegaDot[i] = 0.f; // unfiltered motor rate rad/s/s
        indiRun.omegaDot_fs[i] = 0.f; // sync-filtered motor rate rad/s/s
        indiRun.G2_normalizer[i] = 1.f / (2.f * indiRun.actTimeConstS[i] * indiRun.actPropConst[i]);
    }
    indiRun.erpmToRads = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);
    // ---- runtime values -- axes
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        indiRun.attGainsCasc.A[axis] = indiRun.attGains.A[axis] / indiRun.rateGains.A[axis]; // attitude gains simulating parallel PD
        indiRun.rateSpBody.A[axis] = 0.; // rate setpoint in body coordinates
        indiRun.rateDotSpBody.A[axis] = 0.; // rate derivative setpoint in body coordinates
        indiRun.spfSpBody.A[axis] = 0.; // specific force setpoint in body coordinates
        //indiRun.rate_fs.A[axis] = 0.; // sync-filtered gyro in rad/s
        indiRun.rateDot.A[axis] = 0.; // unfiltered gyro derivative in rad/s/s
        indiRun.rateDot_fs.A[axis] = 0.; // sync-filtered gyro derivative in rad/s/s
        indiRun.spf_fs.A[axis] = 0.; // sync-filterd accelerometer (specific force) in N/kg
    }
    indiRun.attSpNed = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    indiRun.attErrBody = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    for (int j = 0; j < MAXV; j++)
        indiRun.dv[MAXV] = 0.f; // delta-pseudo controls in N/kg and Nm/(kgm^2)

    // ---- housekeeping
    indiRun.dT = gyro.targetLooptime * 1e-6f; // target looptime in S
    indiRun.indiFrequency = 1.0f / gyro.targetLooptime; // target looptime in S
    indiRun.attExecCounter = 0; // count executions (wrapping)
    indiRun.nanCounter = 0; // count times consequtive nans appear in allocation

    // ---- control law selection
    indiRun.bypassControl = false; // no control at all. u and d are unmodified by loop
    indiRun.controlAttitude = true; // attempt to reach tilt given by attSpNed
    indiRun.trackAttitudeYaw = false; // also attempt to reach yaw given by attSpNed

    // ---- filters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        biquadFilterInitLPF(&indiRun.rateFilter[axis], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&indiRun.spfFilter[axis], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
    }
    for (int i = 0; i < indiRun.actNum; i++) {
        pt1FilterInit(&indiRun.uLagFilter[i], pt1FilterGain(1.f / (2.f * M_PIf * indiRun.actTimeConstS[i]), indiRun.dT)); // to simulate spinup
        biquadFilterInitLPF(&indiRun.uStateFilter[i], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&indiRun.omegaFilter[i], indiRun.imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
    }
}

#endif // ifdef USE_INDI
