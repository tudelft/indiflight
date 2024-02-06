
#include "platform.h"

#ifdef USE_INDI

#include "sensors/gyro.h"
#include "common/filter.h"
#include "flight/rpm_filter.h"
#include "flight/mixer_init.h"

#include "att_ctl.h"
#include "att_ctl_init.h"

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
    indiProfile_t *p = indiProfile;
    p->attGains[0] = 2000; // roll
    p->attGains[1] = 2000; // pitch
    p->attGains[2] = 1000; // yaw
    p->rateGains[0] = 200; // roll
    p->rateGains[1] = 200; // pitch
    p->rateGains[2] = 200; // yaw
    p->attMaxTiltRate = 800; // deg/s
    p->attMaxYawRate = 400; // deg/s
    p->manualUseCoordinatedYaw = true;
    p->manualMaxUpwardsSpf = 30;
    p->manualMaxTilt = 45; // degrees
    // ---- general INDI config
    p->useIncrement = true;
    p->useRpmDotFeedback = true;
    // ---- INDI actuator config
    p->actNum = 4;
    for (int i = 0; i < MAXU; i++) {
        p->actHoverRpm[i] = 1500;
        p->actTimeConstMs[i] = 25;
        p->actPropConst[i] = 1e5;
        p->actMaxT[i] = 500;
        p->actNonlinearity[i] = 50;
        p->actLimit[i] = 100;
        p->actG1_fx[i] = 0;
        p->actG1_fy[i] = 0;
        p->actG1_fz[i] = 0;
        p->actG1_roll[i] = 0;
        p->actG1_pitch[i] = 0;
        p->actG1_yaw[i] = 0;
        p->actG2_roll[i] = 0;
        p->actG2_pitch[i] = 0;
        p->actG2_yaw[i] = 0;     // actuator rate effectiveness
        // ---- WLS config
        p->wlsWu[i] = 1;
        p->u_pref[i] = 0;
    }

    p->wlsWv[0] = 1; // fx
    p->wlsWv[1] = 1; // fy
    p->wlsWv[2] = 10; // fz
    p->wlsWv[3] = 10; // roll
    p->wlsWv[4] = 10; // pitch
    p->wlsWv[5] = 1; // yaw

    // ---- Filtering config
    p->imuSyncLp2Hz = 40;

    // -------- inaccessible parameters for now (will always be the values from the reset function)
    // ---- Att/Rate config
    p->attRateDenom = 8;
    // ---- WLS config
    p->useConstantG2 = false;
    p->useRpmFeedback = false;
    // ---- general INDI config
    p->useWls = true;
    p->wlsWarmstart = true;
    p->wlsMaxIter = 1;
    p->wlsAlgo = 1;
    p->wlsCondBound = 1 << 15;
    p->wlsTheta = 1;
    p->wlsNanLimit = 20;
}

void initIndiRuntime(void) {
    indiRuntime_t *r = &indiRuntime;
    const indiProfile_t *p = indiProfiles(systemConfig()->indiProfileIndex);
    // ---- Att/Rate config
    r->attGains.A[0] = p->attGains[0] * 0.1f;
    r->attGains.A[1] = p->attGains[1] * 0.1f;
    r->attGains.A[2] = p->attGains[2] * 0.1f;
    r->rateGains.A[0] = MAX(1U, p->rateGains[0]) * 0.1f;
    r->rateGains.A[1] = MAX(1U, p->rateGains[1]) * 0.1f;
    r->rateGains.A[2] = MAX(1U, p->rateGains[2]) * 0.1f;
    r->attMaxTiltRate = DEGREES_TO_RADIANS(p->attMaxTiltRate);
    r->attMaxYawRate  = DEGREES_TO_RADIANS(p->attMaxYawRate);
    r->attRateDenom = p->attRateDenom;
    r->manualUseCoordinatedYaw = (bool) p->manualUseCoordinatedYaw;
    r->manualMaxUpwardsSpf = (float) p->manualMaxUpwardsSpf;
    r->manualMaxTilt = DEGREES_TO_RADIANS(p->manualMaxTilt);
    // ---- general INDI config
    r->useIncrement = (bool) p->useIncrement;
    r->useConstantG2 = (bool) p->useConstantG2;
    r->useRpmFeedback = (bool) p->useRpmFeedback;
    r->useRpmDotFeedback = (bool) p->useRpmDotFeedback;
    r->actNum = MIN(p->actNum, MAXU);
    // ---- INDI actuator config
    for (int i = 0; i < MAXU; i++) {
        float hoverRpm = MAX(1U, p->actHoverRpm[i]) * 10.f;
        r->actHoverOmega[i]   = hoverRpm / SECONDS_PER_MINUTE * 2.f * M_PIf;
        r->actTimeConstS[i]   = MAX(1UL, p->actTimeConstMs[i]) * 1e-3f;
        r->actPropConst[i]    = MAX(1UL, p->actPropConst[i]) * 1e-11f;
        r->actMaxT[i]         = MAX(1UL, p->actMaxT[i]) * 0.01f;
        r->actNonlinearity[i] = constrainu(p->actNonlinearity[i], 0, 100) * 0.01f;
        r->actLimit[i]        = constrainu(p->actLimit[i], 0, 100) * 0.01f;
        r->actG1[0][i] = p->actG1_fx[i] * 0.01f;
        r->actG1[1][i] = p->actG1_fy[i] * 0.01f;
        r->actG1[2][i] = p->actG1_fz[i] * 0.01f;
        r->actG1[3][i] = p->actG1_roll[i]  * 0.1f;
        r->actG1[4][i] = p->actG1_pitch[i] * 0.1f;
        r->actG1[5][i] = p->actG1_yaw[i]   * 0.01f;
        r->actG2[0][i] = p->actG2_roll[i] / hoverRpm * 0.1f;
        r->actG2[1][i] = p->actG2_pitch[i] / hoverRpm * 0.1f;
        r->actG2[2][i] = p->actG2_yaw[i] / hoverRpm * 0.1f;
        // ---- WLS config
        r->wlsWu[i] = (float) p->wlsWu[i];
        r->u_pref[i] = p->u_pref[i] * 0.01f;
    }
    for (int i = 0; i < MAXV; i++)
        r->wlsWv[i] = (float) p->wlsWv[i];

    // ---- Filtering config
    r->imuSyncLp2Hz = (float) constrainu(p->imuSyncLp2Hz, 1, 5e5 / gyro.targetLooptime);
    // ---- WLS config
    r->wlsAlgo = (activeSetAlgoChoice) p->wlsAlgo;
    r->useWls = (bool) p->useWls;
    r->wlsWarmstart = (bool) p->wlsWarmstart;
    r->wlsMaxIter = p->wlsMaxIter;
    r->wlsCondBound = p->wlsCondBound * 1e4f;
    r->wlsTheta = p->wlsTheta * 1e-4;
    r->wlsNanLimit = p->wlsNanLimit;

    // ---- runtime values -- actauators
    for (int i = 0; i < r->actNum; i++) {
        r->d[i] = 0.f; // command issued to the actuators on [-1, 1] scale
        r->u[i] = 0.f; // control variable proportional to output force, but on [-1, 1], for motors [0, 1]
        r->uState[i] = 0.f; // estimated force state of the actuators [-1, 1]
        r->uState_fs[i] = 0.f; // sync-filtered estiamted force state
        updateLinearization(&r->lin[i], r->actNonlinearity[i]);
        r->omega[i] = 0.f; // unfiltered motor speed rad/s
        r->omega_fs[i] = 0.f; // sync-filtered motor speed rad/s
        //r->omegaDot[i] = 0.f; // unfiltered motor rate rad/s/s
        r->omegaDot_fs[i] = 0.f; // sync-filtered motor rate rad/s/s
        r->G2_normalizer[i] = 1.f / (2.f * r->actTimeConstS[i] * r->actPropConst[i]);
    }
    r->erpmToRads = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.f) * (2.f * M_PIf);
    // ---- runtime values -- axes
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        r->attGainsCasc.A[axis] = r->attGains.A[axis] / r->rateGains.A[axis]; // attitude gains simulating parallel PD
        r->rateSpBody.A[axis] = 0.; // rate setpoint in body coordinates
        r->rateDotSpBody.A[axis] = 0.; // rate derivative setpoint in body coordinates
        r->spfSpBody.A[axis] = 0.; // specific force setpoint in body coordinates
        //r->rate_fs.A[axis] = 0.; // sync-filtered gyro in rad/s
        r->rateDot.A[axis] = 0.; // unfiltered gyro derivative in rad/s/s
        r->rateDot_fs.A[axis] = 0.; // sync-filtered gyro derivative in rad/s/s
        r->spf_fs.A[axis] = 0.; // sync-filterd accelerometer (specific force) in N/kg
    }
    r->attSpNed = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    r->attErrBody = (const fp_quaternion_t) { 1.f, 0.f, 0.f, 0.f };
    for (int j = 0; j < MAXV; j++)
        r->dv[MAXV] = 0.f; // delta-pseudo controls in N/kg and Nm/(kgm^2)

    // ---- housekeeping
    r->dT = gyro.targetLooptime * 1e-6f; // target looptime in S
    r->indiFrequency = 1.0f / gyro.targetLooptime; // target looptime in S
    r->attExecCounter = 0; // count executions (wrapping)
    r->nanCounter = 0; // count times consequtive nans appear in allocation

    // ---- control law selection
    r->bypassControl = false; // no control at all. u and d are unmodified by loop
    r->controlAttitude = true; // attempt to reach tilt given by attSpNed
    r->trackAttitudeYaw = false; // also attempt to reach yaw given by attSpNed

    // ---- filters
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        biquadFilterInitLPF(&r->rateFilter[axis], r->imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&r->spfFilter[axis], r->imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
    }
    for (int i = 0; i < r->actNum; i++) {
        pt1FilterInit(&r->uLagFilter[i], pt1FilterGain(1.f / (2.f * M_PIf * r->actTimeConstS[i]), r->dT)); // to simulate spinup
        biquadFilterInitLPF(&r->uStateFilter[i], r->imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
        biquadFilterInitLPF(&r->omegaFilter[i], r->imuSyncLp2Hz, gyro.targetLooptime); // only support 2nd order butterworth second order section for now
    }
}

#endif // ifdef USE_INDI
