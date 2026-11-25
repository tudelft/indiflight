/*
 * This file is part of Indiflight.
 *
 * Blackbox header lines of indiflight settings. Textually included inside
 * the switch of blackboxWriteSysinfo() in blackbox/blackbox.c, which
 * provides indiProfile.
 */

#ifdef USE_INDI
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_GAINS, "%d,%d,%d",                indiProfile->attGains[0],
                                                                                              indiProfile->attGains[1],
                                                                                              indiProfile->attGains[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RATE_GAINS, "%d,%d,%d",                    indiProfile->rateGains[0],
                                                                                              indiProfile->rateGains[1],
                                                                                              indiProfile->rateGains[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_MAX_TILT_RATE, "%d",              indiProfile->attMaxTiltRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATTITUDE_MAX_YAW_RATE, "%d",               indiProfile->attMaxYawRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MAX_RATE_SETPOINT, "%d,%d,%d",             indiProfile->maxRateSp[0],
                                                                                              indiProfile->maxRateSp[1],
                                                                                              indiProfile->maxRateSp[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_USE_COORDINATED_YAW, "%d",          indiProfile->manualUseCoordinatedYaw);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_MAX_UPWARDS_ACCEL, "%d",            indiProfile->manualMaxUpwardsSpf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_MANUAL_MAX_TILT, "%d",                     indiProfile->manualMaxTilt);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_INCREMENT, "%d",                       indiProfile->useIncrement);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_ACCEL_FOR_SPFZ, "%d",                  indiProfile->useAccelForSpfz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_RPM_DOT_FEEDBACK, "%d",                indiProfile->useRpmDotFeedback);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_RPM_CTL, "%d",                         indiProfile->useRpmCtl);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RPM_CTL_GAIN, "%d",                        indiProfile->rpmCtlGain);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RPM_CTL_DELAY, "%d",                       indiProfile->rpmCtlDelay);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RPM_CTL_LPF_HZ, "%d",                      indiProfile->rpmCtlLpfHz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_NUM, "%d",                             indiProfile->actNum);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_TIME_CONSTANT_MS, "%d,%d,%d,%d",       indiProfile->actTimeConstMs[0],
                                                                                              indiProfile->actTimeConstMs[1],
                                                                                              indiProfile->actTimeConstMs[2],
                                                                                              indiProfile->actTimeConstMs[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_MAX_RPM, "%d,%d,%d,%d",                indiProfile->actMaxRpm[0],
                                                                                              indiProfile->actMaxRpm[1],
                                                                                              indiProfile->actMaxRpm[2],
                                                                                              indiProfile->actMaxRpm[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_NONLINEARITY, "%d,%d,%d,%d",           indiProfile->actNonlinearity[0],
                                                                                              indiProfile->actNonlinearity[1],
                                                                                              indiProfile->actNonlinearity[2],
                                                                                              indiProfile->actNonlinearity[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_MIN, "%d,%d,%d,%d",                    indiProfile->actMin[0],
                                                                                              indiProfile->actMin[1],
                                                                                              indiProfile->actMin[2],
                                                                                              indiProfile->actMin[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_MAX, "%d,%d,%d,%d",                    indiProfile->actMax[0],
                                                                                              indiProfile->actMax[1],
                                                                                              indiProfile->actMax[2],
                                                                                              indiProfile->actMax[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_TYPE, "%d,%d,%d,%d",                   indiProfile->actType[0],
                                                                                              indiProfile->actType[1],
                                                                                              indiProfile->actType[2],
                                                                                              indiProfile->actType[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FX, "%d,%d,%d,%d",                  indiProfile->actG1_fx[0],
                                                                                              indiProfile->actG1_fx[1],
                                                                                              indiProfile->actG1_fx[2],
                                                                                              indiProfile->actG1_fx[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FY, "%d,%d,%d,%d",                  indiProfile->actG1_fy[0],
                                                                                              indiProfile->actG1_fy[1],
                                                                                              indiProfile->actG1_fy[2],
                                                                                              indiProfile->actG1_fy[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_FZ, "%d,%d,%d,%d",                  indiProfile->actG1_fz[0],
                                                                                              indiProfile->actG1_fz[1],
                                                                                              indiProfile->actG1_fz[2],
                                                                                              indiProfile->actG1_fz[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_ROLL, "%d,%d,%d,%d",                indiProfile->actG1_roll[0],
                                                                                              indiProfile->actG1_roll[1],
                                                                                              indiProfile->actG1_roll[2],
                                                                                              indiProfile->actG1_roll[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_PITCH, "%d,%d,%d,%d",               indiProfile->actG1_pitch[0],
                                                                                              indiProfile->actG1_pitch[1],
                                                                                              indiProfile->actG1_pitch[2],
                                                                                              indiProfile->actG1_pitch[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G1_YAW, "%d,%d,%d,%d",                 indiProfile->actG1_yaw[0],
                                                                                              indiProfile->actG1_yaw[1],
                                                                                              indiProfile->actG1_yaw[2],
                                                                                              indiProfile->actG1_yaw[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_ROLL, "%d,%d,%d,%d",                indiProfile->actG2_roll[0],
                                                                                              indiProfile->actG2_roll[1],
                                                                                              indiProfile->actG2_roll[2],
                                                                                              indiProfile->actG2_roll[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_PITCH, "%d,%d,%d,%d",               indiProfile->actG2_pitch[0],
                                                                                              indiProfile->actG2_pitch[1],
                                                                                              indiProfile->actG2_pitch[2],
                                                                                              indiProfile->actG2_pitch[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_G2_YAW, "%d,%d,%d,%d",                 indiProfile->actG2_yaw[0],
                                                                                              indiProfile->actG2_yaw[1],
                                                                                              indiProfile->actG2_yaw[2],
                                                                                              indiProfile->actG2_yaw[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_USE_SCHEDULED, "%d",                 indiProfile->tails_use_scheduled);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_USE_SINE     , "%d",                 indiProfile->tails_use_sine);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_D0           , "%d,%d",              indiProfile->tails_d0[0], indiProfile->tails_d0[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CXW          , "%d,%d",              indiProfile->tails_cxw[0], indiProfile->tails_cxw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CYW          , "%d,%d",              indiProfile->tails_cyw[0], indiProfile->tails_cyw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CZW          , "%d,%d",              indiProfile->tails_czw[0], indiProfile->tails_czw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CLW          , "%d,%d",              indiProfile->tails_clw[0], indiProfile->tails_clw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CMW          , "%d,%d",              indiProfile->tails_cmw[0], indiProfile->tails_cmw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CNW          , "%d,%d",              indiProfile->tails_cnw[0], indiProfile->tails_cnw[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CNWD         , "%d,%d",              indiProfile->tails_cnwd[0], indiProfile->tails_cnwd[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CXD          , "%d,%d",              indiProfile->tails_cxd[0], indiProfile->tails_cxd[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CMD          , "%d,%d",              indiProfile->tails_cmd[0], indiProfile->tails_cmd[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_TAILS_CND          , "%d,%d",              indiProfile->tails_cnd[0], indiProfile->tails_cnd[1]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_SYNC_LOWPASS_HZ, "%d",                     indiProfile->imuSyncLp2Hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_AXES_WEIGHTS, "%d,%d,%d,%d,%d,%d",     indiProfile->wlsWv[0],
                                                                                              indiProfile->wlsWv[1],
                                                                                              indiProfile->wlsWv[2],
                                                                                              indiProfile->wlsWv[3],
                                                                                              indiProfile->wlsWv[4],
                                                                                              indiProfile->wlsWv[5]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_ACT_PENALTIES, "%d,%d,%d,%d",          indiProfile->wlsWu[0],
                                                                                              indiProfile->wlsWu[1],
                                                                                              indiProfile->wlsWu[2],
                                                                                              indiProfile->wlsWu[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_WLS_ACT_PREFERRED_STATE, "%d,%d,%d,%d",    indiProfile->u_pref[0],
                                                                                              indiProfile->u_pref[1],
                                                                                              indiProfile->u_pref[2],
                                                                                              indiProfile->u_pref[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_FF_FILTER, "%d",                       indiProfile->useFeedforwardFilter);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_FF_FILTER_COEFS, "%d,%d,%d",               indiProfile->feedforwardFilterCoefs[0],
                                                                                              indiProfile->feedforwardFilterCoefs[1],
                                                                                              indiProfile->feedforwardFilterCoefs[2]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_FF_COEFS, "%d,%d,%d,%d,%d",                indiProfile->feedforwardCoefs[0],
                                                                                              indiProfile->feedforwardCoefs[1],
                                                                                              indiProfile->feedforwardCoefs[2],
                                                                                              indiProfile->feedforwardCoefs[3],
                                                                                              indiProfile->feedforwardCoefs[4]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_MOTOR_LEAD_LAG, "%d",                  indiProfile->useMotorLeadLag);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_ATT_LEAD_LAG, "%d",                    indiProfile->useAttLeadLag);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ATT_COEFS, "%d,%d,%d,%d,%d",               indiProfile->attCoefs[0],
                                                                                              indiProfile->attCoefs[1],
                                                                                              indiProfile->attCoefs[2],
                                                                                              indiProfile->attCoefs[3],
                                                                                              indiProfile->attCoefs[4]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_RATE_COEFS, "%d,%d,%d,%d,%d",              indiProfile->rateCoefs[0],
                                                                                              indiProfile->rateCoefs[1],
                                                                                              indiProfile->rateCoefs[2],
                                                                                              indiProfile->rateCoefs[3],
                                                                                              indiProfile->rateCoefs[4]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_USE_GAIN_SCHEDULING, "%d",                 indiProfile->useGainScheduling);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_GAIN_SCHEDULING_TYPE, "%d",                indiProfile->gainSchedulingType);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_GAIN_SCHEDULE_FF, "%d",                    indiProfile->gainScheduleFf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_TIME_CONSTANT_TRUE_MS, "%d,%d,%d,%d",  indiProfile->actTimeConstTrueMs[0],
                                                                                              indiProfile->actTimeConstTrueMs[1],
                                                                                              indiProfile->actTimeConstTrueMs[2],
                                                                                              indiProfile->actTimeConstTrueMs[3]);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_INDI_ACT_TIME_CONSTANT_DES_MS, "%d,%d,%d,%d",   indiProfile->actTimeConstDesMs[0],
                                                                                              indiProfile->actTimeConstDesMs[1],
                                                                                              indiProfile->actTimeConstDesMs[2],
                                                                                              indiProfile->actTimeConstDesMs[3]);
#endif // USE_INDI
