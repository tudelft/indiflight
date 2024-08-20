
#include "common/maths.h"
#include "common/time.h"
#include "config/config.h"

#pragma once

typedef struct positionProfile_s {
    uint8_t horz_p; // m/s/s per m * 10
    uint8_t horz_i; // m/s/s per m/s * s  * 10
    uint8_t horz_d; // m/s/s per m/s * 10
    uint16_t horz_max_v; // cm/s 
    uint16_t horz_max_a; // cm/s/s  FIXME: are box shaped constraints what we want?
    uint8_t max_tilt;   // in automatic flight in deg (0, 180)
    uint8_t vert_p; // m/s/s per m * 10
    uint8_t vert_i; // m/s/s per m/s * s  * 10
    uint8_t vert_d; // m/s/s per m/s * 10
    uint16_t vert_max_v_up; // cm/s 
    uint16_t vert_max_v_down; // cm/s 
    uint16_t vert_max_a_up; // cm/s/s
    uint16_t vert_max_a_down; // cm/s/s
    uint8_t yaw_p; // deg/s per deg * 10
    uint8_t weathervane_p; // deg/s per deg * 10
    uint16_t weathervane_min_v; // cm/s min speed to use weathervaneing
    uint8_t use_spf_attenuation;  // bool: enable correcting thrust setpoint if target attitude has not yet been reached
    // --- inaccessible from CLI / dump files for now
    uint16_t vert_max_iterm; // cm/s
    uint16_t horz_max_iterm; // cm/s
} positionProfile_t;

typedef struct positionRuntime_s {
    float horz_p; // m/s/s per m 
    float horz_i; // m/s/s per m/s * s
    float horz_d; // m/s/s per m/s
    float horz_max_v; // m/s
    float horz_max_a; // m/s/s
    float max_tilt;   // rad
    float horz_max_iterm; // m/s
    float vert_p; // m/s/s per m
    float vert_i; // m/s/s per m/s * s
    float vert_d; // m/s/s per m/s
    float vert_max_v_up; // m/s 
    float vert_max_v_down; // m/s 
    float vert_max_a_up; // m/s/s
    float vert_max_a_down; // m/s/s
    float vert_max_iterm; // m/s
    float yaw_p; // deg/s per deg
    float weathervane_p; // deg/s per deg
    float weathervane_min_v; // m/s min speed to use weathervaneing
    bool use_spf_attenuation;
} positionRuntime_t;

#define POSITION_PROFILE_COUNT 3
PG_DECLARE_ARRAY(positionProfile_t, POSITION_PROFILE_COUNT, positionProfiles);

extern positionRuntime_t posRuntime;
void initPositionRuntime();
void changePositionProfile(uint8_t profileIndex);

#define DEADRECKONING_TIMEOUT_HOLD_POSITION_US 500000
#define DEADRECKONING_TIMEOUT_DESCEND_SLOWLY_US 2000000

extern fp_vector_t accSpNedFromPos;
extern fp_quaternion_t attSpNedFromPos;
extern fp_vector_t spfSpBodyFromPos;
extern fp_vector_t rateSpBodyFromPos;

void resetIterms();
void posCtlInit(void);
void updatePosCtl(timeUs_t current);
void posGetAccSpNed(timeUs_t current);
void posGetAttSpNedAndSpfSpBody(timeUs_t current);
void posGetRateSpBody(timeUs_t current);
