
#include "common/maths.h"
#include "common/time.h"

//FIXME: are box shaped constraints what we want?
#define MAX_ACC_Z_NEG -20.f // around 2.0g
#define MAX_ACC_XY 20.f // around 2.0g

#ifndef MAX_ACC_Z_POS
#define MAX_ACC_Z_POS +9.5f // exactly 1g, so maximum commanded descend is falling
//#define MAX_ACC_Z_POS +20.f // haha, inverted go brr
#endif

//extern t_fp_vector posSpNed;
//extern t_fp_vector velSpNed;
extern t_fp_vector accSpNed;
extern float yawRateSpFromOuter;

typedef struct positionProfile_s {
    uint8_t horz_p; // m/s/s per m * 10
    uint8_t horz_i; // m/s/s per m/s * s  * 10
    uint8_t horz_d; // m/s/s per m/s * 10
    uint16_t horz_max_v; // cm/s 
    uint16_t horz_max_a; // cm/s/s
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
} positionProfile_t;

typedef struct positionRuntime_s {
    float horz_p; // m/s/s per m 
    float horz_i; // m/s/s per m/s * s
    float horz_d; // m/s/s per m/s
    float horz_max_v; // m/s
    float horz_max_a; // m/s/s
    float vert_p; // m/s/s per m
    float vert_i; // m/s/s per m/s * s
    float vert_d; // m/s/s per m/s
    float vert_max_v_up; // m/s 
    float vert_max_v_down; // m/s 
    float vert_max_a_up; // m/s/s
    float vert_max_a_down; // m/s/s
    float yaw_p; // deg/s per deg
    float weathervane_p; // deg/s per deg
    float weathervane_min_v; // m/s min speed to use weathervaneing
} positionRuntime_t;

extern positionRuntime_t posRuntime;
void initPositionRuntime(positionProfile_t *p);

#define POSITION_PROFILE_COUNT 3
PG_DECLARE_ARRAY(positionProfile_t, POSITION_PROFILE_COUNT, positionProfiles);


void updatePosCtl(timeUs_t current);
void getAccSpNed(timeUs_t current);
