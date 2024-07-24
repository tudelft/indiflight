
#include "drivers/time.h"
#include "common/maths.h"

#ifdef USE_GPS_PI

typedef enum {
    EXT_POS_NO_SIGNAL,
    EXT_POS_STILL_VALID,
    EXT_POS_NEW_MESSAGE,
} ext_pos_state_t;

// todo: reformulate using t_fp_vector
typedef struct __ext_pos_ned_t {
    uint32_t time_ms;
    t_fp_vector pos;
    t_fp_vector vel;
    fp_angles_t att;
} ext_pos_ned_t;

typedef struct __vio_pos_ned_t {
    uint32_t time_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float p;
    float q;
    float r;
    float qw;
    float qx;
    float qy;
    float qz;
} vio_pos_ned_t;

typedef struct __pos_setpoint_ned_t {
    t_fp_vector pos;
    t_fp_vector vel;
    float psi;
} pos_setpoint_ned_t;

// structs used for EXTERNAL_POSE message
extern ext_pos_ned_t extPosNed;
extern ext_pos_state_t extPosState;
extern timeUs_t extLatestMsgTime;

// structs used for VIO_POSE message
#ifdef USE_VIO_POSE
extern vio_pos_ned_t vioPosNed;
extern ext_pos_state_t vioPosState;
extern timeUs_t vioLatestMsgTime;
#endif

// structs used for POS_SETPOINT message
extern pos_setpoint_ned_t posSetpointNed;
extern ext_pos_state_t posSetpointState;

#define EXT_POS_FREQ 50
#define EXT_POS_TIMEOUT_US 300000

#ifdef USE_VIO_POSE
#define VIO_POS_TIMEOUT_US 300000
#endif

#define POS_SETPOINT_OUTDATED_US 1000000

void checkNewPos(void);
void getExternalPos(timeUs_t current);
#ifdef USE_VIO_POSE
void checkNewVioPos(void);
void getVioPos(timeUs_t current);
#endif
void getFakeGps(timeUs_t current);
void getPosSetpoint(timeUs_t current);
#endif