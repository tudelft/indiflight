
#include "drivers/time.h"
#include "common/maths.h"

#ifdef USE_POS_CTL

typedef enum {
    LOCAL_POS_NO_SIGNAL,
    LOCAL_POS_STILL_VALID,
    LOCAL_POS_NEW_MESSAGE,
} local_pos_measurement_state_t;

// todo: reformulate using t_fp_vector
typedef struct __local_pos_ned_t {
    fp_vector_t pos;
    fp_vector_t vel;
    fp_euler_t att;
} local_pos_ned_t;

typedef struct __pos_setpoint_ned_t {
    uint32_t time_ms;
    uint8_t mode;
    fp_vector_t pos;
    fp_vector_t vel;
    float psi;
    float vel_traj;
} local_pos_setpoint_ned_t;

extern local_pos_ned_t posMeasNed;
extern local_pos_measurement_state_t posMeasState;
extern local_pos_setpoint_ned_t posSpNed;
extern local_pos_measurement_state_t posSpState;
extern timeUs_t posLatestMsgTime;

#define POS_MEAS_TIMEOUT_US 300000

#define POS_SETPOINT_OUTDATED_US 1000000

void getLocalPos(timeUs_t current);
void getFakeGps(timeUs_t current);
void getPosSetpoint(timeUs_t current);
#endif
