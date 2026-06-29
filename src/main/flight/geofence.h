#include "stdint.h"
#include "stdbool.h"
#include "io/gps.h"

#include "common/time.h"

#define GEOFENCE_MAX_VERTICES 20

typedef struct geofenceConfig_s {
    gpsLocation_t vertices[GEOFENCE_MAX_VERTICES];
    uint8_t numActive;
    int16_t maxAltMeters;
    uint8_t hardFenceOffset;
    uint8_t graceCount;
    uint8_t descendDelaySeconds;
} geofenceConfig_t;

PG_DECLARE(geofenceConfig_t, geofenceConfig);

typedef enum {
    GEOFENCE_STATE_DISABLED = 0,
    GEOFENCE_STATE_GOOD,
    GEOFENCE_STATE_SOFT,
    GEOFENCE_STATE_HARD,
    GEOFENCE_STATE_ERROR,
} geofence_state_e;

typedef enum {
    GEOFENCE_ACTION_NONE = 0,
    GEOFENCE_ACTION_HOLD,
    GEOFENCE_ACTION_DESCEND,
    GEOFENCE_ACTION_KILL,
} geofence_action_e;

extern geofence_state_e geofenceState;
extern geofence_action_e geofenceAction;

void geofenceAddPoint(gpsLocation_t* vertex);
void geofenceInit(void);
void geofenceClearHold(void);
void geofenceUpdate(gpsLocation_t* llh);
