
#include "stdbool.h"
#include "string.h"

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "config/config_reset.h"
#include "drivers/time.h"
#include "common/maths.h"
#include "common/time.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "io/local_pos.h"
#include "io/gps.h"

#include "geofence.h"

geofence_state_e geofenceState = GEOFENCE_STATE_ERROR;
geofence_action_e geofenceAction = GEOFENCE_ACTION_NONE;


#ifdef USE_GEOFENCE

#ifndef USE_LOCAL_POSITION
#error "USE_GEOFENCE requires USE_LOCAL_POSITION"
#endif

#ifndef USE_GPS
#error "USE_GEOFENCE requires USE_GPS"
#endif

static unsigned softCounter, hardCounter;


PG_REGISTER_WITH_RESET_FN(geofenceConfig_t, geofenceConfig, PG_GEOFENCE_CONFIG, 1);

void pgResetFn_geofenceConfig(geofenceConfig_t* g) {
    // reset to zeros
    memset(g->vertices, 0, GEOFENCE_MAX_VERTICES * sizeof(gpsLocation_t));
    g->numActive = 0;
    g->maxAltMeters = 20;
    g->hardFenceOffset = 10;
    g->graceCount = 3;
    g->descendDelaySeconds = 20;
}

void geofenceAddPoint(gpsLocation_t* llh) {
    uint8_t i = geofenceConfig()->numActive;

    if (i < GEOFENCE_MAX_VERTICES) {
        geofenceConfigMutable()->vertices[i].lat = llh->lat;
        geofenceConfigMutable()->vertices[i].lon = llh->lon;
        // altitude unused for now
        geofenceConfigMutable()->numActive++;
    }
}

void geofenceInit(void) {
    softCounter = hardCounter = geofenceConfig()->graceCount;

    geofenceState = GEOFENCE_STATE_GOOD;
    geofenceAction = GEOFENCE_ACTION_NONE;

    int n = geofenceConfig()->numActive;
    if (n == 0) {
        geofenceState = GEOFENCE_STATE_DISABLED;
        return;
    } else if (n > 0 && n < 3) {
        geofenceState = GEOFENCE_STATE_ERROR;
        return;
    } else {
        if (!STATE(GPS_FIX)) {
            geofenceState = GEOFENCE_STATE_ERROR;
        }
    }

    // check for self-intersection!
}

static float pointToPointDistance(const fp_vector_t* v, const fp_vector_t* w) {
    return sqrtf(sq(v->V.X - w->V.X) + sq(v->V.Y - w->V.Y));
}

static float pointToSegmentDistance(const fp_vector_t* p, const fp_vector_t* v, const fp_vector_t* w) {
    float px = p->V.X;
    float py = p->V.Y;
    float vx = v->V.X;
    float vy = v->V.Y;
    float wx = w->V.X;
    float wy = w->V.Y;

    float l2 = sq(vx - wx) + sq(vy - wy); // Length squared of the segment
    if (l2 == 0.f) {
        return pointToPointDistance(p, v);  // If the segment is a single point, return the distance to that point
    }

    // Project the point onto the line segment, computing parameter t of the projection
    float t = ( (px - wx) * (vx - wx)  +  (py - wy) * (vy - wy) ) / l2;
    t = constrainf(t, 0.f, 1.f);
    fp_vector_t projection = {.V.X = wx + t * (vx - wx), .V.Y = wy + t * (vy - wy)};

    // Return the distance from the point to the projection
    return pointToPointDistance(p, &projection);
}

static float geofenceDistanceToFence(const gpsLocation_t* fence, const gpsLocation_t* p) {
    int n = MIN(GEOFENCE_MAX_VERTICES, (geofenceConfig()->numActive));

    float minDist = MAXFLOAT;
    fp_vector_t zero = {0};
    for (int i = 0; i < n; i++) {
        const gpsLocation_t* cur = fence+i;
        const gpsLocation_t* next = fence+((i+1) % n);

        // problem with v, w calculations here, or with p
        fp_vector_t v, w;
        llh_to_local(cur, p, &v);
        llh_to_local(next, p, &w);

        float dist = pointToSegmentDistance(&zero, &v, &w);
        minDist = MIN(dist, minDist);
    }

    return minDist;
}

static bool geofenceIsInsideFence(const gpsLocation_t* fence, const gpsLocation_t* p) {
    int n = MIN(GEOFENCE_MAX_VERTICES, (geofenceConfig()->numActive));

    int n_intersec = 0;
    for (int i = 0; i < n; i++) {
        const gpsLocation_t* cur = fence+i;
        const gpsLocation_t* next = fence+((i+1) % n);

        int32_t dlat = (next->lat - cur->lat);
        if (dlat == 0) { continue; } // horizontal edge wont give information

        if ( ((p->lat > cur->lat)   &&  (p->lat <= next->lat))
                || ((p->lat > next->lat)  &&  (p->lat <= cur->lat)) ) {

            int32_t dlon = (next->lon - cur->lon);
            float ratio = (float) dlon / (float) dlat; // hope for no numerical issues
            float dintersec_lon = (float)(p->lat - cur->lat) * ratio;

            if (dintersec_lon > (float)(p->lon - cur->lon)) {
                // intersection of the edge with the ray casted in +x direction 
                // lies to the right of the query point --> we intersected it
                n_intersec++;
            }
        }
    }

    return (n_intersec % 2 == 1); // inside polygon iff odd
}

static float geofenceViolation(gpsLocation_t* p) {
    bool inFence = geofenceIsInsideFence(geofenceConfig()->vertices, p);
    float alt_violation = (0.01f * (float)p->altCm) - (float) geofenceConfig()->maxAltMeters;

    if (inFence && alt_violation <= 0.f) {
        return 0.f;
    } else {
        float xy_violation = geofenceDistanceToFence(geofenceConfig()->vertices, p);
        return MAX(xy_violation, alt_violation); // or vectorsum?
    }
}

static bool geofenceHoldArmed = true;
void geofenceClearHold(void) {
    if (geofenceAction == GEOFENCE_ACTION_HOLD) {
        geofenceAction = GEOFENCE_ACTION_NONE;
    }
}

void geofenceUpdateWatchdog(void) {
    if (geofenceState != GEOFENCE_STATE_DISABLED) {
        if (!STATE(GPS_FIX)) {
            if (ARMING_FLAG(ARMED)) {
                disarm(DISARM_REASON_GEOFENCE);
            }
            geofenceAction = GEOFENCE_ACTION_KILL;
        }
    }
}

void geofenceUpdate(gpsLocation_t* llh) {
    // calculate violation of geofence
    float violation = geofenceViolation(llh);
    static bool inViolation = false;
    static timeUs_t lastViolationAtUs = 0;

    // update/reset grace counters
    if (violation > 0.f) {
        if (!inViolation) {
            inViolation = true;
            lastViolationAtUs = micros();
        }

        if (softCounter) { softCounter--; }

        if (violation >= geofenceConfig()->hardFenceOffset) {
            if (hardCounter) { hardCounter--; }
        }
    } else {
        softCounter = hardCounter = geofenceConfig()->graceCount;
        inViolation = false;
    }

    // manage state, latching any violations
    switch(geofenceState) {
        case GEOFENCE_STATE_DISABLED:
            geofenceHoldArmed = true;
            break;
        case GEOFENCE_STATE_GOOD:
            geofenceAction = GEOFENCE_ACTION_NONE;
            geofenceHoldArmed = true;

            if (softCounter <= 0) { geofenceState = GEOFENCE_STATE_SOFT; }
            if (hardCounter <= 0) { geofenceState = GEOFENCE_STATE_HARD; }

            break;
        case GEOFENCE_STATE_SOFT:
            if (inViolation && cmpTimeUs(micros(), lastViolationAtUs) > 1e6*geofenceConfig()->descendDelaySeconds) {
                geofenceAction = GEOFENCE_ACTION_DESCEND; // 1m/s descebd hardcoded
            } else if (geofenceHoldArmed) {
                geofenceAction = GEOFENCE_ACTION_HOLD;
                geofenceHoldArmed = false;
            }

            if (!inViolation && geofenceAction < GEOFENCE_ACTION_HOLD) {
                // hold cleared, not in DESCEND or KILL
                geofenceState = GEOFENCE_STATE_GOOD;
            }

            if (hardCounter <= 0) { geofenceState = GEOFENCE_STATE_HARD; }

            break;
        case GEOFENCE_STATE_HARD:
            FALLTHROUGH;
        case GEOFENCE_STATE_ERROR:
            if (ARMING_FLAG(ARMED)) {
                disarm(DISARM_REASON_GEOFENCE);
            }
            geofenceAction = GEOFENCE_ACTION_KILL;
            break;
    }
}

#endif
