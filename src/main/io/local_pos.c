/*
 * Get local NED position from difference sources (uplink/GPS)
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


#include "local_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"
#include "io/local_pos.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "sensors/sensors.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "flight/geofence.h"

#include "telemetry/pi.h"
#include "telemetry/uros.h"
#include "io/gps.h"

#ifdef USE_LOCAL_POSITION

#ifndef USE_ACC
#error "USE_EKF requires USE_ACC"
#endif

#ifndef USE_GYRO
#error "USE_EKF requires USE_GYRO"
#endif

#ifndef USE_EKF
#error "USE_LOCAL_POSITION must be used with USE_EKF"
#endif

//extern
local_pos_ned_t posMeasNed;
local_pos_sp_ned_t posSpNed;

void setLocalPosMeas(local_pos_ned_t* pos) {
    if ( (cmpTimeUs(pos->time_us, posMeasNed.time_us) > 0)
            && ( pos->source == ekfConfig()->meas_source
                || pos->source == LOCAL_POS_SOURCE_MOCKUP ) // always accept MOCKUP
            ) {
        posMeasNed = *pos;
        posMeasNed.new = true;
#ifdef USE_GPS
        sensorsSet(SENSOR_GPS);
#endif
#if defined(USE_GPS) && defined(MOCKUP)
        // this is just for debugging geofence
        if (pos->source != LOCAL_POS_SOURCE_GPS) {
            // transform local to global, set and check for geofence
            gpsLocation_t home = { 519906500, 43766250, 0 };
            gpsLocation_t llh;
            local_to_llh(&posMeasNed.pos, &home, &llh);
            gpsSol.llh = llh;
#ifdef USE_GEOFENCE
            geofenceUpdate(&gpsSol.llh);
#endif
        }
#endif
    }
}

void setLocalPosSp(local_pos_sp_ned_t* sp) {
    if (cmpTimeUs(sp->time_us, posSpNed.time_us) < 0) {
        return;
    }
    posSpNed = *sp;
    posSpNed.valid = true;
    setSticksReference();
}

void setLocalPosSpHere(void) {
    if (isConvergedEkf()) {
        local_pos_sp_ned_t sp;
        sp.time_us = micros();
        sp.pos = posEstNed;
        sp.trackPsi = false;
        setLocalPosSp(&sp);
    }
}

#ifdef USE_GPS
void llh_to_local(const gpsLocation_t* llh, const gpsLocation_t* home, fp_vector_t* ned) {
    ned->V.X = 1e-7f * DEGREES_TO_RADIANS(llh->lat - home->lat) * REARTHf;
    ned->V.Y = 1e-7f * DEGREES_TO_RADIANS(llh->lon - home->lon) * REARTHf
        * cosf(1e-7f * DEGREES_TO_RADIANS(home->lat));
    ned->V.Z = -1e-2f * (llh->altCm - home->altCm);
}
void local_to_llh(const fp_vector_t* ned, const gpsLocation_t* home, gpsLocation_t* llh) {
    // Reverse the conversion of NED to LLH
    llh->lat = home->lat + 1e7f * RADIANS_TO_DEGREES(ned->V.X / REARTHf);
    llh->lon = home->lon + 1e7f * RADIANS_TO_DEGREES(ned->V.Y / (REARTHf * cosf(1e-7f * DEGREES_TO_RADIANS(home->lat))) );
    llh->altCm = home->altCm - 100 * ned->V.Z;  // Convert back from meters to centimeters
}
#endif

#endif
