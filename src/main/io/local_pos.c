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
#include "sensors/sensors.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"

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
            && ( pos->source != ekfConfig()->meas_source
                || pos->source == LOCAL_POS_SOURCE_MOCKUP ) // always accept MOCKUP
            ) {
        posMeasNed.new = true;
        posMeasNed = *pos;
    }
}

void setLocalPosSp(local_pos_sp_ned_t* sp) {
    if (cmpTimeUs(sp->time_us, posSpNed.time_us) < 0) {
        return;
    }
    posSpNed.new = true;
    posSpNed = *sp;
}

#endif
