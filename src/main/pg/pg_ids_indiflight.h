/*
 * This file is part of Indiflight.
 *
 * Parameter group numbers of indiflight-owned PGs. Kept in their own block
 * (1000..1099) so they never clash with upstream numbers (upstream took
 * 58..61, which indiflight 4.4 used for INDI/position/catapult/throw).
 * Changing a number here orphans users' stored settings for that PG.
 */

#pragma once

#define PG_INDIFLIGHT_START     1000
#define PG_INDI_PROFILE         1000
#define PG_INDI_CONFIG          1001 // active INDI profile index (was systemConfig_t.indiProfileIndex)
#define PG_POSITION_PROFILE     1002
#define PG_POSITION_CONFIG      1003 // reserved: active position profile index
#define PG_EKF_CONFIG           1004
#define PG_CATAPULT_CONFIG      1005
#define PG_THROW_CONFIG         1006
#define PG_LEARNER_CONFIG       1007
#define PG_NN_CONFIG            1008
#define PG_GEOFENCE_CONFIG      1009
#define PG_PROBER_CONFIG        1010
#define PG_PITOTMETER_CONFIG    1011
#define PG_INDIFLIGHT_END       1099
