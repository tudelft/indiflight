#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"

// STATIC_UNIT_TESTED in position_estimator.c — gravity-removed earth-frame
#ifdef USE_FRD_NED
// linear acceleration in NED (cm/s^2).
void getLinearAccelNED(float *accelNorth, float *accelEast, float *accelDown);
#else
// linear acceleration in ENU (cm/s^2).
void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp);
#endif // USE_FRD_NED

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
}

#include "gtest/gtest.h"

extern "C" {
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

acc_t acc;
matrix33_t rMat;
gpsSolutionData_t gpsSol;

static uint32_t enabledSensors = 0;
static bool rfHealthy = false;
static float rfAltCm = 0.0f;
static float baroAltCm = 0.0f;
static timeUs_t fakeMicros = 0;

bool sensors(uint32_t mask) { return (enabledSensors & mask) != 0; }
bool rangefinderIsHealthy(void) { return rfHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return lrintf(rfAltCm); }
float getBaroAltitude(void) { return baroAltCm; }

timeUs_t micros(void) { return fakeMicros; }

bool gpsHasNewData(uint16_t *gpsStamp)
{
    (*gpsStamp)++;
    return true;
}

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *dest)
{
    // Simplified (no Earth-scale projection): return integer-unit deltas so sign tests work.
#ifdef USE_FRD_NED
    dest->v[EF_EAST] = (float)(to->lon - from->lon);
    dest->v[EF_NORTH] = (float)(to->lat - from->lat);
#else
    dest->x = (float)(to->lon - from->lon);
    dest->y = (float)(to->lat - from->lat);
#endif // USE_FRD_NED
}
}

static void stepEstimator(unsigned count = 1)
{
    for (unsigned i = 0; i < count; i++) {
        fakeMicros += 10000; // 100 Hz
        positionEstimatorUpdate();
    }
}

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&acc, 0, sizeof(acc));
        memset(&rMat, 0, sizeof(rMat));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        // Identity rotation and 1G reciprocal scale so zero accel stays zero.
#ifdef USE_FRD_NED
        rMat.m[NED_N][X] = 1.0f;
        rMat.m[NED_E][Y] = 1.0f;
        rMat.m[NED_D][Z] = 1.0f;
#else
        rMat.m[NWU_N][X] = 1.0f;
        rMat.m[NWU_W][Y] = 1.0f;
        rMat.m[NWU_U][Z] = 1.0f;
#endif // USE_FRD_NED
        acc.dev.acc_1G_rec = 1.0f;
#ifdef USE_FRD_NED
        acc.accADC.z = -1.0f; // FRD: level reads -1 G on Z
#else
        acc.accADC.z = 1.0f;
#endif // USE_FRD_NED

        enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
        rfHealthy = true;
        rfAltCm = 100.0f;
        baroAltCm = 100.0f;
        gpsSol.llh.altCm = 100.0f;
        gpsSol.dop.pdop = 100; // pDOP 1.0

        stateFlags = GPS_FIX;
        armingFlags = ARMED;
        fakeMicros = 0;

        positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_PREFER;
        positionConfigMutable()->altitude_prefer_baro = 100;
        positionConfigMutable()->rangefinder_max_range_cm = 400;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, RangefinderPreferFallsBackAndRecovers)
{
    // Establish offsets/baseline near zero.
    stepEstimator(10);
    const float baseline = positionEstimatorGetAltitudeCm();

    // Rangefinder unavailable: fallback should use baro/GPS updates.
    rfHealthy = false;
    baroAltCm = 150.0f;
    gpsSol.llh.altCm = 150.0f;
    stepEstimator(40);
    const float fallbackAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(fallbackAltitude, baseline + 10.0f);

    // Rangefinder becomes available again at a lower relative altitude.
    rfHealthy = true;
    rfAltCm = 120.0f; // +20cm relative to RF offset baseline
    stepEstimator(40);
    const float recoveredAltitude = positionEstimatorGetAltitudeCm();

    // In RANGEFINDER_PREFER, recovered altitude should be pulled down toward RF value.
    EXPECT_LT(recoveredAltitude, fallbackAltitude - 5.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: GPS_distance2d must be called as (arm, current) so that a craft
// East of the arm position produces a positive kfEast (East) position estimate.
// The bug had the arguments reversed — (current, arm) — giving a negative East estimate.
TEST_F(PositionEstimatorTest, GPSPositionEastOfArmIsPositive)
{
    // Run a couple of steps so the arm location is captured from the initial gpsSol (lon=0).
    stepEstimator(2);

    // Place the craft East of the arm (positive longitude delta).
    gpsSol.llh.lon = 100000;

    // Let the Kalman filter converge toward the GPS position measurement.
    stepEstimator(30);

#ifdef USE_FRD_NED
    // position is NED; index by NED_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->position.v[NED_E], 0.0f);
#else
    // position is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->position.v[ENU_E], 0.0f);
#endif // USE_FRD_NED
}

// Build rMat from roll/pitch/yaw using the exact quaternion + rotation-matrix
// formulas from imu.c (imuComputeQuaternionFromRPY + imuComputeRotationMatrix),
#ifdef USE_FRD_NED
// so the matrix carries the firmware's body FRD -> earth NED convention.
#else
// so the matrix carries the firmware's body->earth NWU (North-West-Up) convention.
#endif // USE_FRD_NED
// Yaw follows the attitude/compass convention: 0 = North, +90 = East increases the
// reported heading, so a craft heading East is built with yawDeg = -90.
static void setRMatFromEuler(float rollDeg, float pitchDeg, float yawDeg)
{
    const float hr = DEGREES_TO_RADIANS(rollDeg) * 0.5f;
    const float hp = DEGREES_TO_RADIANS(pitchDeg) * 0.5f;
    const float hy = DEGREES_TO_RADIANS(yawDeg) * 0.5f;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    const float q0 = cr * cp * cy + sr * sp * sy;
    const float q1 = sr * cp * cy - cr * sp * sy;
    const float q2 = cr * sp * cy + sr * cp * sy;
    const float q3 = cr * cp * sy - sr * sp * cy;

    const float xx = q1 * q1, yy = q2 * q2, zz = q3 * q3;
    const float xy = q1 * q2, xz = q1 * q3, yz = q2 * q3;
    const float wx = q0 * q1, wy = q0 * q2, wz = q0 * q3;

#ifdef USE_FRD_NED
    rMat.m[NED_N][X] = 1.0f - 2.0f * yy - 2.0f * zz;
    rMat.m[NED_N][Y] = 2.0f * (xy - wz);
    rMat.m[NED_N][Z] = 2.0f * (xz + wy);
    rMat.m[NED_E][X] = 2.0f * (xy + wz);
    rMat.m[NED_E][Y] = 1.0f - 2.0f * xx - 2.0f * zz;
    rMat.m[NED_E][Z] = 2.0f * (yz - wx);
    rMat.m[NED_D][X] = 2.0f * (xz - wy);
    rMat.m[NED_D][Y] = 2.0f * (yz + wx);
    rMat.m[NED_D][Z] = 1.0f - 2.0f * xx - 2.0f * yy;
#else
    rMat.m[NWU_N][X] = 1.0f - 2.0f * yy - 2.0f * zz;
    rMat.m[NWU_N][Y] = 2.0f * (xy - wz);
    rMat.m[NWU_N][Z] = 2.0f * (xz + wy);
    rMat.m[NWU_W][X] = 2.0f * (xy + wz);
    rMat.m[NWU_W][Y] = 1.0f - 2.0f * xx - 2.0f * zz;
    rMat.m[NWU_W][Z] = 2.0f * (yz - wx);
    rMat.m[NWU_U][X] = 2.0f * (xz - wy);
    rMat.m[NWU_U][Y] = 2.0f * (yz + wx);
    rMat.m[NWU_U][Z] = 1.0f - 2.0f * xx - 2.0f * yy;
#endif // USE_FRD_NED
}

// Regression test for the East-West / magnitude bug discussed in #15321 and #15339.
#ifdef USE_FRD_NED
// getLinearAccelNED must use matrixVectorMul (body FRD -> earth NED);
#else
// getLinearAccelENU must use matrixVectorMul (body->earth NWU) with East = -West;
#endif // USE_FRD_NED
// the previously-used matrixTrnVectorMul applied the inverse rotation, giving the
// wrong magnitude and a spurious North term.
//
// Scenario: heading East, 30 deg right roll, 0.5 g of forward (East) thrust.
// Gravity projects onto body Y and Z because of the roll, so accBF = (0.5, 0.5, 0.866).
#ifdef USE_FRD_NED
// In the correct NED formulation the gravity cross-terms cancel exactly, leaving
#else
// In the correct NWU formulation the gravity cross-terms cancel exactly, leaving
#endif // USE_FRD_NED
// the full 0.5 g of thrust on East and nothing on North or Up.
#ifdef USE_FRD_NED
TEST_F(PositionEstimatorTest, EastThrustProducesEastAccelNED)
#else
TEST_F(PositionEstimatorTest, EastThrustProducesEastAccelENU)
#endif // USE_FRD_NED
{
    constexpr float GRAVITY_CMSS = 980.665f;

    // rMat produced via imuComputeRotationMatrix's convention for heading East + 30 deg roll.
#ifdef USE_FRD_NED
    setRMatFromEuler(30.0f, 0.0f, 90.0f); // NED: heading East = +90 deg about Down
#else
    setRMatFromEuler(30.0f, 0.0f, -90.0f);
#endif // USE_FRD_NED

    // Forward (East) thrust of 0.5 g; gravity projects onto body Y and Z due to the roll.
    acc.dev.acc_1G_rec = 1.0f;
    acc.accADC.x = 0.5f;
#ifdef USE_FRD_NED
    acc.accADC.y = -0.5f;    // FRD: -sin(30) * 1g
    acc.accADC.z = -0.866f;  // FRD: -cos(30) * 1g
#else
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g
#endif // USE_FRD_NED

#ifdef USE_FRD_NED
    float accelEast, accelNorth, accelDown;
    getLinearAccelNED(&accelNorth, &accelEast, &accelDown);
#else
    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);
#endif // USE_FRD_NED

    EXPECT_NEAR(accelEast,  0.5f * GRAVITY_CMSS, 0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelNorth, 0.0f,                0.01f * GRAVITY_CMSS);
#ifdef USE_FRD_NED
    EXPECT_NEAR(accelDown,  0.0f,                0.01f * GRAVITY_CMSS);
#else
    EXPECT_NEAR(accelUp,    0.0f,                0.01f * GRAVITY_CMSS);
#endif // USE_FRD_NED
}

// Integration counterpart: with the same attitude/thrust, the IMU prediction alone
// (GPS held at the origin) must drive a positive East velocity estimate.
TEST_F(PositionEstimatorTest, EastThrustProducesPositiveEastVelocity)
{
    // Run a couple of steps so XY fusion is established with the arm at origin.
    stepEstimator(2);

#ifdef USE_FRD_NED
    setRMatFromEuler(30.0f, 0.0f, 90.0f); // NED: heading East = +90 deg about Down
#else
    setRMatFromEuler(30.0f, 0.0f, -90.0f);
#endif // USE_FRD_NED

    acc.accADC.x = 0.5f;
#ifdef USE_FRD_NED
    acc.accADC.y = -0.5f;    // FRD: -sin(30) * 1g
    acc.accADC.z = -0.866f;  // FRD: -cos(30) * 1g
#else
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g
#endif // USE_FRD_NED

    // GPS position and velocity remain zero so any non-zero velocity estimate is
    // driven purely by the IMU prediction.
    stepEstimator(100);

#ifdef USE_FRD_NED
    // velocity is NED; index by NED_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->velocity.v[NED_E], 0.0f);
#else
    // velocity is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->velocity.v[ENU_E], 0.0f);
#endif // USE_FRD_NED
}

