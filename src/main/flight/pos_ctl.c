
#include "pos_ctl.h"

#include "io/external_pos.h"
#include "flight/imu.h"
#include "common/maths.h"
#include "fc/runtime_config.h"

#ifdef USE_POS_CTL

#ifndef USE_GPS_PI
#error "USE_POS_CTL can currently only be used with USE_GPS_PI"
#endif

#ifndef USE_INDI
#pragma message "USE_POS_CTL currently only has any effect with USE_INDI"
#endif

// --- control variables
// externs
t_fp_vector accSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};
float yawRateSpFromOuter = {0};

// locals
//t_fp_vector posSpNed = {.V.X = 0., .V.Y = 0., .V.Z = -1.};
//t_fp_vector velSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};
//float yawSpNedRad = 0.;

// position controller configuration
// todo DONE: split in horizontal/vertical, not XYZ.. thats kinda meaningless with yaw
float posHGainP = 3.;
float posHGainD = 5.;
float velHGainI = 0.2;
float posVGainP = 3.0;
float posVGainD = 5.;
float velVGainI = 0.2;
float velSpLimitXY = 5.;
float velSpLimitZ  = 2.;
float yawGainP = 8.;
float weathervaneP = 0.;

t_fp_vector velIError = {0};

static void resetIterms(void) { velIError.V.X=0.f; velIError.V.Y=0.f; velIError.V.Z=0.f; }

void updatePosCtl(timeUs_t current) {

    if (extPosState == EXT_POS_NO_SIGNAL) {
        // panic and level craft
        resetIterms();
        accSpNed.V.X = 0.f;
        accSpNed.V.Y = 0.f;
        accSpNed.V.Z = 0.5f; // also command slight downwards acceleration
        return;
    }

    if ( (!ARMING_FLAG(ARMED)) || (!FLIGHT_MODE(POSITION_MODE | VELOCITY_MODE | GPS_RESCUE_MODE)) ) {
        resetIterms();
    }

    getAccSpNed(current);
}

void getAccSpNed(timeUs_t current) {
    // precalculations
    float accMax = 9.80665f * tan_approx( DEGREES_TO_RADIANS( 40.f ) );
    float posHGainPCasc = posHGainP / posHGainD; // emulate parallel PD with Casc system
    float posVGainPCasc = posVGainP / posVGainD;

    // pos error = pos setpoint - pos estimate
    t_fp_vector posError = posSetpointNed.pos;
    VEC3_SCALAR_MULT_ADD(posError, -1.0f, posEstNed); // extPosNed.pos

    // vel setpoint = posGains * posError
    posSetpointNed.vel.V.X = posError.V.X * posHGainPCasc;
    posSetpointNed.vel.V.Y = posError.V.Y * posHGainPCasc;
    posSetpointNed.vel.V.Z = posError.V.Z * posVGainPCasc;

    // constrain magnitude here
    VEC3_CONSTRAIN_XY_LENGTH(posSetpointNed.vel, velSpLimitXY);

    posSetpointNed.vel.V.Z = constrainf(posSetpointNed.vel.V.Z, -velSpLimitZ, +velSpLimitZ);

    // vel error = vel setpoint - vel estimate
    t_fp_vector velError = posSetpointNed.vel;
    //VEC3_SCALAR_MULT_ADD(velError, -1.0f, extPosNed.vel);
    VEC3_SCALAR_MULT_ADD(velError, -1.0f, velEstNed);

    static bool accSpXYSaturated = true;
    static bool accSpZSaturated = true;
    static timeUs_t lastCall = 0;
    timeDelta_t delta = cmpTimeUs(current, lastCall);
    if ((lastCall > 0) && (delta > 0) && (delta < 50000)) {
        if (!accSpXYSaturated) {
            velIError.V.X += delta * 1e-6f * velError.V.X;
            velIError.V.Y += delta * 1e-6f * velError.V.Y;
        }

        if (!accSpZSaturated)
            velIError.V.Z += delta * 1e-6f * velError.V.Z;

        VEC3_CONSTRAIN_XY_LENGTH(velIError, 2.f);
        velIError.V.Z = constrainf(velIError.V.Z, -1.f, 1.f);
    }
    lastCall = current;

    // acceleration setpoint = velGains * velError
    accSpNed.V.X = velError.V.X * posHGainD  +  velIError.V.X * velHGainI;
    accSpNed.V.Y = velError.V.Y * posHGainD  +  velIError.V.Y * velHGainI;
    accSpNed.V.Z = velError.V.Z * posVGainD  +  velIError.V.Z * velVGainI;

    // limit such that max acceleration likely results in bank angle below 40 deg
    // but log if acceleration saturated, so we can pause error integration
    accSpXYSaturated = VEC3_XY_LENGTH(accSpNed) > accMax;
    accSpZSaturated = (accSpNed.V.Z < MAX_ACC_Z_NEG) || (accSpNed.V.Z > MAX_ACC_Z_POS);

    VEC3_CONSTRAIN_XY_LENGTH(accSpNed, accMax);
    accSpNed.V.Z = constrainf(accSpNed.V.Z, MAX_ACC_Z_NEG, MAX_ACC_Z_POS);

    // todo: how to handle course/yaw?
    float yawError = posSetpointNed.psi - DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    if (yawError > M_PIf)
        yawError -= 2.f * M_PIf;
    else if (yawError < -M_PIf)
        yawError += 2.f * M_PIf;

    yawRateSpFromOuter = yawError;
    // todo: weathervaning
    // DONE todo: transform acc measurement from body to global and use INDI on acc --> settled for PI on vel and NDI on acc
}

// TODO
// 1. velocity control..

#endif // USE_POS_CTL
