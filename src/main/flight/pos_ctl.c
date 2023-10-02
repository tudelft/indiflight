

#include "io/external_pos.h"
#include "common/maths.h"

#include "pos_ctl.h"

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
float posHGainP = 0.4;
float posHGainD = 1.5;
float posVGainP = 1.0;
float posVGainD = 2.5;
float velSpLimitXY = 1.5;
float velSpLimitZ  = 0.5;
float weathervaneP = 0.;

void updatePosCtl(timeUs_t current) {
    UNUSED(current);

    if (extPosState == EXT_POS_NO_SIGNAL) {
        // panic and level craft
        accSpNed.V.X = 0.f;
        accSpNed.V.Y = 0.f;
        accSpNed.V.Z = 0.f;
        return;
    }

    getAccSpNed();
}

void getAccSpNed(void) {
    // precalculations
    float accMax = tan_approx( DEGREES_TO_RADIANS( 40.f ) );
    float posHGainPCasc = posHGainP / posHGainD; // emulate parallel PD with Casc system
    float posVGainPCasc = posVGainP / posVGainD;

    // pos error = pos setpoint - pos estimate
    t_fp_vector posError = posSetpointNed.pos;
    VEC3_SCALAR_MULT_ADD(posError, -1.0f, extPosNed.pos);
    // vel setpoint = posGains * posError
    posSetpointNed.vel.V.X = posError.V.X * posHGainPCasc;
    posSetpointNed.vel.V.Y = posError.V.Y * posHGainPCasc;
    posSetpointNed.vel.V.Z = posError.V.Z * posVGainPCasc;

    // constrain magnitude here
    VEC3_CONSTRAIN_XY_LENGTH(posSetpointNed.vel, velSpLimitXY);
    posSetpointNed.vel.V.Z = constrainf(posSetpointNed.vel.V.Z, -velSpLimitZ, +velSpLimitZ);

    // vel error = vel setpoint - vel estimate
    t_fp_vector velError = posSetpointNed.vel;
    VEC3_SCALAR_MULT_ADD(velError, -1.0f, extPosNed.vel);
    // acceleration setpoint = velGains * velError
    accSpNed.V.X = velError.V.X * posHGainD;
    accSpNed.V.Y = velError.V.Y * posHGainD;
    accSpNed.V.Z = velError.V.Z * posVGainD;

    // limit such that max acceleration likely results in bank angle below 40 deg
    VEC3_CONSTRAIN_XY_LENGTH(accSpNed, accMax);
    accSpNed.V.Z = constrainf(accSpNed.V.Z, MAX_ACC_Z_NEG, MAX_ACC_Z_POS);

    // todo: how to handle course/yaw?
    yawRateSpFromOuter = 0.f;
    // todo: weathervaning
    // todo: transform acc measurement from body to global and use INDI on acc
}

// TODO
// 1. velocity control..
