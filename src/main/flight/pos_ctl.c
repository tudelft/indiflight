
#include "pos_ctl.h"

#include "io/external_pos.h"
#include "flight/imu.h"
#include "common/maths.h"
#include "fc/runtime_config.h"
#include "pg/pg_ids.h"
#include "config/config.h"
#include "flight/indi.h"

#ifdef USE_POS_CTL

#ifndef USE_GPS_PI
#error "USE_POS_CTL can currently only be used with USE_GPS_PI"
#endif

#ifndef USE_INDI
#pragma message "USE_POS_CTL currently only has any effect with USE_INDI"
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(positionProfile_t, POSITION_PROFILE_COUNT, positionProfiles, PG_POSITION_PROFILE, 0);

void pgResetFn_positionProfiles(positionProfile_t *positionProfiles) {
    for (int i = 0; i < POSITION_PROFILE_COUNT; i++) {
        positionProfile_t *p = &positionProfiles[i];
        p->horz_p = 30;
        p->horz_i = 2;
        p->horz_d = 40;
        p->horz_max_v = 250;
        p->horz_max_a = 500;
        p->horz_max_iterm = 200;
        p->max_tilt = 30;
        p->vert_p = 30;
        p->vert_i = 2;
        p->vert_d = 30;
        p->vert_max_v_up = 100;
        p->vert_max_v_down = 100;
        p->vert_max_a_up = 500;
        p->vert_max_a_down = 500;
        p->vert_max_iterm = 100;
        p->yaw_p = 30;
        p->weathervane_p = 0;
        p->weathervane_min_v = 200;
        p->use_spf_attenuation = 1;
    }
}

void posCtlInit(void) {
    initPositionRuntime();
}

positionRuntime_t posRuntime;
void initPositionRuntime(void) {
    const positionProfile_t* p = positionProfiles(systemConfig()->positionProfileIndex);
    posRuntime.horz_p = p->horz_p * 0.1f;
    posRuntime.horz_i = p->horz_i * 0.1f;
    posRuntime.horz_d = MAX(p->horz_d, 1) * 0.1f;
    posRuntime.horz_max_v = p->horz_max_v * 0.01f;
    posRuntime.horz_max_a = p->horz_max_a * 0.01f; 
    posRuntime.horz_max_iterm = p->horz_max_iterm * 0.01f;
    posRuntime.max_tilt = DEGREES_TO_RADIANS(p->max_tilt);
    posRuntime.vert_p = p->vert_p * 0.1f;
    posRuntime.vert_i = p->vert_i * 0.1f;
    posRuntime.vert_d = p->vert_d * 0.1f;
    posRuntime.vert_max_v_up = p->vert_max_v_up * 0.01f;
    posRuntime.vert_max_v_down = p->vert_max_v_down * 0.01f;
    posRuntime.vert_max_a_up = p->vert_max_a_up * 0.01f;
    posRuntime.vert_max_a_down = p->vert_max_a_down * 0.01f;
    posRuntime.vert_max_iterm = p->vert_max_iterm * 0.01f;
    posRuntime.yaw_p = p->yaw_p * 0.1f;
    posRuntime.weathervane_p = p->weathervane_p * 0.1f;
    posRuntime.weathervane_min_v = p->weathervane_min_v * 0.01f;
    posRuntime.use_spf_attenuation = (bool) p->use_spf_attenuation;
}

void changePositionProfile(uint8_t profileIndex)
{
    if (profileIndex < POSITION_PROFILE_COUNT) {
        systemConfigMutable()->positionProfileIndex = profileIndex;
    }

    initPositionRuntime();
}

// --- control variables
// externs
fp_vector_t accSpNedFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 1. };
fp_quaternion_t attSpNedFromPos = { .w = 1., .x = 0., .y = 0., .z = 0. };
fp_vector_t spfSpBodyFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 1. };
fp_vector_t rateSpBodyFromPos = { .V.X = 0., .V.Y = 0., .V.Z = 0. };

// locals
fp_vector_t velIError = {0};
static void resetIterms(void) {
    velIError.V.X = 0.f;
    velIError.V.Y = 0.f;
    velIError.V.Z = 0.f;
}

void updatePosCtl(timeUs_t current) {

    if (extPosState == EXT_POS_NO_SIGNAL) {
        // panic and level craft
        resetIterms();
        accSpNedFromPos.V.X = 0.f;
        accSpNedFromPos.V.Y = 0.f;
        accSpNedFromPos.V.Z = 1.f; // also command slight downwards acceleration
        return;
    }

    if ( (!ARMING_FLAG(ARMED)) || (!FLIGHT_MODE(POSITION_MODE | VELOCITY_MODE | GPS_RESCUE_MODE)) ) {
        resetIterms();
    }

    posGetAccSpNed(current);
    posGetAttSpNedAndSpfSpBody(current);
    posGetRateSpBody(current);
}

void posGetAccSpNed(timeUs_t current) {
    // precalculations
    float horzPCasc = posRuntime.horz_p / posRuntime.horz_d; // emulate parallel PD with Casc system
    float vertPCasc = posRuntime.vert_p / posRuntime.vert_d; // emulate parallel PD with Casc system

    // pos error = pos setpoint - pos estimate
    fp_vector_t posError = posSpNed.pos;
    VEC3_SCALAR_MULT_ADD(posError, -1.0f, posEstNed); // extPosNed.pos

    // vel setpoint = posGains * posError
    posSpNed.vel.V.X = posError.V.X * horzPCasc;
    posSpNed.vel.V.Y = posError.V.Y * horzPCasc;
    posSpNed.vel.V.Z = posError.V.Z * vertPCasc;

    // constrain magnitude here
    VEC3_CONSTRAIN_XY_LENGTH(posSpNed.vel, posRuntime.horz_max_v);

    posSpNed.vel.V.Z = constrainf(posSpNed.vel.V.Z, -posRuntime.vert_max_v_up, posRuntime.vert_max_v_down);

    // vel error = vel setpoint - vel estimate
    fp_vector_t velError = posSpNed.vel;
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

        VEC3_CONSTRAIN_XY_LENGTH(velIError, posRuntime.horz_max_iterm);
        velIError.V.Z = constrainf(velIError.V.Z, -posRuntime.vert_max_iterm, posRuntime.vert_max_iterm);
    }
    lastCall = current;

    // acceleration setpoint = velGains * velError
    accSpNedFromPos.V.X = velError.V.X * posRuntime.horz_d  +  velIError.V.X * posRuntime.horz_i;
    accSpNedFromPos.V.Y = velError.V.Y * posRuntime.horz_d  +  velIError.V.Y * posRuntime.horz_i;
    accSpNedFromPos.V.Z = velError.V.Z * posRuntime.vert_d  +  velIError.V.Z * posRuntime.vert_i;

    // limit such that max acceleration likely results in bank angle below 40 deg
    // but log if acceleration saturated, so we can pause error integration
    accSpXYSaturated = VEC3_XY_LENGTH(accSpNedFromPos) > posRuntime.horz_max_a;
    accSpZSaturated = (accSpNedFromPos.V.Z < -posRuntime.vert_max_a_up) || (accSpNedFromPos.V.Z > posRuntime.vert_max_a_down);

    VEC3_CONSTRAIN_XY_LENGTH(accSpNedFromPos, posRuntime.horz_max_a);
    accSpNedFromPos.V.Z = constrainf(accSpNedFromPos.V.Z, -posRuntime.vert_max_a_up, posRuntime.vert_max_a_down);
}

void posGetAttSpNedAndSpfSpBody(timeUs_t current) {
    UNUSED(current);
    // NOTE: this only makes sense for multicopters of course
    /*
    * system of equations:
    * 
    * a^I = Rz(Psi) @ axang(alpha, axis) @ f^B   +   (0 0 G)'
    *   where:
    *       a^I: craft acceleration in inertial frame
    *       Psi: craft yaw
    *       alpha: tilt angle (assume positive)
    *       axis: tilt axis (tx ty 0), where (tx^2 + ty^2 == 1)
    *       f^B: body forces in body frame. Assume for MC: f^B == (0 0 fz)
    *       G: gravity (9.80665)
    * 
    * define    v^Y := Rz(Psi)^(-1) @ ( a^I - (0 0 G)' )   which results in:
    * 
    * v^Y = axang(alpha, axis) @ f^B  +  (0 0 G)'
    * 
    * We require to solve tilt/thrust setpoint. Ie: solve the following 
    * system for (fz alpha tx ty):
    *   1      ==   tx**2  +  ty**2
    *   vx^Y   ==   ty * sin(alpha) * fz
    *   vy^Y   ==  -tx * sin(alpha) * fz
    *   vz^Y   ==        cos(alpha) * fz
    * 
    * where we obtain the axang multplication from the last column of https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    * 
    * 
    * The solution of the system is given by:
    *   tx      =  sgn(vy) * 1 / sqrt( 1 + (vx / vy)^2 ) = vy / sqrt(vy^2 + vx^2)
    *   ty      =  - (vx * tx) / vy
    *   alpha   =  atan( sqrt(vx^2 + vy^2) / (-vz) )
    *   fz      =  vz / cos(alpha)
    *
    */

    //float Psi = (float) DECIDEGREES_TO_RADIANS(-attitude.values.yaw);
    float Psi = getYawWithoutSingularity();
    float cPsi = cos_approx(Psi);
    float sPsi = sin_approx(Psi);

    // a^I - g^I
    fp_vector_t aSp_min_g = {
        .V.X = accSpNedFromPos.V.X,
        .V.Y = accSpNedFromPos.V.Y,
        .V.Z = accSpNedFromPos.V.Z - GRAVITYf,
    };

    // v^I = Rz(Psi)^(-1) @ aSp_min_g
    fp_vector_t v = {
        .V.X =  cPsi * aSp_min_g.V.X  +  sPsi * aSp_min_g.V.Y,
        .V.Y = -sPsi * aSp_min_g.V.X  +  cPsi * aSp_min_g.V.Y,
        .V.Z =                                                  aSp_min_g.V.Z,
    };

    fp_vector_t ax = {0};
    float vx2 = v.V.X*v.V.X;
    float vy2 = v.V.Y*v.V.Y;
    float XYnorm = sqrtf( vx2  +  vy2 );
    if ( XYnorm < 1e-4f ) {
        // 0.01% of a g
        // fall back logic, either ax = (+-1 0 0) or (0 +-1 0)
        // doesnt really matter since alpha will be tiny or close to pi
        ax.V.X = (vy2 >= vx2) ? ((v.V.Y >= 0.f) ? +1.f : -1.f) : 0.f;
        ax.V.Y = (vy2 <  vx2) ? ((v.V.X >= 0.f) ? -1.f : +1.f) : 0.f;
    } else {
        // base case
        if (vy2 > vx2) {
            ax.V.X  =  v.V.Y  /  XYnorm;
            ax.V.Y  =  - (v.V.X * ax.V.X) / v.V.Y;
        } else {
            ax.V.Y  =  - v.V.X  /  XYnorm;
            ax.V.X  =  - (v.V.Y * ax.V.Y) / v.V.X;
        }
    }

    float alpha = atan2_approx( XYnorm, -v.V.Z ); // norm is positive, so this is (0, M_PIf)
    alpha = constrainf(alpha, 0.f, posRuntime.max_tilt);

    spfSpBodyFromPos.V.X = 0.f;
    spfSpBodyFromPos.V.Y = 0.f;
    spfSpBodyFromPos.V.Z = v.V.Z / cos_approx(alpha);

    // attitude setpoint in the yaw frame
    fp_quaternion_t attSpYaw;
    quaternion_of_axis_angle(&attSpYaw, &ax, alpha);

    // add in the yaw
    fp_quaternion_t yawNed = {
        .w = cos_approx(Psi/2.f),
        .x = 0.f,
        .y = 0.f,
        .z = sin_approx(Psi/2.f),
    };

    // this is probaby the most expensive operation... can be half the cost if
    // optimized for .x = 0, .y = 0, unless compiler does that for us?
    attSpNedFromPos = quatMult(&yawNed, &attSpYaw);

    if (posRuntime.use_spf_attenuation) {
        // discount thrust if we have not yet reached our attitude
        fp_vector_t zDesNed = quatRotMatCol(&attSpNedFromPos, 2);
        float zDotProd = 
            - zDesNed.V.X * (+rMat.m[0][2]) // negative because z points down, but rMat points up
            - zDesNed.V.Y * (-rMat.m[1][2])
            - zDesNed.V.Z * (-rMat.m[2][2]);

        spfSpBodyFromPos.V.Z *= constrainf(zDotProd, 0.f, 1.f); // zDotProd is on [-1, +1]
    }
}

bool isWeathervane = false;

void posGetRateSpBody(timeUs_t current) {
    // for handling yaw
    // todo: weathervaning

    UNUSED(current);

    float yawSet = 0.;

    float speed = VEC3_XY_LENGTH(velEstNed);
    if (posRuntime.weathervane_p) {
        if ( speed >= posRuntime.weathervane_min_v )
            isWeathervane = true;
        else if ( speed < 0.8f * posRuntime.weathervane_min_v )
            isWeathervane = false;
    } else {
        isWeathervane = false;
    }

    yawSet = isWeathervane ? atan2_approx(velEstNed.V.Y, velEstNed.V.X) : posSpNed.psi;

    float yawError = yawSet - DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    if (yawError > M_PIf)
        yawError -= 2.f * M_PIf;
    else if (yawError < -M_PIf)
        yawError += 2.f * M_PIf;

    rateSpBodyFromPos.V.X = 0.;
    rateSpBodyFromPos.V.Y = 0.;
    rateSpBodyFromPos.V.Z = yawError * (isWeathervane ? posRuntime.weathervane_p : posRuntime.yaw_p);
}

// TODO
// 1. velocity control..

#endif // USE_POS_CTL
