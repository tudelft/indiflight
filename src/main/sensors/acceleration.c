/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "io/hil.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"

#include "acceleration.h"
#include "flight/rpm_filter.h"
#include "flight/mixer.h"


FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions
FAST_DATA_ZERO_INIT rpmFilter_t rpmFilterAcc;

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADCunfiltered[X] -= accelerationTrims->raw[X];
    acc.accADCunfiltered[Y] -= accelerationTrims->raw[Y];
    acc.accADCunfiltered[Z] -= accelerationTrims->raw[Z];
}

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
#ifdef HIL_BUILD
        const int16_t val = hilInput.acc[axis] * acc.dev.acc_1G;
#else
        const int16_t val =  acc.dev.ADCRaw[axis];
#endif
        acc.accADC[axis] = val;
    }

    if (acc.dev.accAlign == ALIGN_CUSTOM) {
        alignSensorViaMatrix(acc.accADC, &acc.dev.rotationMatrix);
    } else {
        alignSensorViaRotation(acc.accADC, acc.dev.accAlign);
    }

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADCunfiltered[axis] = acc.accADC[axis];
    }

#if defined(USE_ACCEL_RPM_FILTER) && defined(USE_RPM_FILTER)
    // update rpm notches
    for (int motor = 0; motor < getMotorCount(); motor++) {
        // only one harmonic for speed
        const float frequencyHz = constrainf(motorFrequencyHz[motor], rpmFilterAcc.minHz, rpmFilterAcc.maxHz);
        const float marginHz = frequencyHz - rpmFilterAcc.minHz;
        
        // fade out notch when approaching minHz (turn it off)
        float weight = 1.0f;
        if (marginHz < rpmFilterAcc.fadeRangeHz) {
            weight = marginHz / rpmFilterAcc.fadeRangeHz;
        }

        // update notch
        biquadFilterUpdate(&rpmFilterAcc.notch[FD_ROLL][motor][0], frequencyHz, rpmFilterAcc.looptimeUs, rpmFilterAcc.q, FILTER_NOTCH, weight);
        acc.accADC[FD_ROLL] = biquadFilterApplyDF1Weighted(&rpmFilterAcc.notch[FD_ROLL][motor][0], acc.accADC[FD_ROLL]);

        // copy over to other axes
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t *dest = &rpmFilter.notch[axis][motor][0];
            dest->b0 = rpmFilterAcc.notch[FD_ROLL][motor][0].b0;
            dest->b1 = rpmFilterAcc.notch[FD_ROLL][motor][0].b1;
            dest->b2 = rpmFilterAcc.notch[FD_ROLL][motor][0].b2;
            dest->a1 = rpmFilterAcc.notch[FD_ROLL][motor][0].a1;
            dest->a2 = rpmFilterAcc.notch[FD_ROLL][motor][0].a2;
            dest->weight = rpmFilterAcc.notch[FD_ROLL][motor][0].weight;
            acc.accADC[axis] = biquadFilterApplyDF1Weighted(&rpmFilterAcc.notch[axis][motor][0], acc.accADC[axis]);
        }
    }
#endif

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADCafterRpm[axis] = acc.accADC[axis];
    }

    // apply LP filtering
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const int16_t val = acc.accADC[axis];
        acc.accADC[axis] = accelerationRuntime.accLpfCutHz ? pt2FilterApply(&accelerationRuntime.accFilter[axis], val) : val;
    }

}

#endif
