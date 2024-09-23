/*
 * Process keystrokes received from uplink using pi-telemetry
 *
 * Copyright 2024 Robin Ferede (Delft University of Technology)
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

#include "pi-messages.h"                // for piMsgKeyboardRx
#include "fc/core.h"                    // for disarm
#include "io/external_pos.h"            // for posSpNed
#include "flight/trajectory_tracker.h"  // for initTrajectoryTracker, stopTrajectoryTracker, trajectoryTrackerSetSpeed
#include "flight/nn_control.h"          // for nn_init, nn_activate

// HID Codes (https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2)
#define KEY_A           0x04
#define KEY_B           0x05
#define KEY_C           0x06
#define KEY_D           0x07
#define KEY_E           0x08
#define KEY_F           0x09
#define KEY_G           0x0A
#define KEY_H           0x0B
#define KEY_I           0x0C
#define KEY_J           0x0D
#define KEY_K           0x0E
#define KEY_L           0x0F
#define KEY_M           0x10
#define KEY_N           0x11
#define KEY_O           0x12
#define KEY_P           0x13
#define KEY_Q           0x14
#define KEY_R           0x15
#define KEY_S           0x16
#define KEY_T           0x17
#define KEY_U           0x18
#define KEY_V           0x19
#define KEY_W           0x1A
#define KEY_X           0x1B
#define KEY_Y           0x1C
#define KEY_Z           0x1D
#define KEY_0           0x27
#define KEY_1           0x1E
#define KEY_2           0x1F
#define KEY_3           0x20
#define KEY_4           0x21
#define KEY_5           0x22
#define KEY_6           0x23
#define KEY_7           0x24
#define KEY_8           0x25
#define KEY_9           0x26
#define KEY_SPACE       0x2C
#define KEY_ENTER       0x28
#define KEY_BACKSPACE   0x2A
#define KEY_TAB         0x2B
#define KEY_ESCAPE      0x29
#define KEY_LEFT        0x50
#define KEY_UP          0x48
#define KEY_RIGHT       0x4D
#define KEY_DOWN        0x50

#ifdef USE_TELEMETRY_PI
void processKey(uint8_t key) {
    // 0 = go to center
    // p = go to above nn_init
    // 1 = initTrajectoryTracker
    // 2 = decrease speed by 0.5 m/s
    // 3 = increase speed by 0.5 m/s
    // 4 = stopTrajectoryTracker
    // 5 = land
    // 6 = nn_init
    // 7 = nn_activate
    // 8 = recovery_mode
    // 9 = kill

    switch (key) {
#ifdef USE_GPS_PI
        case KEY_0: posSpNed.pos.V.X = 0.; posSpNed.pos.V.Y = 0.; posSpNed.pos.V.Z = -1.5; posSetpointState = EXT_POS_NEW_MESSAGE; break;
        case KEY_5: posSpNed.pos.V.X = 0.; posSpNed.pos.V.Y = 0.; posSpNed.pos.V.Z = 0.; posSetpointState = EXT_POS_NEW_MESSAGE; break;
#endif
#ifdef USE_TRAJECTORY_TRACKER
        case KEY_1: initTrajectoryTracker(); break;
        case KEY_2: incrementSpeedTrajectoryTracker(-0.5f); break;
        case KEY_3: incrementSpeedTrajectoryTracker(+0.5f); break;
        case KEY_4: stopTrajectoryTracker(); break;
        case KEY_8: initRecoveryMode(); break;
        case KEY_H: toggleHeadingTracking(); break;
#endif
#ifdef USE_NN_CONTROL
        case KEY_6: nn_init(); break;
        case KEY_7: if (nn_is_active()) { nn_deactivate(); } else { nn_activate(); } break;
        case KEY_P: posSpNed.pos.V.X = 1.5; posSpNed.pos.V.Y = -3.; posSpNed.pos.V.Z = -1.5; posSetpointState = EXT_POS_NEW_MESSAGE; break;
#endif
        case KEY_9: disarm(DISARM_REASON_KEYBOARD); break;
    }
}

void processKeyboard(void) {
    static timeUs_t latestKeyTime = 0;

    if (piMsgKeyboardRx) {
        timeUs_t currentKeyTime = piMsgKeyboardRx->time_us;
        timeDelta_t deltaMsgs = cmpTimeUs(currentKeyTime, latestKeyTime);
        if (deltaMsgs != 0) {
            latestKeyTime = currentKeyTime;
            processKey(piMsgKeyboardRx->key);
        }
    }
}
#endif