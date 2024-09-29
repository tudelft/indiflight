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

#pragma once

#include "common/utils.h"

#define FC_FIRMWARE_NAME            "Indiflight"
#define FC_CUSTOM_DEFAULT_FIRMWARE_NAME "Betaflight" // this is a hack, because I didnt yet figure out how to reflash custom defaults
// #define FC_FIRMWARE_IDENTIFIER      "INFL"
#define FC_FIRMWARE_IDENTIFIER      "BTFL"
#define FC_VERSION_MAJOR            5  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            0  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   15 // lower case hexadecimal digits, plus eventual +dirty flag
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"

#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)

extern const char* const buildKey;
extern const char* const releaseName;
