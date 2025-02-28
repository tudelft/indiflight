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

int blackboxPrintf(const char *fmt, ...);
void blackboxPrintfHeaderLine(const char *name, const char *fmt, ...);

void blackboxWriteUnsignedVB(uint32_t value);
void blackboxWriteSignedVB(int32_t value);
void blackboxWriteSignedVBArray(int32_t *array, int count);
void blackboxWriteSigned16VBArray(int16_t *array, int count);
void blackboxWriteUnsigned16VBArray(uint16_t *array, int count);
void blackboxWriteS16(int16_t value);
void blackboxWriteTag2_3S32(int32_t *values);
int blackboxWriteTag2_3SVariable(int32_t *values);
void blackboxWriteTag8_4S16(int32_t *values);
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount);
void blackboxWriteU32(int32_t value);
void blackboxWriteFloat(float value);

// scaling
#define UNIT_FLOAT_TO_SIGNED16VB ((127 << 6) - 1) // float from -1 to +1 to VB that takes up 2 bytes max
#define UNIT_FLOAT_TO_UNSIGNED16VB ((127 << 7) - 1) // float from 0 to +1 to VB that takes up 2 bytes max
#define METER_TO_MM 1000
#define METER_TO_CM 100
