/***********************************************************************
Copyright (C) 2022 Joseph Charamut

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************/
#pragma once
#include <stdint.h>

namespace CRC {
/// @brief Initial value for CRC_A calculations
const uint16_t CRC_A_INITIAL = 0x6363;

/// @brief Update a CRC_A signature with a byte of data
/// @param crc the crc value to update
/// @param data the byte of data
/// @return new crc value
uint16_t updateCrc16A(uint16_t crc, uint8_t data);

/// @brief Calculate the CRC_A value of an entire block of data
/// @param buf data buffer
/// @param len length of data
/// @return the crc value
uint16_t calcCrc16A(uint8_t *buf, uint8_t len);

/// @brief Calculate and append the CRC_A value to a block of data
/// @param buf data buffer
/// @param len length of data
void appendCrc16A(uint8_t *buf, uint8_t len);
}
