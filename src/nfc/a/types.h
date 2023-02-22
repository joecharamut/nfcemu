/***********************************************************************
Copyright (C) 2022-2023 Joseph Charamut

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
#define __packed __attribute__((packed))

namespace NfcA {

/// @brief Valid commands for short frames
enum __packed ShortCommand {
  ALL_REQ = 0x52,
  SENS_REQ = 0x26,
};
static_assert(sizeof(ShortCommand) == 1);

/// @brief A short frame is used to initiate communication between the Poll and Listen devices
/// @details A short frame is used to initiate communication and consists of the following (see Figure 3):
///           • SoF
///           • Up to 7 data bits transmitted lsb first
///           • EoF
///           No parity is added.
/// @ref Section 4.3.2 of NFCForum-TS-DigitalProtocol-1.0
struct __packed ShortFrame {
  ShortCommand command;
};
static_assert(sizeof(ShortFrame) == 1);

/// @brief Collision level values for SDD frames
enum __packed SDDCollisionLevel {
  SDD_CL1 = 0b0011,
  SDD_CL2 = 0b0101,
  SDD_CL3 = 0b0111,
};
static_assert(sizeof(SDDCollisionLevel) == 1);

/// @brief Valid commands for SDD frames
enum __packed SDDCommand {
  SDD_REQ = 0b1001,
};
static_assert(sizeof(SDDCommand) == 1);

/// @brief Bit oriented SDD frames are used for collision resolution
/// @details Bit oriented SDD frames are used for collision resolution 
///           and result from a standard frame with a length of 7 bytes 
///           that is split into two parts.
struct __packed SDDFrame {
  SDDCollisionLevel collisionLevel : 4;
  SDDCommand command : 4;
  uint8_t bitCount : 4;
  uint8_t byteCount : 4;
  uint8_t data[];
};
static_assert(sizeof(SDDFrame) == 2);

}
