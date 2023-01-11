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

namespace NfcA {
  /// @brief Calculate the UID check byte for 4 bytes of UID data
  /// @details UID CLn check byte for NFC-A. 
  ///          BCC is an exclusive-OR over the first 4 bytes of the SDD_RES Response.
  /// @param buf buffer containing the 4 UID bytes
  /// @return the check byte
  uint8_t calcBcc(const uint8_t *buf);

  /// @brief Calculate the reply to SDD_REQ frames for the tag's UID
  /// @param uid buffer of all UID bytes
  /// @param uidSize size of UID in bytes (must be 4, 7, or 10 bytes)
  /// @param collisionLevel which collision level to generate reply 4 (0, 1, 2)
  /// @param outputBuf generated response output (must be 5 bytes)
  /// @return 0 on success, -1 for invalid parameter
  int8_t genSddResponse(const uint8_t *uid, uint8_t uidSize, uint8_t collisionLevel, uint8_t *outputBuf);
};
