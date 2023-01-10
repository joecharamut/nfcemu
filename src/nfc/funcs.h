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

  /**
   * @short Calculate the UID check byte for 4 bytes of UID data
   * 
   * @details UID CLn check byte for NFC-A. 
   *   BCC is an exclusive-OR over the first 4 bytes of the SDD_RES Response.
   * 
   * @param buf: buffer containing 4 bytes of the UID
   * @return the check byte
  */
  uint8_t calcBcc(const uint8_t *buf);

  /**
   * @short Calculate the reply to SDD_REQ frames for the tag's UID
   * 
   * @details 
   * 
   * @param uid: buffer of all UID bytes
   * @param uidSize: size of UID in bytes (must be 4, 7, or 10 bytes long)
   * @param collisionLevel: which collision level to generate reply 4 (0, 1, 2)
   * @param outputBuf: generated response output (must be 5 bytes)
   * @return 0 on success, -1 for invalid parameter
  */
  int8_t genSddResponse(const uint8_t *uid, uint8_t uidSize, uint8_t collisionLevel, uint8_t *outputBuf);

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
};
