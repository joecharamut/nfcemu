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
#include "impl/common.h"

#ifndef NFC_PHY_BUF_SIZE
#define NFC_PHY_BUF_SIZE 64
#endif

namespace NfcA {

/// @brief Physical layer implementation for data transmission
class Phy : public PhyInterface {
public:
  void begin();

  void transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0);
  uint8_t receive();

  void onReceive(PhyReceiveFnPtr fn);
  uint8_t read();
  uint8_t *buffer();

  uint8_t available();
  uint8_t bitsAvailable();

private:
  PhyReceiveFnPtr receiveCallback;

  uint8_t bytePos;
  uint8_t bitPos;

  uint8_t readPtr;
  uint8_t m_buffer[NFC_PHY_BUF_SIZE];

  

};

}
