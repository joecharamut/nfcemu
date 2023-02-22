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
************************************************************************/
#pragma once

#include <stdint.h>

/// @brief Physical Data Layer functions
class PhyInterface {
public:
  /// @brief Setup phy layer (peripherals, etc)
  virtual void begin() = 0;

  /// @brief Transmit some data
  /// @param buf data buffer
  /// @param count number of bytes
  /// @param skipBits TODO
  virtual void transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0) = 0;

  /// @brief Try to receive some data
  /// @return number of bytes received
  virtual uint8_t receive() = 0;
  
  /// @brief Read one byte from buffer
  /// @return a byte
  virtual uint8_t read() = 0;

  /// @brief Get a pointer to the received data buffer
  /// @return pointer to the received data buffer
  virtual uint8_t *buffer() = 0;
};

