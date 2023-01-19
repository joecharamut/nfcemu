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
#define __packed __attribute__((packed))

namespace NTAG21x {

enum __packed Command {
  GET_VERSION = 0x60,
  READ = 0x30,
  FAST_READ = 0x3A,
  WRITE = 0xA2,
  COMP_WRITE = 0xA0,
  READ_CNT = 0x39,
  PWD_AUTH = 0x1B,
  READ_SIG = 0x3C,
};
static_assert(sizeof(Command) == 1);

enum __packed ACK {
  Acknowledge = 0xA,
};
static_assert(sizeof(ACK) == 1);

enum __packed NACK {
  InvalidArgument = 0x0,
  ParityError = 0x1,
  InvalidAuthCounter = 0x4,
  WriteError = 0x5,
};
static_assert(sizeof(NACK) == 1);

struct __packed VersionInfo {
  uint8_t header;
  uint8_t vendorId;
  uint8_t productType;
  uint8_t productSubtype;
  uint8_t productMajorVersion;
  uint8_t productMinorVersion;
  uint8_t storageSize;
  uint8_t protocolType;
};
static_assert(sizeof(VersionInfo) == 8);



};
