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

namespace Type2Tag {

struct __packed Block {
  uint8_t data[4];
};
static_assert(sizeof(Block) == 4);

struct __packed Sector {
  Block blocks[256];
};
static_assert(sizeof(Sector) == 1024);

const uint8_t NDEF_CC_MAGIC = 0xE1;
const uint8_t NDEF_CC_VERSION = 0x10;

const uint8_t NDEF_CC_READ_ALLOW = 0x0;
const uint8_t NDEF_CC_WRITE_ALLOW = 0x0;
const uint8_t NDEF_CC_WRITE_DENY = 0xf;

struct __packed CompatibilityContainer {
  uint8_t magic;
  uint8_t version;
  uint8_t memorySize;
  uint8_t readAccess : 4;
  uint8_t writeAccess : 4;
};
static_assert(sizeof(CompatibilityContainer) == 4);

namespace TLV {

enum __packed Tag {
  TAG_NULL = 0x00,
  TAG_LOCK_CTRL = 0x01,
  TAG_MEM_CTRL = 0x02,
  TAG_NDEF_MSG = 0x03,
  TAG_PROPRIETARY = 0xFD,
  TAG_TERMINATOR = 0xFE,
};
static_assert(sizeof(Tag) == 1);

struct __packed TLVBlock {
  Tag tag;
  union {
    struct __packed oneByteLen {
      uint8_t len1;
      uint8_t value[];
    };

    struct __packed threeByteLen {
      uint8_t len1;
      uint16_t len2;
      uint8_t value[];
    };
  };
};

};

};
