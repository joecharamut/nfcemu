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
#include <avr/io.h>

#define __packed __attribute__((packed))

namespace NfcEmu {

enum ST_ENUM {
  ST_IDLE,
  ST_READY,
  ST_READY_CL1,
  ST_READY_CL2,
  ST_READY_CL3,
  ST_ACTIVE,
  ST_SLEEP,

  ST_MAX = 0xff,
};
typedef uint8_t emu_state_t;

typedef enum {
  ALL_REQ = 0x52,
  SENS_REQ = 0x26,
} nfc_short_cmd_t;

typedef struct __packed {
  nfc_short_cmd_t cmd;
} nfc_short_frame_t;

typedef enum {
  NFC_CL1 = 0b0011,
  NFC_CL2 = 0b0101,
  NFC_CL3 = 0b0111,
} collision_level_t;

typedef enum {
  SDD_REQ = 0b1001,
} sdd_cmd_t;

typedef struct __packed {
  collision_level_t col : 4;
  sdd_cmd_t cmd : 4;
} nfc_sel_cmd_t;

typedef struct __packed {
  collision_level_t col : 4;
  sdd_cmd_t cmd : 4;
  uint8_t bit_count : 4;
  uint8_t byte_count : 4;
} nfc_sdd_frame_t;


class Emulator {
public:
  void setup(uint8_t *storage, uint16_t storageSize);
  void tick();
  void waitForReader();

  int8_t setUid(uint8_t *uid, uint8_t uidSize);

  explicit Emulator();
  Emulator(Emulator const &) = delete;
  void operator=(Emulator) = delete;

  uint8_t rx();
  void tx(const uint8_t *buf, uint8_t count);

private:
  emu_state_t state = ST_IDLE;
  uint8_t debugFlags;

  uint8_t *storage;
  uint16_t storageSize;

  uint8_t nfcid1Size;
  uint8_t nfcid[3][5];

  uint8_t buffer[64];
};

};
