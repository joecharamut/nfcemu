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

namespace NfcEmu {

typedef enum {
  ST_IDLE,
  ST_READY,
  ST_READY_CL1,
  ST_READY_CL2,
  ST_READY_CL3,
  ST_ACTIVE,
  ST_SLEEP,
} emu_state_t;

class Emulator {
public:
  void setup(uint8_t *storage, uint16_t storageSize);
  void tick();

  void setUid(uint8_t *uid, uint8_t uidSize);

private:
  uint8_t *storage;
  uint16_t storageSize;

  uint8_t nfcid1Size;
  uint8_t nfcid[3][5];

  uint8_t buffer[64];

  emu_state_t state;

  uint8_t rxMiller();
  void txManchester(const uint8_t *buf, uint8_t count);

  uint8_t rx();

  void handleIdle(uint8_t bytes);
  void handleReady(uint8_t bytes);
  void handleActive(uint8_t bytes);
  void handleSleep(uint8_t bytes);
};

};
