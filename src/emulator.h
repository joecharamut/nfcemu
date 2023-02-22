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
#include <avr/io.h>
#include <noncopyable.hpp>
#include "phy/interface.h"

#define __packed __attribute__((packed))

namespace NfcEmu {

enum ST_ENUM {
  ST_IDLE,
  ST_READY,
  ST_ACTIVE,
  ST_SLEEP,

  ST_MAX = 0xff,
};
typedef uint8_t emu_state_t;

class Emulator : private NonCopyable {
public:
  Emulator(PhyInterface &phy) : phy(phy) {};
  void setup(uint8_t *storage, uint16_t storageSize);
  void setUid(uint8_t *uid, uint8_t uidSize);
  void waitForReader();

private:
  emu_state_t state = ST_IDLE;
  PhyInterface &phy;

  uint8_t *storage;
  uint16_t storageSize;

  uint8_t *uid;
  uint8_t uidSize;

  void idleState(uint8_t read);
  void sleepState(uint8_t read);
  void readyState(uint8_t read);
  void activeState(uint8_t read);
};

}
