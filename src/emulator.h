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
  ST_ACTIVE,
  ST_SLEEP,

  ST_MAX = 0xff,
};
typedef uint8_t emu_state_t;

class Emulator {
public:
  void setup(uint8_t *storage, uint16_t storageSize);
  void waitForReader();

  void setUid(uint8_t *uid, uint8_t uidSize);

  explicit Emulator();
  Emulator(Emulator const &) = delete;
  void operator=(Emulator) = delete;

  void receiveHandler(uint8_t read);
  // uint8_t receive();
  // void transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0);

private:
  emu_state_t state = ST_IDLE;

  uint8_t *storage;
  uint16_t storageSize;

  uint8_t *uid;
  uint8_t uidSize;

  uint8_t buffer[64];

  void idleState(uint8_t read);
  void sleepState(uint8_t read);
  void readyState(uint8_t read);
  void activeState(uint8_t read);
};

};
