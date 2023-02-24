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

#include "emulator.h"
#include "usart.h"
#include "nfc/a/funcs.h"
#include "nfc/a/types.h"
#include "nfc/crc.h"

#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>

#define force_inline inline __attribute__((always_inline))

using namespace NfcEmu;

void Emulator::setUid(uint8_t *uid, uint8_t size) {
  m_uid = uid;
  m_uidSize = size;
}

void Emulator::setStorage(uint8_t *storage, uint16_t size) {
  m_storage = storage;
  m_storageSize = size;
}

void Emulator::tx(const uint8_t *data, uint8_t size, uint8_t skip) {
  phy.transmit(data, size, skip);
}

uint8_t Emulator::rx() {
  return phy.receive();
}

static const uint8_t SLP_REQ[2] = {0x50, 0x00};
static const uint8_t SEL_RES_INCOMPLETE[3] = {0x04, 0xDA, 0x17};
static const uint8_t SEL_RES_COMPLETE[3] = {0x00, 0xFE, 0x51};

static const uint8_t SENS_RES[3][2] = {
  { 0b00000100, 0b00000000 }, // SEL_RES CL1 ( 4 UID bytes)
  { 0b01000100, 0b00000000 }, // SEL_RES CL2 ( 7 UID bytes)
  { 0b10000100, 0b00000000 }, // SEL_RES CL3 (10 UID bytes)
};

void Emulator::idleState(uint8_t read) {
  if (read == 1) {
    uint8_t cmd = phy.read();
    if (cmd == NfcA::SENS_REQ || cmd == NfcA::ALL_REQ) {
      if (m_uidSize == 4) {
        tx(SENS_RES[0], 2);
      } else if (m_uidSize == 7) {
        tx(SENS_RES[1], 2);
      } else if (m_uidSize == 10) {
        tx(SENS_RES[2], 2);
      }

      m_state = ST_READY;
    }
  }
}

void Emulator::sleepState(uint8_t read) {
  if (read == 1) {
    uint8_t cmd = phy.read();
    if (cmd == NfcA::ALL_REQ) {
      if (m_uidSize == 4) {
        tx(SENS_RES[0], 2);
      } else if (m_uidSize == 7) {
        tx(SENS_RES[1], 2);
      } else if (m_uidSize == 10) {
        tx(SENS_RES[2], 2);
      }

      m_state = ST_READY;
    }
  }
}

void Emulator::readyState(uint8_t read) {
  NfcA::SDDFrame *fr = (NfcA::SDDFrame *)phy.buffer();

  // Serial.print(read);

  // TODO: not really correct
  if (read == 4 && phy.buffer()[0] == 0x50 && phy.buffer()[1] == 0x00) {
    m_state = ST_SLEEP;
    // Serial.print("Z");
    return;
  }

  // if receive unexpected frame type, return to IDLE state.
  if (fr->command != NfcA::SDDCommand::SDD_REQ) {
    m_state = ST_IDLE;
    Serial.print("#");
    for (uint8_t i = 0; i < read; i++) Serial.printHex(phy.buffer()[i]);
    return;
  }

  // get which collision level the command wants
  uint8_t col_idx = fr->collisionLevel / 2 - 1;
  // SDD_REQ is at most 7 bytes, use space after as work area
  uint8_t nfcid_buf[5];
  // calculate sdd compare bytes
  NfcA::genSddResponse(m_uid, m_uidSize, col_idx, nfcid_buf);

  if (fr->byteCount == 2 && fr->bitCount == 0) {
    // Poll device sent 16 bits (2b+0), expects 40 bits (5b+0) back
    // this is the basic SDD_REQ command to return the whole UID part
    tx(nfcid_buf, 5);
  } else if (fr->byteCount == 7 && fr->bitCount == 0) {
    // SEL_REQ command defined as having 0x70 as second byte (56 bits / 7b+0)
    // check if the id matches expected value
    if (memcmp(nfcid_buf, fr->data, 5) == 0) {
      // if matching, send select acknowledge
      if (
        (col_idx == 0 && m_uidSize ==  4) ||
        (col_idx == 1 && m_uidSize ==  7) ||
        (col_idx == 2 && m_uidSize == 10) 
      ) {
        tx(SEL_RES_COMPLETE, 3);
        // after being selected, now in ACTIVE state ready to handle type-2 tag commands
        m_state = ST_ACTIVE;
      } else {
        // still need to select next id part
        tx(SEL_RES_INCOMPLETE, 3);
      }
    }
  } else {
    // If some amount of bits between 16 and 56, this is a collision resolution frame.
    // In this case, we need to compare the first n bits of our UID with the UID recieved and reply with the rest
    // TODO: impl. this
    uint8_t uidBytes = fr->byteCount - 2;
    uint8_t uidBits = fr->bitCount;

    // match uid bits first
    if (uidBits) {
      uint8_t mask = (1 << uidBits) - 1;
      if ((nfcid_buf[uidBytes] & mask) != (fr->data[uidBytes] & mask)) {
        m_state = ST_IDLE;
        return;
      }
    }

    // TODO: also check uid bytes if present
    // while (uidBytes--) {
    //   if (nfcid_buf[uidBytes] != fr->data[uidBytes]) {
    //     state = ST_IDLE;
    //     return;
    //   }
    // }
    
    tx(nfcid_buf+uidBytes, 5-uidBytes, uidBits);
  }
}

void Emulator::activeState(uint8_t read) {
  (void)read;
}

void Emulator::waitForReader() {
  uint8_t read;
  uint8_t timeout = 0;
  m_state = ST_IDLE;

  do {
    read = rx();

    if (read) {
      timeout = 0;
      // Serial.hexdump(buffer, read);

      switch (m_state) {
        case ST_IDLE: idleState(read); break;
        case ST_SLEEP: sleepState(read); break;
        case ST_READY: readyState(read); break;
        case ST_ACTIVE: activeState(read); break;
        default: {
          Serial.print("m_state is an invalid value: ");
          Serial.print(m_state);
          cli();
          while (1);
        }; break;
      }

      // resetBitTimeout();
    } else {
      timeout++;
    }
  } while (timeout < 0xFF);
}
