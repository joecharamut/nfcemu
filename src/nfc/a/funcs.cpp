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
#include "funcs.h"

#include <string.h>

uint8_t NfcA::calcBcc(const uint8_t *buf) {
  return buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
}

int8_t NfcA::genSddResponse(const uint8_t *uid, uint8_t uidSize, uint8_t collisionLevel, uint8_t *outputBuf) {
  // uidSize ==  4 -> CL1 / single size uid
  // uidSize ==  7 -> CL2 / double size uid
  // uidSize == 10 -> CL3 / triple size uid
  if (uidSize == 4) {
    // single size uid: no cascade
    if (collisionLevel == 0) {
      // copy in all 4 uid bytes
      memcpy(outputBuf, uid, 4);
      // add check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    }
  } else if (uidSize == 7) {
    // double size uid: 1 level cascade
    if (collisionLevel == 0) {
      // CL1 response:
      // multiple cascade levels: 88h = 'cascade tag'
      outputBuf[0] = 0x88;
      // first 3 bytes of uid
      memcpy(outputBuf+1, uid, 3);
      // check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    } else if (collisionLevel == 1) {
      // CL2 response:
      // rest 4 bytes of uid
      memcpy(outputBuf, uid+3, 4);
      // check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    }
  } else if (uidSize == 10) {
    // triple size uid: 2 level cascade
    if (collisionLevel == 0) {
      // CL1 response:
      // multiple cascade levels: 88h = 'cascade tag'
      outputBuf[0] = 0x88;
      // first 3 bytes of uid
      memcpy(outputBuf+1, uid, 3);
      // check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    } else if (collisionLevel == 1) {
      // CL2 response:
      // multiple cascade levels: 88h = 'cascade tag'
      outputBuf[0] = 0x88;
      // next 3 bytes of uid
      memcpy(outputBuf+1, uid+3, 3);
      // check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    } else if (collisionLevel == 2) {
      // CL3 response:
      // last 4 bytes of uid
      memcpy(outputBuf, uid+6, 4);
      // check byte
      outputBuf[4] = calcBcc(outputBuf);
      return 0;
    }
  }

  // invalid parameter
  return -1;
}
