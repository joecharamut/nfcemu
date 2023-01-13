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
************************************************************************/

#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usart.h"
#include "emulator.h"

uint8_t tagStorage[] = {
  0x00, 0x00, 0x00, 0x00, // Page 0, byte 0..3: UID0-UID3
  0x00, 0x00, 0x00, 0x00, // Page 1, byte 0..3: UID4-UID7
  0x00, 0x00, 0x00, 0x00, // P2 b0: UID8, P2 b1: Internal, P2 b2..3: Lock bytes
  0xE1, 0x10, 0x80, 0x00, // P3 b0..3: Capability Container [E1h = magic, 10h = version, 80h = mem size, 00h = rw access]

  0x03, 0xFF, 0x00, 0x1D, // P4 b0..3: TLV Block, Type: NDEF Message (03h), length: 29 bytes (FFh: 2 byte length)
  0xC2, // NDEF Record [MB=1, ME=1, CF=0, SR=0, IL=0, TNF=0x2]
  0x0A, // Type length: 10 bytes
  0x00, // Payload length 3: 0
  0x00, // Payload length 2: 0
  0x00, // Payload length 1: 0
  0x0D, // Payload length 0: 13 bytes

  // TYPE field: TNF=0x2 = MIME type
  't', 'e', 'x', 't', '/', 'p', 'l', 'a', 'i', 'n',

  // Payload 0
  'H', 'e', 'l', 'l', 'o', ' ', ',', 'W', 'o', 'r', 'l', 'd', '!',
};

uint8_t tagUid[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
            
//Byte 14 (value=0x80=1024/8) specifies tag size in bytes divided by 8
//Byte 18, 19 specify NDEF-Message size, Byte 20..35 is NDEF-header, content starts at Byte 36

NfcEmu::Emulator emu;

int main() {
  // disable clock prescaler (use 20MHz clock)
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x00);
  // set use extclk
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);

  Serial.begin();
  Serial.print("Hello world\n");

  emu.setup(tagStorage, sizeof(tagStorage));
  emu.setUid(tagUid, sizeof(tagUid));
  Serial.print("tag setup\n");

  while (1) {
    emu.waitForReader();
  }
}
