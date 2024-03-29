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

#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>

#define __ASSERT_USE_STDERR 1
#include <assert.h>

#include "usart.h"
#include "emulator.h"
#include "phy/nfcA.h"

uint8_t tagStorage[] = {
  // Block 0: UID/Internal
  0x00, 0x00, 0x00, 0x00, // UID0-UID3

  // Block 1: UID/Internal
  0x00, 0x00, 0x00, 0x00, // UID4-UID7

  // Block 2: UID/Internal, Lock
  0x00, 0x00, 0x00, 0x00, // UID8, Internal, Lock0, Lock1

  // Block 3: Capability Container
    // CC magic number
    0xE1, 
    // version 1.0
    0x10, 
    // memory size/8 (0x04 = 32 bytes)
    0x04, 
    // access (r/w allowed)
    0x00, 

  // Block 4+: Data area

  // TLV Block Header
    // Tag: NDEF Message
    0x03, 
    // Length: 0xFF = 3 byte length value
    0xFF, 
    // Length: 28 bytes
    0x00, 0x1C,

  // TLV Block Data
  0xC2, // NDEF Record [MB=1, ME=1, CF=0, SR=0, IL=0, TNF=0x2]
  0x0A, // Type length: 10 bytes
  0x00, // Payload length 3: 0
  0x00, // Payload length 2: 0
  0x00, // Payload length 1: 0
  0x0D, // Payload length 0: 13 bytes

  // TYPE field: TNF=0x2 = MIME type
  't', 'e', 'x', 't', '/', 'p', 'l', 'a', 'i', 'n',

  // Payload 0
  'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!',
};

uint8_t tagUid[10] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23};
            
//Byte 14 (value=0x80=1024/8) specifies tag size in bytes divided by 8
//Byte 18, 19 specify NDEF-Message size, Byte 20..35 is NDEF-header, content starts at Byte 36

NfcA::Phy phy;
NfcEmu::Emulator emu(phy);

// extern "C" void abort() {
//   cli();
//   Serial.print("Aborted");
//   while (1);
// }

// extern "C" void __assert(const char *func, const char *file, int line, const char *expr) {
//   Serial.print("Assertion failed: (");
//   Serial.print(expr);
//   Serial.print(")");
//   if (func != NULL) {
//     Serial.print(", function "); Serial.print(func);
//   }
//   Serial.print(", file "); Serial.print(file);
//   Serial.print(", line "); Serial.print(line);
//   Serial.print(".\n");
//   abort();
// }

extern "C" void __cxa_pure_virtual() {
  while (1);
}

ISR(BADISR_vect) {
  for (;;) USART0.TXDATAL = '!';
}

int main() {
  // disable clock prescaler (use 20MHz clock)
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x00);
  // set use external clock
  // _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);

  Serial.begin();
  Serial.print("Hello world\n");

  if (RSTCTRL.RSTFR & RSTCTRL_UPDIRF_bm) {
    Serial.print("UPDI Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_UPDIRF_bm;
  }
  if (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm) {
    Serial.print("Software Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_SWRF_bm;
  }
  if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm) {
    Serial.print("WDT Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_WDRF_bm;
  }
  if (RSTCTRL.RSTFR & RSTCTRL_EXTRF_bm) {
    Serial.print("External Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_EXTRF_bm;
  }
  if (RSTCTRL.RSTFR & RSTCTRL_BORF_bm) {
    Serial.print("BOD Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_BORF_bm;
  }
  if (RSTCTRL.RSTFR & RSTCTRL_PORF_bm) {
    Serial.print("POR Reset\n");
    RSTCTRL.RSTFR |= RSTCTRL_PORF_bm;
  }
  
  if (RSTCTRL.RSTFR) {
    Serial.print("Unknown Reset (RSTFR != 0)\n");
    Serial.putchar(RSTCTRL.RSTFR);
    
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    while (1);
  }

  // set uid to device serial num
  // TODO: check if this works
  // memcpy(tagUid, (void*)&SIGROW.SERNUM0, 10);

  phy.begin();

  emu.setStorage(tagStorage, sizeof(tagStorage));
  emu.setUid(tagUid, 7);
  Serial.print("tag setup\n");

  while (1) {
    emu.waitForReader();
  }
}
