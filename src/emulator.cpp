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

#include "emulator.h"
#include "usart.h"

#include <util/delay.h>
#include <string.h>

using namespace NfcEmu;

// #define force_inline inline __attribute__((always_inline))
#define force_inline 

#define STRINGIFY(x) DO_STRINGIFY(x)
#define DO_STRINGIFY(x) #x

#define DEBUG
#ifdef DEBUG
#define ASSERT(x) do { \
  if (!(x)) { \
    Serial.print("ASSERTION FAILED: (" #x ") in func "); \
    Serial.print(__PRETTY_FUNCTION__); \
    Serial.print(", line " STRINGIFY(__LINE__) "\n"); \
    while (1); \
  } \
} while (0)
#else
#define ASSERT(x)
#endif

#define NFC_CARRIER 13560000L
#define NFC_SUBCARRIER (NFC_CARRIER/16L)
#define NFC_BITPERIOD (NFC_CARRIER/128L)
#define SUBCARRIER_PER ((F_CPU + (NFC_SUBCARRIER * 0.5)) / NFC_SUBCARRIER - 1)
#define SUBCARRIER_CMP ((SUBCARRIER_PER / 2) + 0.5)
#define BITPERIOD_PER ((F_CPU + (NFC_BITPERIOD * 0.5)) / NFC_BITPERIOD - 1)
#define BITPERIOD_1_0 (BITPERIOD_PER * 1.0)
#define BITPERIOD_1_5 (BITPERIOD_PER * 1.5)
#define BITPERIOD_2_0 (BITPERIOD_PER * 2.0)

Emulator::Emulator() {}

/**
 * configure analog comparator
*/
static void AC0_setup() {
  // set AINN0 and AINP0 as INPUT
  PORTA.DIRCLR |= _BV(PIN6) | _BV(PIN7);
  // enable pullup (TODO: needed here or setup hysteresis?)
  // PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
  // PORTA.PIN7CTRL |= PORT_PULLUPEN_bm;

  // enable AC with negative edge interrupt
  AC0.CTRLA = AC_INTMODE_NEGEDGE_gc | AC_ENABLE_bm;

  // set AC as event generator 0
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_AC0_OUT_gc;

#define AC_DEBUG
#ifdef AC_DEBUG
  // enable output pin on PA5/PIN3 for debugging
  PORTA.DIRSET |= _BV(PIN5);
  AC0.CTRLA |= AC_OUTEN_bm;
#endif
}

/**
 * Setup TCA0 to generate the required 848KHz (fc/16) subcarrier on WO4 (PA4/PIN2)
*/
static void TCA0_setup() {
  // setup pin as input / hi-Z because we dont want to output it just yet
  PORTA.DIRCLR |= _BV(PIN4);

  // enable TCA split mode
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;

  // setup TCA0 high to generate the load modulation subcarrier
  // enable comparator 1 high (WO4/PA4/PIN2)
  TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP1EN_bm;
  // set period value to full range
  TCA0.SPLIT.HPER = SUBCARRIER_PER;
  // set duty cycle / compare value
  TCA0.SPLIT.HCMP1 = SUBCARRIER_CMP;

  // setup TCA0 low as a timeout counter
  TCA0.SPLIT.LPER = BITPERIOD_PER * 2; // 2 bp

  // set divider to CLK/1 and enable
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc | TCA_SPLIT_ENABLE_bm;
}

/**
 * Setup TCB0 to capture when the comparator detects a falling edge (for modified miller decoding)
*/
static void TCB0_setup() {
  // set TCB0 event source to async event ch 0
  EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;

  // enable input capture event on negative edge
  TCB0.EVCTRL = TCB_EDGE_bm | TCB_CAPTEI_bm;

  // input capture frequency measurement mode
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;

  // set clock source to same as TCA0 and enable
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
}

void Emulator::setup(uint8_t *storage, uint16_t storageSize) {
  this->storage = storage;
  this->storageSize = storageSize;

  AC0_setup();
  TCA0_setup();
  TCB0_setup();
}

static void genBcc(uint8_t *buf) {
  //add exclusive-OR of 4 bytes
  buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
}

int8_t Emulator::setUid(uint8_t *uid, uint8_t uidSize) {
  if (uidSize <= 4) {
    // 4 byte uid = Cascade Lv 1
    nfcid1Size = 0;
    
    for (uint8_t i = 0; i < 4; i++) {
      nfcid[0][i] = uid[i];
    }
    genBcc(nfcid[0]);
    return 0;
  } else if (uidSize <= 7) {
    // 7 byte uid = Cascade Lv 2
    nfcid1Size = 1;

    // CL1 part: CT id0 id1 id2 BCC
    nfcid[0][0] = 0x88; // Cascade Tag = 88h
    for (uint8_t i = 0; i < 3; i++) {
      nfcid[0][i+1] = uid[i];
    }
    genBcc(nfcid[0]);

    // CL2 part: id3 id4 id5 id6 BCC
    for (uint8_t i = 0; i < 3; i++) {
      nfcid[1][i] = uid[i+3];
    }
    genBcc(nfcid[1]);
    return 1;
  } else if (uidSize <= 10) {
    // 10 byte uid = Cascade Lv 3
    nfcid1Size = 2;

    // CL1 part: CT id0 id1 id2 BCC
    nfcid[0][0] = 0x88; // Cascade Tag = 88h
    for (uint8_t i = 0; i < 3; i++) {
      nfcid[0][i+1] = uid[i];
    }
    genBcc(nfcid[0]);

    // CL2 part: CT id3 id4 id5 BCC
    nfcid[0][0] = 0x88; // Cascade Tag = 88h
    for (uint8_t i = 0; i < 3; i++) {
      nfcid[1][i+1] = uid[i+3];
    }
    genBcc(nfcid[1]);

    // CL3 part: id6 id7 id8 id9 BCC
    for (uint8_t i = 0; i < 3; i++) {
      nfcid[2][i] = uid[i+6];
    }
    genBcc(nfcid[2]);
    return 2;
  } else {
    // invalid uidSize or uidSize > 10 bytes (not supported)
    return -1;
  }
}

#define RX_EDGE_FLAG (TCB0.INTFLAGS & TCB_CAPT_bm)
#define RX_EDGE_TIME (TCB0.CCMP)
static inline void resetRxEdgeTimer() {
  // clear counter
  TCB0.CNT = 0;
  // clear flag register
  TCB0.INTFLAGS |= TCB_CAPT_bm;
}

#define BIT_TIMEOUT_FLAG (TCA0.SPLIT.INTFLAGS & TCA_SPLIT_LUNF_bm)
static inline void resetBitTimeout() {
  // TCA0 is clocked at appx. fc/16 (848KHz),
  // 1 bit-period is fc/128 (106KHz) = 8 counts
  TCA0.SPLIT.LPER = BITPERIOD_PER * 4; // timeout after 4 bp
  // clear underflow interrupt
  TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
}

/**
 * Decode modified miller encoding of the demodulated signal
 * 
 * "mark" = '1' bit
 * a mark will always be encoded as having an edge in the middle of the bit period
 * 
 * "space" = '0' bit
 * a space has an edge at the beginning or none at all
 * after a mark, a space will have no edge
 * after a space, a space has an edge
 * 
 * Start of message/frame: space with edge
 * End of message/frame: space followed by idle period
 * 
 * Protocol info referenced from https://github.com/sigrokproject/libsigrokdecode/blob/master/decoders/miller/pd.py
*/
uint8_t Emulator::rx() {
  // prevent some runtime cost of bitshift (needed?)
  static const uint8_t bitmask[8] = { (1<<0), (1<<1), (1<<2), (1<<3), (1<<4), (1<<5), (1<<6), (1<<7) }; 
  
  // get buffer ready
  uint8_t bytePos = 0;
  buffer[bytePos] = 0;

  // wait for start of message (TCB0 interrupt)
  while (!RX_EDGE_FLAG) {
    // if timeout return no data
    if (BIT_TIMEOUT_FLAG) return 0;
  }

  // read time between edges (clears interrupt flag)
  uint16_t edgeDelta = RX_EDGE_TIME;

  // start of message is always 0 bit
  uint8_t bitPos = 1;
  uint8_t prevBit = 0;

  do {
    // on next edge...
    if (RX_EDGE_FLAG) {
      // read delta
      edgeDelta = RX_EDGE_TIME;
      resetBitTimeout();

      // interpret resulting bit time
      // should always be {1.0, 1.5, 2.0}bp
      if (prevBit == 0) {
        // space->???
        if (edgeDelta == BITPERIOD_1_0) {
          // 1.0 time between bits = space
          bitPos++;
        } else if (edgeDelta == BITPERIOD_1_5) {
          // 1.5 time = mark
          buffer[bytePos] |= (1<<bitPos);
          prevBit = 1;
          bitPos++;
        } else if (edgeDelta >= BITPERIOD_2_0) {
          // idle symbol (end of message)
          break;
        } else {
          // delta < 1.0 == invalid?
          break;
        }
      } else {
        // mark->???
        if (edgeDelta == BITPERIOD_1_0) {
          // 1.0 time = mark
          buffer[bytePos] |= (1<<bitPos);
          prevBit = 1;
          bitPos++;
        } else if (edgeDelta == BITPERIOD_1_5) {
          // 1.5 time = 2 spaces
          bitPos += 2;
        } else if (edgeDelta == BITPERIOD_2_0) {
          // 2.0 time = space, mark
          bitPos += 2;
          if (bitPos >= 8) {
            bitPos -= 8;
            bytePos++;
          }
          buffer[bytePos] |= (1<<bitPos);
          prevBit = 1;
          bitPos++;
        } else {
          // delta < 1.0 || delta > 2.0  == invalid?
          break;
        }
      }

      if (bitPos >= 8) {
        bitPos -= 8;
        bytePos++;
      }
    }
  } while (!BIT_TIMEOUT_FLAG);

  // 7-bit messages are allowed
  if (bitPos >= 7) {
    bytePos++;
  }
  return bytePos;
}

#define BIT_TIMEOUT_FLAG (TCA0.SPLIT.INTFLAGS & TCA_SPLIT_LUNF_bm)
static inline void resetOneBitTimer() {
  // TCA0 is clocked at appx. fc/16 (848KHz),
  // 1 bit-period is fc/128 (106KHz) = 8 counts
  TCA0.SPLIT.LPER = BITPERIOD_PER;
  // clear underflow interrupt
  TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
}
static inline void waitForOneBit() {
  while (!BIT_TIMEOUT_FLAG);
  resetOneBitTimer();
}

// macros to toggle PA4 waveform pin
#define TX_ON() do { PORTA.DIRSET |= _BV(PIN4); } while (0)
#define TX_OFF() do { PORTA.DIRCLR |= _BV(PIN4); } while (0)

#define TX_1() do { \
  waitForOneBit(); \
  TX_ON(); \
  waitForOneBit(); \
  TX_OFF(); \
} while (0)

#define TX_0() do { \
  waitForOneBit(); \
  TX_OFF(); \
  waitForOneBit(); \
  TX_ON(); \
} while (0)

#define TXBIT(x) TX_##x()

void Emulator::tx(const uint8_t *buf, uint8_t count) {
  uint8_t bytePos = 0;
  uint8_t bitPos = 0;
  uint8_t parity = 0;

  resetOneBitTimer();
  
  // send SOC
  TXBIT(1);

  do {
    if (buf[bytePos] & (1<<bitPos)) {
      // 1 bit
      TXBIT(1);
      parity ^= 1;
    } else {
      // 0 bit
      TXBIT(0);
    }

    if (bitPos > 7) {
      if (parity) {
        TXBIT(0);
      } else {
        TXBIT(1);
      }
      
      bytePos++;
      bitPos = 0;
      parity = 0;
    }
  } while (bytePos < count);

  // send EOC
  waitForOneBit();
  TX_OFF();
}

static uint8_t ATQA[] = {0x44, 0x00};
static uint8_t SLP_REQ[] = {0x50, 0x00};
static uint8_t SAK_NC[3] = {0x04, 0xDA, 0x17}; //Select acknowledge uid not complete
static uint8_t SAK_C[3] = {0x00, 0xFE, 0x51}; //Select acknowledge uid complete, Type 2 (PICC not compliant to ISO/IEC 14443-4)

static const uint8_t SENS_RES[3][2] = {
  { 0b00000100, 0b00000000 }, // SEL_RES CL1 ( 4 UID bytes)
  { 0b01000100, 0b00000000 }, // SEL_RES CL2 ( 7 UID bytes)
  { 0b10000100, 0b00000000 }, // SEL_RES CL3 (10 UID bytes)
};

void Emulator::waitForReader() {
  uint8_t read;
  do {
    read = rx();

    if (read) {

    }
  } while (0);
}
