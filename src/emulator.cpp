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
#include "nfc/a/funcs.h"
#include "nfc/a/types.h"
#include "nfc/crc.h"

#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

using namespace NfcEmu;

// general constants
static constexpr uint32_t NFC_CARRIER = 13560000UL;
static constexpr uint32_t NFC_SUBCARRIER = NFC_CARRIER / 16UL;
static constexpr uint32_t NFC_BITPERIOD = NFC_CARRIER / 128UL;

// for TCA0
static constexpr uint8_t SUBCARRIER_PER = (((F_CPU/2) + (NFC_SUBCARRIER * 0.5)) / NFC_SUBCARRIER);
static constexpr uint8_t SUBCARRIER_CMP = ((SUBCARRIER_PER / 2));
static constexpr uint8_t BITTIMEOUT_PER = (((F_CPU/2) + (NFC_BITPERIOD * 0.5)) / NFC_BITPERIOD);

// for TCB0
static constexpr uint16_t BITPERIOD_PER = (((F_CPU) + (NFC_BITPERIOD * 0.5)) / NFC_BITPERIOD);
static constexpr uint16_t BITPERIOD_1_0 = (BITPERIOD_PER * 1.0);
static constexpr uint16_t BITPERIOD_1_5 = (BITPERIOD_PER * 1.5);
static constexpr uint16_t BITPERIOD_2_0 = (BITPERIOD_PER * 2.0);

Emulator::Emulator() {}

/**
 * Setup analog comparator and connect it to the event system
*/
static void AC0_setup() {
  // set AINN0 and AINP0 as INPUT
  PORTA.DIRCLR = _BV(PIN6) | _BV(PIN7);
  // disable digital input buffer
  PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  // enable pullup (TODO: needed vs hysteresis?)
  // PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
  // PORTA.PIN7CTRL |= PORT_PULLUPEN_bm;

  // set AC as event generator 0
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_AC0_OUT_gc;

  #define AC_DEBUG
  #ifdef AC_DEBUG
  // set PA5 as OUTPUT
  PORTA.DIRSET = _BV(PIN5);
  // enable comparator output
  AC0.CTRLA |= AC_OUTEN_bm;
  #endif

  // enable AC
  AC0.CTRLA |= AC_ENABLE_bm | AC_HYSMODE1_bm;
}

/**
 * Setup TCA0 to generate the required 848KHz (fc/16) subcarrier on WO4 (PA4/PIN2)
*/
static void TCA0_setup() {
  // setup WO4 pin as input / hi-Z because we dont want to output it just yet
  PORTA.DIRCLR = _BV(PIN4);

  // enable TCA split mode
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;

  // setup TCA0 high to generate the load modulation subcarrier
  // enable comparator 1 high (WO4/PA4/PIN2)
  TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP1EN_bm;
  // set waveform period
  TCA0.SPLIT.HPER = SUBCARRIER_PER;
  // set duty cycle
  TCA0.SPLIT.HCMP1 = SUBCARRIER_CMP;

  // setup TCA0 low as a timeout counter 
  TCA0.SPLIT.LPER = BITTIMEOUT_PER << 1;

  // set divider to CLK/2 and enable
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV2_gc | TCA_SPLIT_ENABLE_bm;
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

  // set divider to CLK/1 and enable
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

void Emulator::setup(uint8_t *storage, uint16_t storageSize) {
  this->storage = storage;
  this->storageSize = storageSize;

  AC0_setup();
  TCA0_setup();
  TCB0_setup();

  PORTA.DIRSET = _BV(PIN1);
}

void Emulator::setUid(uint8_t *uid, uint8_t uidSize) {
  this->uid = uid;
  this->uidSize = uidSize;
}

#define RX_EDGE_FLAG (TCB0.INTFLAGS & TCB_CAPT_bm)
#define RX_EDGE_TIME (TCB0.CCMP)

#define BIT_TIMEOUT_FLAG (TCA0.SPLIT.INTFLAGS & TCA_SPLIT_LUNF_bm)

// set timeout duration to bd*(2^n)
static inline void setBitTimeoutDuration(uint8_t n) {
  // TCA0 is clocked at appx. fc/16 (848KHz),
  // 1 bit-period is fc/128 (106KHz) = 8 counts
  // bitshift is faster than multiplication, but can only step in *2 multiples
  TCA0.SPLIT.LPER = (uint8_t) BITTIMEOUT_PER << n;
}

static inline void resetBitTimeout() {
  // clear count
  TCA0.SPLIT.LCNT = 0;
  // wait for underflow
  loop_until_bit_is_set(TCA0.SPLIT.INTFLAGS, TCA_SPLIT_LUNF_bp);
  // clear underflow interrupt
  TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
}

/// @brief Decode modified miller encoding of the demodulated signal
/// @details "mark" = '1' bit
///          a mark will always be encoded as having an edge in the middle of the bit period
///          
///          "space" = '0' bit
///          a space has an edge at the beginning or none at all
///          after a mark, a space will have no edge
///          after a space, a space has an edge
///          
///          Start of message/frame: space with edge
///          End of message/frame: space followed by idle period
/// @return number of bytes successfully received
/// @ref Protocol info referenced from https://github.com/sigrokproject/libsigrokdecode/blob/master/decoders/miller/pd.py
uint8_t Emulator::receive() {
  // wait for start of message (TCB0 interrupt) for bd * (2^n)
  setBitTimeoutDuration(3);
  resetBitTimeout();
  while (!RX_EDGE_FLAG) {
    // if timeout return no data
    if (BIT_TIMEOUT_FLAG) return 0;
  }

  // set timeout to 4 bd
  setBitTimeoutDuration(1);
  resetBitTimeout();

  uint16_t delta = RX_EDGE_TIME;
  uint8_t bitPos = 0;
  uint8_t bytePos = 0;
  uint8_t lastBit = 0;
  buffer[0] = 0;

  // macro to handle checking whether it's the next byte, 
  //  skipping the parity bit, and writing in the data
  #define BIT(b) do {                   \
    if (bitPos == 8) {                  \
      /* parity bit */                  \
      bitPos = 0;                       \
      bytePos++;                        \
      if (bitPos >= sizeof(buffer)) {   \
        return bytePos;                 \
      }                                 \
      buffer[bytePos] = 0;              \
    } else {                            \
      buffer[bytePos] |= ((b)<<bitPos); \
      bitPos++;                         \
    }                                   \
  } while (0)

  do {
    if (RX_EDGE_FLAG) {
      delta = RX_EDGE_TIME;

      if (lastBit == 0) {
        // space->???
        if (delta <= BITPERIOD_1_0) {
          // 1.0 time between bits = space
          BIT(0);
        } else if (delta <= BITPERIOD_1_5) {
          // 1.5 time = mark
          BIT(1);
          lastBit = 1;
        } else {
          // delta > 2.0 = space, idle (end of message)
          // BIT(0);
          break;
        }
      } else {
        // mark->???
        if (delta <= BITPERIOD_1_0) {
          // 1.0 time = mark
          BIT(1);
        } else if (delta <= BITPERIOD_1_5) {
          // 1.5 time = 2 spaces
          BIT(0);
          BIT(0);
          lastBit = 0;
        } else if (delta <= BITPERIOD_2_0) {
          // 2.0 time = space, mark
          BIT(0);
          BIT(1);
        } else {
          // delta > 2.0  == space, idle (end of message)
          // BIT(0);
          break;
        }
      }

      resetBitTimeout();
    }
  } while (!BIT_TIMEOUT_FLAG);
  
  /**
   * A short frame is used to initiate communication and consists of the following:
   * • SoF
   * • Up to 7 data bits transmitted lsb first
   * • EoF
   * so according to the nfc forum 0 bit frames are valid but i'm going to ignore them
   * unfortunetly the nfc forum decided that they only feel like issuing 4 bit responses
   * for type-2 tags so this is a compromise
   */
  if (bitPos >= 4) {
    bytePos++;
  }

  return bytePos;
}

static inline void waitForOneBit() {
  while (!BIT_TIMEOUT_FLAG);
  TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
}

static inline void waitNBitPeriods(uint8_t n) {
  setBitTimeoutDuration(0);
  resetBitTimeout();
  while (n) {
    waitForOneBit();
    --n;
  }
}

// enable waveform output on PA4
#define TX_ON() do { PORTA.DIRSET = _BV(PIN4); PORTA.OUTSET = _BV(PIN1); } while (0)
// disable waveform output on PA4
#define TX_OFF() do { PORTA.DIRCLR = _BV(PIN4); PORTA.OUTCLR = _BV(PIN1); } while (0)

// TODO: should waitForOneBit really be waiting for half a bit??
// transmit a '1' (modulation for 1/2 bd, no modulation 1/2 bd)
#define TX_1() do { \
  TX_ON(); \
  waitForOneBit(); \
  TX_OFF(); \
  waitForOneBit(); \
} while (0)

// transmit a '0' (no modulation for 1/2 bd, modulation 1/2 bd)
#define TX_0() do { \
  TX_OFF(); \
  waitForOneBit(); \
  TX_ON(); \
  waitForOneBit(); \
} while (0)

void Emulator::transmit(const uint8_t *buf, uint8_t count) {
  uint8_t bytePos = 0;
  uint8_t bitPos = 0;
  uint8_t parity = 0;

  // TODO: wait correct delay before transmit
  waitNBitPeriods(6);
  
  // PORTA.OUTSET = _BV(PIN1);
  setBitTimeoutDuration(0); // 1 bd timeout (2^0)
  waitForOneBit();
  
  // send SoF
  TX_1();

  do {
    if (buf[bytePos] & (1<<bitPos)) {
      // 1 bit
      TX_1();
      parity ^= 1;
    } else {
      // 0 bit
      TX_0();
    }
    bitPos++;

    if (bitPos > 7) {
      if (parity) {
        TX_0();
      } else {
        TX_1();
      }
      
      bytePos++;
      count--;
      bitPos = 0;
      parity = 0;
    }
  } while (count);

  // send EoF
  TX_OFF();
  waitForOneBit();

  // PORTA.OUTCLR = _BV(PIN1);
}

static const uint8_t ATQA[2] = {0x44, 0x00};
static const uint8_t SLP_REQ[2] = {0x50, 0x00};
static const uint8_t SAK_NC[3] = {0x04, 0xDA, 0x17}; //Select acknowledge uid not complete
static const uint8_t SAK_C[3] = {0x00, 0xFE, 0x51}; //Select acknowledge uid complete, Type 2 (PICC not compliant to ISO/IEC 14443-4)

static const uint8_t SENS_RES[3][2] = {
  { 0b00000100, 0b00000000 }, // SEL_RES CL1 ( 4 UID bytes)
  { 0b01000100, 0b00000000 }, // SEL_RES CL2 ( 7 UID bytes)
  { 0b10000100, 0b00000000 }, // SEL_RES CL3 (10 UID bytes)
};

void Emulator::handleShortFrame() {
  NfcA::ShortFrame *fr = (NfcA::ShortFrame *)buffer;

  if (fr->command == NfcA::ShortCommand::SENS_REQ || fr->command == NfcA::ShortCommand::ALL_REQ) {
    if (uidSize == 4) {
      transmit(SENS_RES[0], 2);
    } else if (uidSize == 7) {
      transmit(SENS_RES[1], 2);
    } else if (uidSize == 10) {
      transmit(SENS_RES[2], 2);
    }
  }
}

void Emulator::handleSDDFrame() {
  NfcA::SDDFrame *fr = (NfcA::SDDFrame *)buffer;
  
  if (fr->command == NfcA::SDDCommand::SDD_REQ && fr->byteCount == 2 && fr->bitCount == 0) {
    uint8_t col_idx = fr->collisionLevel / 2 - 1;
    NfcA::genSddResponse(uid, uidSize, col_idx, buffer);
    transmit(buffer, 5);
  }
}

void Emulator::handleStandardFrame(uint8_t read) {
  NfcA::SDDFrame *fr = (NfcA::SDDFrame *)buffer;

  // TODO: check crc value?
  if (buffer[0] == SLP_REQ[0] && buffer[1] == SLP_REQ[1]) {
    // SLP_REQ: do nothing / TODO: put device into idle state
  } else if (read == 7 && fr->command == NfcA::SDDCommand::SDD_REQ && fr->byteCount == 7 && fr->bitCount == 0) {
    // SEL_REQ
    uint8_t col_idx = fr->collisionLevel / 2 - 1;
    uint8_t *uid_buf = fr->data + 5;

    NfcA::genSddResponse(uid, uidSize, col_idx, uid_buf);
    if (memcmp(fr->data, uid_buf, 5) == 0) {
      buffer[0] = 0x00;
      if (uid_buf[0] == 0x88) {
        buffer[0] |= 0b00000100;
      }
      CRC::appendCrc16A(buffer, 1);
      transmit(buffer, 3);
    }
  }
  
  
  else {
    Serial.hexdump(buffer, read);
  }
}

void Emulator::waitForReader() {
  uint8_t read;
  do {
    read = receive();

    if (read) {
      // Serial.hexdump(buffer, read);

      if (read == 1) {
        handleShortFrame();
      } else if (read == 2) {
        handleSDDFrame();
      } else {
        handleStandardFrame(read);
      }

      resetBitTimeout();
    }
  } while (1);
}
