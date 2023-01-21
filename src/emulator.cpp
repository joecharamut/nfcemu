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
static constexpr uint8_t SUBCARRIER_PER = (((F_CPU + (NFC_SUBCARRIER * 0.5)) / NFC_SUBCARRIER) - 0.5);
static constexpr uint8_t SUBCARRIER_CMP = ((SUBCARRIER_PER / 2));
static constexpr uint8_t BITTIMEOUT_PER = (((F_CPU/1) + (NFC_BITPERIOD * 0.5)) / NFC_BITPERIOD);
static constexpr uint8_t HALF_BIT_TIMEOUT_PER = (BITTIMEOUT_PER/2);

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
  TCA0.SPLIT.HPER = SUBCARRIER_PER-1;
  // set duty cycle
  TCA0.SPLIT.HCMP1 = SUBCARRIER_CMP;

  // setup TCA0 low as a timeout counter 
  TCA0.SPLIT.LPER = BITTIMEOUT_PER;

  // set divider to CLK/2 and enable
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc | TCA_SPLIT_ENABLE_bm;

  // PORTA.DIRSET = _BV(PIN4);
  // while (1);
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

  PORTA.DIRSET = _BV(PIN1);// | _BV(PIN2);
  // PORTB.DIRSET = _BV(PIN3);
}

void Emulator::setUid(uint8_t *uid, uint8_t uidSize) {
  this->uid = uid;
  this->uidSize = uidSize;
}

#define RX_EDGE_FLAG (TCB0.INTFLAGS & TCB_CAPT_bm)
#define RX_EDGE_TIME (TCB0.CCMP)

#define BIT_TIMEOUT_FLAG (TCA0.SPLIT.INTFLAGS & TCA_SPLIT_LUNF_bm)

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
/// @ref Protocol info referenced from https://github.com/sigrokproject/libsigrokdecode/blob/master/decoders/miller/pd.py
/// @return number of bytes successfully received
uint8_t Emulator::receive() {
  // wait for start of message (TCB0 interrupt) for bd * (2^n)
  uint8_t timeouts = 6;
  resetBitTimeout();
  while (!RX_EDGE_FLAG) {
    if (BIT_TIMEOUT_FLAG) {
      // if timeout return no data
      if (!timeouts) return 0;
      timeouts--;
      TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
    }
  }

  // set timeout
  TCA0.SPLIT.LPER = 0xFF;
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
   * TODO: AAAAAAAAAAAAA FINE I GUESS ITS TIME TO DO 1 BIT FRAMES (collision resolution piss)
   */
  if (bitPos >= 4) {
    bytePos++;
  }

  TCA0.SPLIT.LPER = BITTIMEOUT_PER;

  return bytePos;
}

static inline void waitForBitTimer() {
  while (!BIT_TIMEOUT_FLAG);
  TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
}

static inline void waitNBitPeriods(uint8_t n) {
  uint8_t old = TCA0.SPLIT.LPER;
  TCA0.SPLIT.LPER = BITTIMEOUT_PER;
  resetBitTimeout();
  while (n) {
    waitForBitTimer();
    --n;
  }
  TCA0.SPLIT.LPER = old;
}

/// @brief Output waveform on PA4 for load modulation
static inline void tx_high() {
  // reset waveform timer
  TCA0.SPLIT.HCNT = 0;
  // enable output
  // PORTA.DIRSET = _BV(PIN4);
  VPORTA.DIR |= _BV(PIN4);

  // debug
  VPORTA.OUT |= _BV(PIN1);
  // PORTA.OUTTGL = _BV(PIN2);
}

/// @brief Stop outputting waveform on PA4 for load modulation
static inline void tx_low() {
  // disable output
  // PORTA.DIRCLR = _BV(PIN4);
  VPORTA.DIR &= ~_BV(PIN4);

  // debug
  VPORTA.OUT &= ~_BV(PIN1);
  // PORTA.OUTTGL = _BV(PIN2);
}

/// @brief Transmit a one bit (high->low after 1/2bd)
static inline void tx_one() {
  waitForBitTimer();
  tx_high();
  waitForBitTimer();
  tx_low();
}

/// @brief Transmit a zero bit (low->high after 1/2bd)
static inline void tx_zero() {
  waitForBitTimer();
  tx_low();
  waitForBitTimer();
  tx_high();
}

/// @brief Transmit start of frame condition (a one bit)
static inline void tx_sof() {
  tx_one();
}

/// @brief Transmit end of frame condition (no modulation for 1bd)
static inline void tx_eof() {
  waitForBitTimer();
  tx_low();
  waitForBitTimer();
}

void Emulator::transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0) {
  uint8_t bytePos = 0;
  uint8_t bitPos = skipBits;
  uint8_t parity = 0;

  // TODO: wait correct delay before transmit
  waitNBitPeriods(4);
  
  // setup bit timer to wait for half bit period (oops)
  TCA0.SPLIT.LPER = HALF_BIT_TIMEOUT_PER;
  // reset count
  TCA0.SPLIT.LCNT = HALF_BIT_TIMEOUT_PER;
  waitForBitTimer();
  waitForBitTimer();

  // send SoF
  tx_sof();

  do {
    if (buf[bytePos] & (1<<bitPos)) {
      // 1 bit
      tx_one();
      parity ^= 1;
    } else {
      // 0 bit
      tx_zero();
    }
    bitPos++;

    if (bitPos > 7) {
      if (parity) {
        tx_zero();
      } else {
        tx_one();
      }
      
      bytePos++;
      count--;
      bitPos = 0;
      parity = 0;
    }
  } while (count);

  // send EoF
  tx_eof();
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
  if (read == 1 && (buffer[0] == NfcA::SENS_REQ || buffer[0] == NfcA::ALL_REQ)) {
    if (uidSize == 4) {
      transmit(SENS_RES[0], 2);
    } else if (uidSize == 7) {
      transmit(SENS_RES[1], 2);
    } else if (uidSize == 10) {
      transmit(SENS_RES[2], 2);
    }

    state = ST_READY;
  }
}

void Emulator::sleepState(uint8_t read) {
  if (read == 1 && buffer[0] == NfcA::ALL_REQ) {
    if (uidSize == 4) {
      transmit(SENS_RES[0], 2);
    } else if (uidSize == 7) {
      transmit(SENS_RES[1], 2);
    } else if (uidSize == 10) {
      transmit(SENS_RES[2], 2);
    }

    state = ST_READY;
  }
}

void Emulator::readyState(uint8_t read) {
  NfcA::SDDFrame *fr = (NfcA::SDDFrame *)buffer;

  // TODO: not really correct
  if (read == 4 && buffer[0] == 0x50 && buffer[1] == 0x00) {
    state = ST_SLEEP;
    return;
  }

  // if recieve unexpected frame type, return to IDLE state.
  if (fr->command != NfcA::SDDCommand::SDD_REQ) {
    state = ST_IDLE;
    return;
  }

  // get which collision level the command wants
  uint8_t col_idx = fr->collisionLevel / 2 - 1;
  // SDD_REQ is at most 7 bytes, use space after as work area
  uint8_t *nfcid_buf = buffer+8;
  // calculate sdd compare bytes
  NfcA::genSddResponse(uid, uidSize, col_idx, nfcid_buf);

  if (fr->byteCount == 2 && fr->bitCount == 0) {
    // Poll device sent 16 bits (2b+0), expects 40 bits (5b+0) back
    // this is the basic SDD_REQ command to return the whole UID part
    transmit(nfcid_buf, 5);
  } else if (fr->byteCount == 7 && fr->bitCount == 0) {
    // SEL_REQ command defined as having 0x70 as second byte (56 bits / 7b+0)
    // check if the id matches expected value
    if (memcmp(nfcid_buf, fr->data, 5) == 0) {
      // if matching, send select acknowledge
      if (
        (col_idx == 0 && uidSize ==  4) ||
        (col_idx == 1 && uidSize ==  7) ||
        (col_idx == 2 && uidSize == 10) 
      ) {
        transmit(SEL_RES_COMPLETE, 3);
        // after being selected, now in ACTIVE state ready to handle type-2 tag commands
        state = ST_ACTIVE;
      } else {
        // still need to select next id part
        transmit(SEL_RES_INCOMPLETE, 3);
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
        state = ST_IDLE;
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
    
    transmit(nfcid_buf, 5, uidBits);
  }
}

void Emulator::activeState(uint8_t read) {

}

void Emulator::waitForReader() {
  uint8_t read;
  uint8_t timeout = 0;
  state = ST_IDLE;

  do {
    read = receive();

    if (read) {
      timeout = 0;
      // Serial.hexdump(buffer, read);

      switch (state) {
        case ST_IDLE: idleState(read); break;
        case ST_SLEEP: sleepState(read); break;
        case ST_READY: readyState(read); break;
        case ST_ACTIVE: activeState(read); break;
        default: 
          Serial.print("!!! INVALID STATE !!!\n");
          while (1);
          break;
      }

      // resetBitTimeout();
    } else {
      timeout++;
    }
  } while (timeout < 0xFF);
}
