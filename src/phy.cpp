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

#include "phy.h"

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>

#define force_inline inline __attribute__((always_inline))

using namespace NfcA;

void Phy::onReceive(PhyReceiveFnPtr fn) {
  receiveCallback = fn;
}

uint8_t Phy::available() {
  return bytePos;
}

uint8_t Phy::bitsAvailable() {
  return bitPos;
}

uint8_t Phy::read() {
  return buffer[readPtr++];
}

uint8_t *Phy::getBuffer() {
  return buffer;
}

// general constants
static constexpr uint32_t NFC_CARRIER = 13560000UL;
static constexpr uint32_t NFC_SUBCARRIER = NFC_CARRIER / 16UL;
static constexpr uint32_t NFC_BITPERIOD = NFC_CARRIER / 128UL;
static constexpr uint32_t NFC_HBITPERIOD = NFC_BITPERIOD * 2UL;

// for TCA0
static constexpr uint8_t TCA0_DIVISOR = 2;
static constexpr uint8_t SUBCARRIER_PER = ((((F_CPU/TCA0_DIVISOR)) / NFC_SUBCARRIER) + 0.5);
static constexpr uint8_t BITTIMEOUT_PER = ((((F_CPU/TCA0_DIVISOR)) / NFC_BITPERIOD) + 0.5);
static constexpr uint8_t HALFBIT_PER    = ((((F_CPU/TCA0_DIVISOR)) / NFC_HBITPERIOD) + 0.5);
static constexpr uint8_t HALFBIT_TIMER  = ((((F_CPU)) / NFC_HBITPERIOD)/3.5);

// for TCB0
static constexpr uint16_t BITPERIOD_MSK = 0b1111111111111110;
static constexpr uint16_t BITPERIOD_PER = (((F_CPU)) / NFC_BITPERIOD);
static constexpr uint16_t N_BITPERIODS(float n) {
  return ((uint16_t)(BITPERIOD_PER * n)) & BITPERIOD_MSK;
}

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
  // set 50% duty cycle
  TCA0.SPLIT.HCMP1 = (SUBCARRIER_PER/2);

  // setup TCA0 low as a timeout counter 
  TCA0.SPLIT.LPER = BITTIMEOUT_PER;

  // set divider to CLK/2 and enable
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV2_gc | TCA_SPLIT_ENABLE_bm;

  // PORTA.DIRSET = _BV(PIN4);
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

void Phy::begin() {
  AC0_setup();
  TCA0_setup();
  TCB0_setup();

  // FIXME: for manchester debug output
  PORTA.DIRSET = _BV(PIN1);
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
uint8_t Phy::receive() {
  TCA0.SPLIT.LPER = HALFBIT_PER;

  uint8_t timeouts = 0;
  resetBitTimeout();

  // wait for falling edge / start of frame
  while (!RX_EDGE_FLAG) {
    if (BIT_TIMEOUT_FLAG) {
      TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
      // if timeout return no data
      if (timeouts > 8) return 0;
      timeouts++;
    }
  }

  uint16_t delta = RX_EDGE_TIME;
  uint8_t bitPos = 0;
  uint8_t bytePos = 0;
  uint8_t lastBit = 0;
  buffer[0] = 0;

  // expecting odd parity bit, start at 1; (0 has an even # of ones)
  uint8_t parity = 1;

  // macro to handle checking whether it's the next byte, 
  //  skipping the parity bit, and writing in the data
  #define BIT(b) do {                   \
    if (bitPos == 8) {                  \
      /* TODO: check parity bit */                  \
      /*if ((b) != parity) {              \
        Serial.print("#P");             \
        for (uint8_t i = 0; i <= bytePos; i++) Serial.printHex(buffer[i]); \
        return 0;                       \
      }*/                                \
      parity = 1;                       \
      bitPos = 0;                       \
      bytePos++;                        \
      if (bytePos >= sizeof(buffer)) {  \
        return bytePos;                 \
      }                                 \
      buffer[bytePos] = 0;              \
    } else {                            \
      parity ^= (b);                    \
      buffer[bytePos] |= ((b)<<bitPos); \
      bitPos++;                         \
    }                                   \
  } while (0)

  do {
    if (RX_EDGE_FLAG) {
      delta = RX_EDGE_TIME & BITPERIOD_MSK;

      if (lastBit == 0) {
        // space->???
        if (delta <= N_BITPERIODS(1.0)) {
          // 1.0 time between bits = space
          BIT(0);
        } else if (delta <= N_BITPERIODS(1.5)) {
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
        if (delta <= N_BITPERIODS(1.0)) {
          // 1.0 time = mark
          BIT(1);
        } else if (delta <= N_BITPERIODS(1.5)) {
          // 1.5 time = 2 spaces
          BIT(0);
          BIT(0);
          lastBit = 0;
        } else if (delta <= N_BITPERIODS(2.0)) {
          // 2.0 time = space, mark
          BIT(0);
          BIT(1);
        } else {
          // delta > 2.0  == space, idle (end of message)
          // BIT(0);
          break;
        }
      }

      timeouts = 8;
    }

    if (BIT_TIMEOUT_FLAG) {
      timeouts--;
      TCA0.SPLIT.INTFLAGS |= TCA_SPLIT_LUNF_bm;
    }
  } while (timeouts);
  
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
  loop_until_bit_is_set(TCA0.SPLIT.INTFLAGS, TCA_SPLIT_LUNF_bp);
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
static force_inline void tx_high() {
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
static force_inline void tx_low() {
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

void Phy::transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0) {
  uint8_t bytePos = 0;
  uint8_t bitPos = skipBits;
  uint8_t parity = 0;

  // TODO: wait correct delay before transmit
  waitNBitPeriods(4);
  
  // setup bit timer to wait for half bit period (oops)
  TCA0.SPLIT.LPER = HALFBIT_PER;
  waitForBitTimer();
  // waitForBitTimer();

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


