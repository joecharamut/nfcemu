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

#define force_inline inline __attribute__((always_inline))
#define __packed __attribute__((packed))

//#define USE_OC0A
#define USE_OC0B

static force_inline void enableT0Out() {
  // set OC0x to OUTPUT mode
  #ifdef USE_OC0A
  DDRD |= (1<<PD6);
  #endif
  
  #ifdef USE_OC0B
  DDRD |= (1<<PD5);
  #endif
}

static force_inline void disableT0Out() {
  // set OC0x to INPUT mode
  #ifdef USE_OC0A
  DDRD &= ~(1<<PD6);
  #endif
  
  #ifdef USE_OC0B
  DDRD &= ~(1<<PD5);
  #endif
}

static force_inline void setupTimer0() {
  // clear timer registers
  TCCR0A = 0;
  TCCR0B = 0;
  
  // WGM02=0, WGM01=1, WGM00=0 - CTC Mode
  TCCR0A = (1<<WGM01);

  // CS02=0, CS01=0, CS00=1 - clk/1 (no prescale)
  TCCR0B = (1<<CS00);

  #ifdef USE_OC0A
  // Toggle OC0A on compare match
  TCCR0A |= (1<<COM0A0);
  #endif

  #ifdef USE_OC0B
  // Toggle OC0B on compare match
  TCCR0A |= (1<<COM0B0);
  #endif

  disableT0Out();
}

static force_inline void setupTimer1() {
  // CTC-Mode and no clock divider for 16 bit timer1: clk/1
  TCCR1B = (1<<WGM12) | (1<<CS10);
}

static force_inline void setupAnalog() {
  // Port D Bit 6: AIN0, 
  DDRD &= ~(1<<PD6); // input
  PORTD &= ~(1<<PD6); // no pullup

  // Port D Bit 7: AIN1, 
  DDRD &= ~(1<<PD7); // input
  PORTD &= ~(1<<PD7); // no pullup

  // Setup Analog Comparator, Enable (ACD=0), Set Analog Comparator
  // Interrupt Flag on Rising Output Edge (ACIS0, ACIS1)
  ACSR = (0<<ACD) | (1<<ACIS0) | (1<<ACIS1);
}

#define AC_INTERRUPT_SET ((ACSR & (1<<ACI)) != 0)
static force_inline void clearAcInterrupt() {
  ACSR |= (1<<ACI);
}

// PORTD.7; Pin 13; AIN1
static force_inline void enableAin1Pullup() { PORTD |= (1<<PD7); }
static force_inline void disableAin1Pullup() { PORTD &= ~(1<<PD7); }

// Timer 1 Output Compare A Match
#define IS_OCF1A_SET ((TIFR1 & (1<<OCF1A)) != 0)
static force_inline void clearOCF1A() { TIFR1 |= (1<<OCF1A); }

// pin 14
#define GRN_SETUP() do { DDRB  |=  (1<<PB0); } while (0)
#define GRN_ON()    do { PORTB |=  (1<<PB0); } while (0)
#define GRN_OFF()   do { PORTB &= ~(1<<PB0); } while (0)

// pin 15
#define RED_SETUP() do { DDRB  |=  (1<<PB1); } while (0)
#define RED_ON()    do { PORTB |=  (1<<PB1); } while (0)
#define RED_OFF()   do { PORTB &= ~(1<<PB1); } while (0)

// pin 16
#define BLU_SETUP() do { DDRB  |=  (1<<PB2); } while (0)
#define BLU_ON()    do { PORTB |=  (1<<PB2); } while (0)
#define BLU_OFF()   do { PORTB &= ~(1<<PB2); } while (0)

void Emulator::setup(uint8_t *storage, uint16_t storageSize) {
  this->storage = storage;
  this->storageSize = storageSize;

  setupTimer0();
  setupTimer1();
  setupAnalog();
  GRN_SETUP();
  RED_SETUP();
  BLU_SETUP();
}

static void genBcc(uint8_t *buf) {
  //add exclusive-OR of 4 bytes
  buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
}

void Emulator::setUid(uint8_t *uid, uint8_t uidSize) {
  if (uidSize <= 4) {
    // 4 byte uid = Cascade Lv 1
    
    memcpy(nfcid[0], uid, 4);
    genBcc(nfcid[0]);
    nfcid1Size = 0;
  } else if (uidSize <= 7) {
    // 7 byte uid = Cascade Lv 2

    nfcid[0][0] = 0x88; // Cascade Tag = 88h
    memcpy(nfcid[0], uid, 3);
    genBcc(nfcid[0]);

    memcpy(nfcid[1], uid+3, 4);
    genBcc(nfcid[1]);
    nfcid1Size = 1;
  } else if (uidSize <= 10) {
    // 10 byte uid = Cascade Lv 3

    nfcid[0][0] = 0x88; // Cascade Tag = 88h
    memcpy(nfcid[0], uid, 3);
    genBcc(nfcid[0]);

    nfcid[1][0] = 0x88; // Cascade Tag = 88h
    memcpy(nfcid[1], uid+3, 3);
    genBcc(nfcid[1]);

    memcpy(nfcid[2], uid+6, 4);
    genBcc(nfcid[2]);
    nfcid1Size = 2;
  } else {
    // uidSize > 10 bytes not supported
  }
}

#define RFID_FREQU 13560000UL
#define CLC_PBIT (uint16_t)(128.0 * F_CPU / RFID_FREQU + 0.5)
#define CLCS CLC_PBIT * 5 / 4
#define CLCM CLC_PBIT * 7 / 4
#define CLCL CLC_PBIT * 9 / 4

#define FDT_DELAY_BD9 (uint16_t)(9.0 * 128 * F_CPU / RFID_FREQU - 1) //Nr of cycles-1 for 9 bit durations
#define FDT_DELAY_BIT0 (FDT_DELAY_BD9 + 20 - CLCL)
#define FDT_DELAY_BIT1 (FDT_DELAY_BD9 + 84 - CLCL)

#define SUBC_OVF (uint8_t)(F_CPU / 847500.0 / 2.0 + 0.5 - 1) //847500 Hz Subcarrier

static uint8_t RX_MASK[18] = {0, 1, 0, 2, 0, 4, 0, 8, 0, 16, 0, 32, 0, 64, 0, 128, 0, 0};
static uint8_t TX_MASK[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint16_t FDT_DELAY[2] = {FDT_DELAY_BIT0, FDT_DELAY_BIT1};

static force_inline void resetRxFlags() {
  TCNT1 = 0;
  TIFR1 |= (1<<OCF1A); //Clear Timer Overflow Flag 
  ACSR |= (1<<ACI); //Clear Analog Comparator Interrupt Flag
}

uint8_t Emulator::rxMiller() {
  #if (F_CPU > 16000000)
    uint16_t t; //For hi cpu clock a 8 bit variable will overflow (CLCM > 0xFF)
  #else
    uint8_t t; //For low cpu clock computing time is to low for 16bit a variable
  #endif
  
  uint16_t cDown = 0x0FFF;
  uint8_t bytePos = 0;
  uint8_t hbitPos = 0;
  
  OCR1A = CLCL-1;
  buffer[0] = 0;

  //Wait for transmission end if there is data arriving
  do {
    if (ACSR & (1<<ACI)) resetRxFlags();
  } while(~TIFR1 & (1<<OCF1A));
  
  //Wait for transmission end if there is data arriving
  // Wait for start-of-communication condition:
  //   Modulation gets pulled low at the start of a bit period to indicate SOC,
  //   which will set the comparator interrupt as a rising edge
  do {
    if (TIFR1 & (1<<OCF1A)) {
      TCNT1 = 0;
      TIFR1 |= (1<<OCF1A);
      cDown--;
      if (!cDown) break;
    }
  } while(~ACSR & (1<<ACI));
  
  if (cDown) {
    resetRxFlags();
    do {
      if ((ACSR & (1<<ACI)) && (TCNT1 > 1)) {
        t = TCNT1;
        resetRxFlags();

        hbitPos += (t > CLCS) + (t > CLCM);

        if(hbitPos > 17) {
          bytePos++;
          hbitPos -= 18;
          buffer[bytePos] = 0;
        }
       
        buffer[bytePos] |= RX_MASK[hbitPos];
            
        hbitPos += 2;
      } //34 or 41 (hbitPos > 17) click cycles
    } while(~TIFR1 & (1<<OCF1A));
  }
  
  OCR1A = FDT_DELAY[hbitPos & 1]; //Set delay for answer
  TIFR1 |= (1<<OCF1A);
  
  if (hbitPos > 7) bytePos++;

  return bytePos;
}

static force_inline void waitForTimer1() {
  // loop until OCF1A is set
  while (!(TIFR1 & (1<<OCF1A)));
  // write to OCF1A to clear it
  TIFR1 |= (1<<OCF1A);
}

// PORTD.5; Pin 11, OC0B
#define TX_ON() do { DDRD |= (1<<PD5); } while (0)
#define TX_OFF() do { DDRD &= ~(1<<PD5); } while (0)

void Emulator::txManchester(const uint8_t *buf, uint8_t count) {
  uint8_t txBytePos = 0;
  uint8_t txbitPos = 0;
  uint8_t parity = 0;
  
  TIFR1 |= (1<<OCF1A);
  
  //Send SOC
  waitForTimer1();
  TX_ON();
  OCR1A = CLC_PBIT / 2 - 1; //Set Hi- and Low-Bit
  waitForTimer1();
  TX_OFF();

  do {
    if (TX_MASK[txbitPos] & buf[txBytePos]) {
      waitForTimer1();
      TX_ON();
      parity ^= 1;
      waitForTimer1();
      TX_OFF();
    } else {
      waitForTimer1();
      TX_OFF();
      waitForTimer1();
      TX_ON();
    }
    
    txbitPos++;
    
    if (txbitPos > 7) {
      if (parity) {
        waitForTimer1();
        TX_OFF();
        waitForTimer1();
        TX_ON();
      } else {
        waitForTimer1();
        TX_ON();
        waitForTimer1();
        TX_OFF();
      }
      
      txBytePos++;
      txbitPos=0;
      parity = 0;
    }
  } while(txBytePos < count);
  
  //Send EOC
  waitForTimer1();
  TX_OFF();
}

typedef enum {
  ALL_REQ = 0x52,
  SENS_REQ = 0x26,
} nfc_short_cmd_t;

typedef struct __packed {
  nfc_short_cmd_t cmd;
} nfc_short_frame_t;

typedef enum {
  NFC_CL1 = 0b0011,
  NFC_CL2 = 0b0101,
  NFC_CL3 = 0b0111,
} collision_level_t;

typedef enum {
  SDD_REQ = 0b1001,
} sdd_cmd_t;

typedef struct __packed {
  collision_level_t col : 4;
  sdd_cmd_t cmd : 4;
} nfc_sel_cmd_t;

typedef struct __packed {
  collision_level_t col : 4;
  sdd_cmd_t cmd : 4;
  uint8_t bit_count : 4;
  uint8_t byte_count : 4;
} nfc_sdd_frame_t;


static const uint8_t ATQA[] = {0x44, 0x00};
static const uint8_t SLP_REQ[] = {0x50, 0x00};

static const uint8_t SENS_RES[3][2] = {
  { 0b00000100, 0b00000000 }, // SEL_RES CL1 ( 4 UID bytes)
  { 0b01000100, 0b00000000 }, // SEL_RES CL2 ( 7 UID bytes)
  { 0b10000100, 0b00000000 }, // SEL_RES CL3 (10 UID bytes)
};

// comparator interrupt set on rising edge == start of frame condition
#define START_OF_FRAME (ACSR & (1<<ACI))

void Emulator::tick() {
  // big TODO: need to wait after rx-ing data
  // spec says minimum response time for tag->device is 87us

  GRN_OFF();
  RED_OFF();
  BLU_OFF();

  do {
    switch (state) {
      case ST_IDLE:       handleIdle();   break;
      case ST_READY_CL1:  handleReady1(); break;
      case ST_READY_CL2:  handleReady2(); break;
      case ST_READY_CL3:  handleReady2(); break;
      case ST_ACTIVE:     handleActive(); break;
      case ST_SLEEP:      handleSleep();  break;

      default: break;
    }
  } while (state != ST_IDLE);
}

void Emulator::handleIdle() {
  /**
   * IDLE state:
   *  if recieve ALL_REQ or SENS_REQ: switch to READY state
   *  else: stay in IDLE state
   **/

  if (START_OF_FRAME) {
    disableAin1Pullup();
    uint8_t read = rxMiller();
    // ALL_REQ/SENS_REQ are short frames = 1 bytes
    if (read == 1) {
      nfc_short_frame_t *short_frame = (nfc_short_frame_t *) buffer;
      if (short_frame->cmd == SENS_REQ || short_frame->cmd == ALL_REQ) {
        // after recieving ALL_REQ or SENS_REQ, 
        //  nfc device must transmit SENS_RES and enter READY state
        txManchester(SENS_RES[nfcid1Size], sizeof(SENS_RES[nfcid1Size]));
        state = ST_READY_CL1;
        GRN_ON();
        return;
      }
    }
    // enableAin1Pullup();
    clearAcInterrupt();
  }
}

void Emulator::handleReady1() {
  /**
   * READY_CL1 state:
   *  if recieve SDD_REQ CL1, send NFCID1 CL1 and stay in READY_CL1 state
   *  else if recieve SEL_REQ CL1 with matching NFCID1 CL1:
   *    if normal size NFCID1: enter ACTIVE state
   *    else (double/triple size NFCID1): enter READY_CL2 state
   *  else: enter IDLE state
   **/

  // if (START_OF_FRAME) {
    // disableAin1Pullup();
    uint8_t read = rxMiller();
    // SDD_REQ/SEL_REQ frames will be 2+ bytes long
    if (read == 1) {
      nfc_short_frame_t *short_frame = (nfc_short_frame_t *) buffer;
      if (short_frame->cmd == SENS_REQ || short_frame->cmd == ALL_REQ) {
        // after recieving ALL_REQ or SENS_REQ, 
        //  nfc device must transmit SENS_RES and enter READY state
        txManchester(SENS_RES[nfcid1Size], sizeof(SENS_RES[nfcid1Size]));
        state = ST_READY_CL1;
        GRN_ON();
        return;
      }
    } else if (read >= 2) {
      nfc_sdd_frame_t *sdd_frame = (nfc_sdd_frame_t *) buffer;
      // frame format:
      //   byte 0 bits 7..4: SDD_REQ [0b1001]
      //   byte 0 bits 3..0: CLx [0b0011 = CL1, 0b0101 = CL2, 0b0111 = CL3]
      //   byte 1 bits 7..4: byte count [0x2 = 0 extra bytes, 0x7 = 5 extra bytes]
      //   byte 1 bits 3..0: bit count [always 0 for NFCA?]
      if (sdd_frame->cmd == SDD_REQ && sdd_frame->col == NFC_CL1) {
        if (sdd_frame->byte_count == 0x2) {
          // 0 extra bytes: Valid SDD_REQ CL1 command
          txManchester(nfcid[0], sizeof(nfcid[0]));
          Serial.print("SDD_REQ CL1\n");
        } else if (sdd_frame->byte_count == 0x7) {
          // 5 extra bytes: Potentially valid SEL_REQ CL1 command
          Serial.print("SEL_REQ CL1\n");
          if (memcmp(buffer + 2, nfcid[0], sizeof(nfcid[0])) == 0) {
            // if matching uid: send SEL_RES resp
            uint8_t sel_res = 0b00000000;
            //                  -tt--c--
            // - = unused
            // tt = tag type (00 = type 2, 01 = type 4A, 10 = nfc-dep, 11 = type 4A + nfc-dep)
            // c = cascade bit (1 = not complete, 0 = complete)
            if (nfcid1Size != 0) {
              // CL2 or CL3 uid, set cascade bit and go to next READY_CLx state
              sel_res |= (1 << 3);
              txManchester(&sel_res, 1);
              state = ST_READY_CL2;
              RED_ON();
              Serial.print("CL1->CL2\n");
            } else {
              // CL1 uid, tx response and switch to ACTIVE state
              txManchester(&sel_res, 1);
              state = ST_ACTIVE;
              Serial.print("CL1->ACTIVE\n");
            }
          }
        }
      }
    }
    // enableAin1Pullup();
  // }
  // clearAcInterrupt();
}

void Emulator::handleReady2() {
  /**
   * READY_CL2 state:
   *  if recieve SDD_REQ CL2, send NFCID1 CL2 and stay in READY_CL2 state
   *  else if recieve SEL_REQ CL2 with matching NFCID1 CL2:
   *    if double size NFCID1: enter ACTIVE state
   *    else (triple size NFCID1): enter READY_CL3 state
   *  else: enter IDLE state
   **/

  if (START_OF_FRAME) {
    disableAin1Pullup();
    uint8_t read = rxMiller();
    // SDD_REQ/SEL_REQ frames will be 2+ bytes long
    if (read >= 2) {
      nfc_sdd_frame_t *sdd_frame = (nfc_sdd_frame_t *) buffer;
      // frame format:
      //   byte 0 bits 7..4: SDD_REQ [0b1001]
      //   byte 0 bits 3..0: CLx [0b0011 = CL1, 0b0101 = CL2, 0b0111 = CL3]
      //   byte 1 bits 7..4: byte count [0x2 = 0 extra bytes, 0x7 = 5 extra bytes]
      //   byte 1 bits 3..0: bit count [always 0 for NFCA?]
      if (sdd_frame->cmd == SDD_REQ && sdd_frame->col == NFC_CL2) {
        if (sdd_frame->byte_count == 0x2) {
          // 0 extra bytes: Valid SDD_REQ CL2 command
          txManchester(nfcid[1], sizeof(nfcid[1]));
        } else if (sdd_frame->byte_count == 0x7) {
          // 5 extra bytes: Potentially valid SEL_REQ CL2 command
          if (memcmp(buffer + 2, nfcid[1], sizeof(nfcid[1])) == 0) {
            // if matching uid: send SEL_RES resp
            uint8_t sel_res = 0b00000000;
            //                  -tt--c--
            // - = unused
            // tt = tag type (00 = type 2, 01 = type 4A, 10 = nfc-dep, 11 = type 4A + nfc-dep)
            // c = cascade bit (1 = not complete, 0 = complete)
            if (nfcid1Size != 1) {
              // CL3 uid, set cascade bit and go to next READY_CLx state
              sel_res |= (1 << 3);
              txManchester(&sel_res, 1);
              state = ST_READY_CL3;
            } else {
              // CL2 uid, tx response and switch to ACTIVE state
              txManchester(&sel_res, 1);
              state = ST_ACTIVE;
              BLU_ON();
            }
          }
        }
      }
    }
    enableAin1Pullup();
  }
  clearAcInterrupt();
}

void Emulator::handleReady3() {
  /**
   * READY_CL3 state:
   *  if recieve SDD_REQ CL3, send NFCID1 CL3 and stay in READY_CL3 state
   *  else if recieve SEL_REQ CL3 with matching NFCID1 CL3:
   *    if triple size NFCID1: enter ACTIVE state
   *  else: enter IDLE state
   **/

  if (START_OF_FRAME) {
    disableAin1Pullup();
    uint8_t read = rxMiller();
    // SDD_REQ/SEL_REQ frames will be 2+ bytes long
    if (read >= 2) {
      nfc_sdd_frame_t *sdd_frame = (nfc_sdd_frame_t *) buffer;
      // frame format:
      //   byte 0 bits 7..4: SDD_REQ [0b1001]
      //   byte 0 bits 3..0: CLx [0b0011 = CL1, 0b0101 = CL2, 0b0111 = CL3]
      //   byte 1 bits 7..4: byte count [0x2 = 0 extra bytes, 0x7 = 5 extra bytes]
      //   byte 1 bits 3..0: bit count [always 0 for NFCA?]
      if (sdd_frame->cmd == SDD_REQ && sdd_frame->col == NFC_CL3) {
        if (sdd_frame->byte_count == 0x2) {
          // 0 extra bytes: Valid SDD_REQ CL3 command
          txManchester(nfcid[2], sizeof(nfcid[2]));
        } else if (sdd_frame->byte_count == 0x7) {
          // 5 extra bytes: Potentially valid SEL_REQ CL3 command
          if (memcmp(buffer + 2, nfcid[2], sizeof(nfcid[2])) == 0) {
            // if matching uid: send SEL_RES resp
            uint8_t sel_res = 0b00000000;
            //                  -tt--c--
            // - = unused
            // tt = tag type (00 = type 2, 01 = type 4A, 10 = nfc-dep, 11 = type 4A + nfc-dep)
            // c = cascade bit (1 = not complete, 0 = complete)
            if (nfcid1Size == 2) {
              // CL3 uid, tx response and switch to ACTIVE state
              txManchester(&sel_res, 1);
              state = ST_ACTIVE;
            }
          }
        }
      }
    }
    enableAin1Pullup();
  }
  clearAcInterrupt();
}

void Emulator::handleActive() {
  /**
   * ACTIVE state:
   *  todo
   **/

  if (START_OF_FRAME) {
    disableAin1Pullup();
    uint8_t read = rxMiller();
    
    if (read) {
      Serial.hexdump(buffer, read);
    }

    enableAin1Pullup();
  }
  clearAcInterrupt();
}

void Emulator::handleSleep() {
  /**
   * SLEEP state:
   *  if recieve ALL_REQ: switch to READY state
   *  else: stay in SLEEP state
   **/

  if (START_OF_FRAME) {
    disableAin1Pullup();
    uint8_t read = rxMiller();
    // ALL_REQ is a short frame = 1 bytes
    if (read == 1) {
      nfc_short_frame_t *short_frame = (nfc_short_frame_t *) buffer;
      if (short_frame->cmd == ALL_REQ) {
        // after recieving ALL_REQ, 
        //  nfc device must transmit SENS_RES and enter READY state
        txManchester(SENS_RES[nfcid1Size], sizeof(SENS_RES[nfcid1Size]));
        state = ST_READY_CL1;
      }
    }
    enableAin1Pullup();
  }
  clearAcInterrupt();
}
