/*****************************************************************************
Written and Copyright (C) by Nicolas Kruse

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include "nfcemulator.h"
#include "logging.h"

#ifdef __AVR_ATmega328P__

// PORTD.5; Pin 11, OC0B
#define TX_ON() do { DDRD |= (1<<PD5); } while (0)
#define TX_OFF() do { DDRD &= ~(1<<PD5); } while (0)

// PORTD.7; Pin 13; AIN1
#define PULLUP_ON_AIN1() do { PORTD |= (1<<PD7); } while (0)
#define PULLUP_OFF_AIN1() do { PORTD &= ~(1<<PD7); } while (0)

#define SETUP_COMPARATOR() do { \
  /* PORTD.6; AIN0 => (Input, No pullup) */ \
  DDRD &= ~(1<<PD6); \
  PORTD &= ~(1<<PD6); \
  /* PORTD.7; AIN1 => (Input, No pullup) */ \
  DDRD &= ~(1<<PD7); \
  PORTD &= ~(1<<PD7); \
  /* Setup Analog Comparator, Enable (ACD=0), Set Analog Comparator \
     Interrupt Flag on Rising Output Edge (ACIS0, ACIS1) */ \
  ACSR = (0<<ACD) | (1<<ACIS0) | (1<<ACIS1); \
  \
  /* Disable digital input buffer on AIN0 and AIN1 */ \
  DIDR1 = (1<<AIN0D) | (1<<AIN1D); \
} while (0)

#else
#error Unsupported Processor
#endif

// Read Analog Comparator Interrupt flag [ACSR & (1<<ACI)]
#define IS_ACI_SET ((ACSR & (1<<ACI)) != 0)

// Clear Analog Comparator Interrupt flag [ACSR |= (1<<ACI)]
#define CLR_ACI() do { ACSR |= (1<<ACI); } while (0)

#define __always_inline __attribute__((always_inline))
#define force_inline inline __attribute__((always_inline))

static uint8_t RX_MASK[18] = {0, 1, 0, 2, 0, 4, 0, 8, 0, 16, 0, 32, 0, 64, 0, 128, 0, 0};
static uint8_t TX_MASK[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint16_t FDT_DELAY[2] = {FDT_DELAY_BIT0, FDT_DELAY_BIT1};

static uint8_t REQA[1] = {0x26};
static uint8_t WUPA[1] = {0x52};
static uint8_t HLTA[4] = {0x50,  0x00,  0x57,  0}; //'50' '00' CRC_A
static uint8_t ATQA[2] = {0x44, 0x00}; //Anticollision with udi size = double
static uint8_t SEL_CL1[2] =  {0x93, 0x20};
static uint8_t SEL_CL2[2] =  {0x95, 0x20};
static uint8_t CT_UID1[5] = {0x88, 0x04, 0xE3, 0xEF, 0}; //uid0 uid1 uid2 uid3 BCC
static uint8_t UID2[5] = {0xA2, 0xEF, 0x20, 0x80, 0}; //uid3 uid4 uid5 uid6 BCC
static uint8_t SAK_NC[3] = {0x04, 0xDA, 0x17}; //Select acknowledge uid not complete
static uint8_t SAK_C[3] = {0x00, 0xFE, 0x51}; //Select acknowledge uid complete, Type 2 (PICC not compliant to ISO/IEC 14443-4)
static uint8_t READ[1] = {0x30};
static uint8_t WRITE[1] = {0xA2};

uint8_t *sto; //Pointer to tag content
uint16_t stoSize; //Number of avalible bytes on the tag

uint8_t buffer[64];
uint8_t rCount = 0;

void addCrc16(uint8_t *Data, uint8_t Length);
void addBcc(uint8_t *Data);

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

#define CYCLES_PER_BIT ((uint16_t) (F_CPU / (106000)))

void setupNfcEmulator(uint8_t *storage, uint16_t storageSize) {
  // setup pins

  // debug leds
  GRN_SETUP();
  RED_SETUP();
  BLU_SETUP();

  // Port D Bit 5: OC0B, 
//  DDRD |= (1<<PD5); // output
//  PORTD &= ~(1<<PD5); // write LOW

//  // Port D Bit 6: AIN0, 
//  DDRD &= ~(1<<PD6); // input
//  PORTD &= ~(1<<PD6); // no pullup
//
//  // Port D Bit 7: AIN1, 
//  DDRD &= ~(1<<PD7); // input
//  PORTD &= ~(1<<PD7); // no pullup
  SETUP_COMPARATOR();

  //8 bit timer0: Toggle OC0B on Compare Match and CTC-Mode
  //for 847.5 kHz subcarrier
  TCCR0A = (1<<COM0B0) | (1<<WGM01);
  
  //clock divider for 8 bit timer0: clk/1 -> 13.5225 MHz
  // TCCR0B.CS00 = 1; // no prescale
  TCCR0B = (1<<CS00);
    
  //set up 847.5 kHz subcarrier for sending (8 bit timer0)
  // timer TOP value for CTC is only in OCR0A
  OCR0A = SUBC_OVF+1;
  // but also needs to be compared in OCR0B
  OCR0B = SUBC_OVF;
  
  //CTC-Mode and no clock divider for 16 bit timer1: clk/1
  TCCR1B = (1<<WGM12) | (1<<CS10);
  // set TOP val
  OCR1A = CYCLES_PER_BIT;
  
  //Setup Analog Comparator, Enable (ACD), Set Analog Comparator
  //Interrupt Flag on Rising Output Edge (ACIS0, ACIS1)
//  ACSR = (0<<ACD) | (1<<ACIS0) | (1<<ACIS1);
  
  addCrc16(HLTA, sizeof(HLTA));
  addBcc(CT_UID1);
  addBcc(UID2);
  
  stoSize = storageSize;
  sto = storage;
}

void addCrc16(uint8_t *Data, uint8_t Length) {
  uint8_t ch;
  uint16_t wCrc = 0x6363; // ITU-V.41

  do {
    ch = *Data++;
    
    ch = (ch^(uint8_t)((wCrc) & 0x00FF));
    ch = (ch^(ch<<4));
    wCrc = (wCrc >> 8)^((uint16_t)ch << 8)^((uint16_t)ch<<3)^((uint16_t)ch>>4);
  } while (--Length);

  *Data = (uint8_t) (wCrc & 0xFF);
  Data++;
  *Data = (uint8_t) ((wCrc >> 8) & 0xFF);
}


void addBcc(uint8_t *Data) {
  //add exclusive-OR of 4 bytes
  Data[4] = Data[0] ^ Data[1] ^ Data[2] ^ Data[3];
}

//inline void waitForBitend() __attribute__((always_inline));
force_inline void waitForBitend() {
  while(!(TIFR1 & (1<<OCF1A))); //Wait till end of bit-time
  TIFR1 |= (1<<OCF1A);
}

//inline void waitForOneBitTime() __attribute__((always_inline));
force_inline void waitForOneBitTime() {
  waitForBitend();
}

void txManchester(uint8_t *data, uint8_t length) {
  uint8_t txBytePos = 0;
  uint8_t txbitPos = 0;
  uint8_t parity = 0;
  
  TIFR1 |= (1<<OCF1A);
  
  //Send SOC
  waitForBitend();
  TX_ON();
  OCR1A = CLC_PBIT / 2 - 1; //Set Hi- and Low-Bit
  waitForOneBitTime();
  TX_OFF();

  do {
    if (TX_MASK[txbitPos] & data[txBytePos]) {
      waitForOneBitTime();
      TX_ON();
      parity ^= 1;
      waitForOneBitTime();
      TX_OFF();
    } else {
      waitForOneBitTime();
      TX_OFF();
      waitForOneBitTime();
      TX_ON();
    }
    
    txbitPos++;
    
    if (txbitPos > 7) {
      if (parity) {
        waitForOneBitTime();
        TX_OFF();
        waitForOneBitTime();
        TX_ON();
      } else {
        waitForOneBitTime();
        TX_ON();
        waitForOneBitTime();
        TX_OFF();
      }
      
      txBytePos++;
      txbitPos=0;
      parity = 0;
    }
  } while(txBytePos < length);
  
  //Send EOC
  waitForOneBitTime();
  TX_OFF();
}

//inline void resetRxFlags() __attribute__((always_inline));
void resetRxFlags() {
  TCNT1 = 0;
  TIFR1 |= (1<<OCF1A); //Clear Timer Overflow Flag 
  ACSR |= (1<<ACI); //Clear Analog Comparator Interrupt Flag
}


uint8_t rxMiller1() {
  BLU_ON();
  /**
   * NFC-A active data transmission:
   * 106kbps, 100% ASK, Modified Miller Coding
   * 
   * Data should be transmitted at 106kbps, each bit is 4 sub-bits(?) long,
   *   which means it should sample (106000 * 4) times per second,
   *   if TIMER1 is clocked at 13.56 MHz, it should work out to be
   *   13560000/(106000*4)=32 clock cycles per bit, 8 cycles per sub-bit
   * 
   * ACI will be set whem the signal changes from low to high
   * 
   * a 1 is always encoded as HI, HI, LO, HI --> starts HI, reads ACI=1
   * a 0 preceded by 0 will be LO, HI, HI, HI --> starts LO, reads ACI=1
   * a 0 preceded by 1 will be HI, HI, HI, HI --> starts HI, reads ACI=0
   * 
   * 
   * 
   **/

  static const uint8_t bitMask[] = { (1<<0), (1<<1), (1<<2), (1<<3), (1<<4), (1<<5), (1<<6), (1<<7) };
  uint8_t bitPos = 0;
  uint8_t bytePos = 0;

  // Wait for start of communication: pulldown at start of bit time (zero bit)
  do {
    if (ACSR & (1<<ACI)) resetRxFlags();
  } while(~TIFR1 & (1<<OCF1A));

  uint8_t lastBit = 0;
  while (1) {
    // clear byte in progress
    if (bitPos % 8 == 0) {
      buffer[bytePos] = 0;
    }
    // clear flags
    // resetRxFlags();

    // read initial sub-bit state
    uint8_t initialState = IS_ACI_SET;
    
    // clear flags again
    resetRxFlags();

    // wait 1 bit period
    while (~TIFR1 & (1<<OCF1A));

    // read again
    uint8_t finalState = IS_ACI_SET;

    if (initialState && finalState) {
      // '1' bit: initial HI, ACI set means LO->HI edge
      lastBit = 1;
      buffer[bytePos] |= bitMask[bitPos];
      bitPos++;
    } else if (initialState && !finalState) {
      // '0' bit after '1' bit: initial HI, no rising edge
      if (lastBit == 1) {
        lastBit = 0;
        bitPos++;
      } else {
        // End of Communication: HI, no rising edge after '0' bit
        break;
      }
    } else if (lastBit == 0 && !initialState && finalState) {
      // '0' bit after '0' bit: initial LO, yes rising edge
      lastBit = 0;
      bitPos++;
    }

    if (bitPos >= 8) {
      bytePos++;
      bitPos -= 8;
    }
  }

  BLU_OFF();
  return bytePos;
}

uint8_t rxMiller() {
  #if (F_CPU > 16000000)
    uint16_t t; //For hi cpu clock a 8 bit variable will overflow (CLCM > 0xFF)
  #else
    uint8_t t; //For low cpu clock computing time is to low for 16bit a variable
  #endif
  // dbg_rst();
  
  uint16_t cDown = 0x0FFF;
  uint8_t bytePos = 0;
  uint8_t hbitPos = 0;
  
  OCR1A = CLCL-1;
  resetRxFlags();
  buffer[0] = 0;

  //Wait for transmission end if there is data arriving
  do {
    if (ACSR & (1<<ACI)) resetRxFlags();
    // dbg_grn();
  } while(~TIFR1 & (1<<OCF1A));
  
  //Wait for transmission end if there is data arriving
  do {
    // dbg_red();
    if (TIFR1 & (1<<OCF1A)) {
      TCNT1 = 0;
      TIFR1 |= (1<<OCF1A);
      cDown--;
      if (!cDown) break;
    }
  } while(~ACSR & (1<<ACI));
  
  if (cDown) {
    // dbg_blu();
    resetRxFlags();
    do {
      if ((ACSR & (1<<ACI)) && (TCNT1 > 1)) {
        t = TCNT1;
        resetRxFlags();

        hbitPos += (t > CLCS) + (t > CLCM);

        if(hbitPos > 17) {
//          logHex(buffer[bytePos]);
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

  // dbg_rst();
  return bytePos;
}

void sendData(uint8_t block) {
  uint8_t i = 0;
  uint16_t pos = (uint16_t)block * 4;
  
  for(i=0; i < 16; i++) {
    if (pos >= stoSize) {
      buffer[i] = 0;
    } else {
      buffer[i] = sto[pos];
    }      

    pos++;
  }
  
  addCrc16(buffer, 16);
  txManchester(buffer, 18);
}

void receiveData(uint8_t block) {
  uint8_t i = 0;
  uint16_t pos = (uint16_t)block * 4;
  uint8_t crc1 = buffer[6];
  uint8_t crc2 = buffer[7];
  
  addCrc16(buffer, 6);
  
  if (buffer[6] == crc1 && buffer[7] == crc2) {
    for(i=2; i < 6; i++) {
      //byte 2-5 contains Data
      if (pos < stoSize) sto[pos] = buffer[i];
      pos++;
    }
    
    buffer[0] = 0x0A; //ACK
    txManchester(buffer, 1);
  } else {
    buffer[0] = 0x01; //NAK for CRC error
    txManchester(buffer, 1);
  }
}

int checkForNfcReader() {
  uint8_t bytes = 1;
  uint8_t state = 0;
  uint8_t cdow = 8;

  if (IS_ACI_SET) {
    GRN_ON();
    // 13.56 MHz carrier available?
    // Deactivate pull up to increase sensitivity
    PULLUP_OFF_AIN1();
    
    while(cdow > 0) {
      bytes = rxMiller();

      // todo: fix this

      // if (bytes==1) RED_ON();

      if ((state & 7) == S_READY) {
        BLU_ON();
      } else {
        BLU_OFF();
      }

      if ((state & 7) == S_ACTIVE) {
        RED_ON();
      } else {
        RED_OFF();
      }

      if ((state & 7) == S_READY) {
        if (buffer[0] == SEL_CL1[0] && buffer[1] == SEL_CL1[1] ) {
          txManchester(CT_UID1, sizeof(CT_UID1));
        } else if (buffer[0] == SEL_CL2[0] && buffer[1] == SEL_CL2[1] ) {
          txManchester(UID2, sizeof(UID2));
        } else if (buffer[0] == SEL_CL1[0] && buffer[1] == 0x70 ) {
          txManchester(SAK_NC, sizeof(SAK_NC));
        } else if (buffer[0] == SEL_CL2[0] && buffer[1] == 0x70 ) {
          txManchester(SAK_C, sizeof(SAK_C));
          state++; //Set state to ACTIVE
        } else {
          state &= 8; //Set state to IDLE/HALT
        }
      } else if ((state & 7) == S_ACTIVE) {
        if (buffer[0] == READ[0]) {
          sendData(buffer[1]);
        } else if (buffer[0] == WRITE[0]) {
          receiveData(buffer[1]);    
        } else if (buffer[0] == HLTA[0] && buffer[2] == HLTA[2] ) {
          state = S_HALT;
        } else if(bytes) {
          state &= 8; //Set state to IDLE/HALT
        }
      } else if (bytes == 1 && (buffer[0] == REQA[0] || buffer[0] == WUPA[0])) {
        //state == S_IDLE
        txManchester(ATQA, sizeof(ATQA));
        state = (state & 8) + S_READY; //Set state to READY
      }
      
      cdow -= (bytes == 0);
      // if (bytes) hexdump(buffer, bytes);
    }
    // Activate pull up to prevent noise from toggling the comparator
    PULLUP_ON_AIN1();
  } else {
    GRN_OFF();
  }
  CLR_ACI();
  
  return 0;
}
