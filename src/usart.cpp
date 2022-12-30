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

#include "usart.h"
#include <avr/io.h>
#include <avr/common.h>

static const char NUM_ALPHABET[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; 
USART Serial;

#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

void USART::begin() {
  // set PORTB.PIN2 (TXD) as output
  PORTB.DIRSET |= _BV(PIN2);
  PORTB.OUTSET |= _BV(PIN2);
  
  // set baud rate
  // static constexpr uint16_t UBRR_VAL = ((64*20000000)/(8*9600));
  // USART0.BAUDH = UBRR_VAL >> 8;
  // USART0.BAUDL = UBRR_VAL & 0xFF;
  USART0.BAUD = USART0_BAUD_RATE(9600);

  // set frame format
  // 8 data bits, no parity, 1 stop bit
  USART0.CTRLC = 
    USART_CMODE_ASYNCHRONOUS_gc |
    USART_PMODE_DISABLED_gc |
    USART_SBMODE_1BIT_gc |
    USART_CHSIZE_8BIT_gc;

  // enable tx mode
  USART0.CTRLB |= USART_TXEN_bm;
}

void USART::putchar(uint8_t data) {
  // wait for data register ready
  while (!(USART0.STATUS & USART_DREIF_bm));
  USART0.TXDATAL = data;
}

void USART::print(const char *str) {
  while (*str) {
    putchar(*str++);
  }
}

void USART::print(uint32_t num, uint8_t base) {
  char buf[16];
  uint8_t digits = 0;

  do {
    buf[digits] = NUM_ALPHABET[num % base];
    digits++;
    if (digits >= sizeof(buf)) break;
    num /= base;
  } while (num > 0);

  while (digits) {
    digits--;
    putchar(buf[digits]);
  }
}

void USART::hexdump(uint8_t *buf, uint8_t sz) {
  print(sz, DEC); print(" bytes:\n");
  for (uint8_t i = 0; i < sz; i++) {
    putchar(NUM_ALPHABET[(buf[i] >> 4) & 0xF]);
    putchar(NUM_ALPHABET[(buf[i] >> 0) & 0xF]);
    putchar(' ');
  }
  putchar('\n');
}
