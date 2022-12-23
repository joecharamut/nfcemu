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

static const char NUM_ALPHABET[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; 
USART Serial;

void USART::begin(uint32_t baudrate) {
    uint16_t ubrr = (uint16_t)(F_CPU / 16 / baudrate - 1);

    // set baud rate
    USART0.BAUD = ubrr;

    // enable tx only
    USART0.CTRLB |= USART_TXEN_bm;

    // set frame format
    // 8 data bits, no parity, 1 stop bit
    USART0.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
}

void USART::transmit(uint8_t data) {
    // wait for data register ready
    while (!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = data;
}

void USART::print(const char *str) {
    while (*str) {
        transmit(*str++);
    }
}


void USART::print(uint32_t num, uint8_t base) {
    uint8_t digits = 0;

    do {
        printBuf[digits] = NUM_ALPHABET[num % base];
        digits++;
        if (digits >= sizeof(printBuf)) break;
        num /= base;
    } while (num > 0);

    while (digits) {
        digits--;
        transmit(printBuf[digits]);
    }
}

void USART::hexdump(uint8_t *buf, uint8_t sz) {
  print(sz, DEC); print(" bytes:\n");
  for (uint8_t i = 0; i < sz; i++) {
    print(buf[i], HEX);
    print(" ");
  }
  print("\n");
}
