#include "usart.h"

#include <avr/io.h>

static const char NUM_ALPHABET[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; 

void USART::begin(uint32_t baudrate) {
    uint32_t ubrr = F_CPU / 16 / baudrate - 1;

    // set baud rate
    UBRR0H = (uint8_t)(ubrr>>8);
    UBRR0L = (uint8_t)(ubrr);

    // enable tx only
    UCSR0B = (1<<TXEN0);

    // set frame format
    // 8 data bits, no parity, 1 stop bit
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
}

void USART::transmit(uint8_t data) {
    // wait for data register ready
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void USART::print(const char *str) {
    while (*str) {
        transmit(*str++);
    }
}

void USART::print(uint32_t num, uint8_t base) {
    if (num < base) {
        transmit(NUM_ALPHABET[num % base]);
    } else {
        print(num / base, base);
        transmit(NUM_ALPHABET[num % base]);
    }
}
