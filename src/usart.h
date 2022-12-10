#pragma once

#include <avr/common.h>

#define DEC 10
#define HEX 16

class USART {
public:
    void begin(uint32_t baudrate);
    void transmit(uint8_t data);
    void puts(char *str);
    void printNum(uint32_t num, uint8_t base);

    void print(const char *str);
    void print(uint32_t num, uint8_t base = 10);
};


