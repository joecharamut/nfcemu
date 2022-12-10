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

#pragma once
#include <avr/common.h>

#define DEC 10
#define HEX 16

class USART {
public:
    void begin(uint32_t baudrate);
    void transmit(uint8_t data);
    void print(const char *str);
    void print(uint32_t num, uint8_t base = DEC);
    void hexdump(uint8_t *buf, uint8_t sz);
};

extern USART Serial;
