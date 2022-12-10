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

#include <avr/io.h>
#include "logging.h"

#include "usart.h"
USART Serial;

void logInit() {
  Serial.begin(9600);
}

void logStr(const char *str) {
  Serial.print(str);
}

void logChr(char c) {
  Serial.transmit(c);
}

void logInt(uint8_t v) {
  Serial.print(v, DEC);
}

void logHex(uint8_t v) {
  Serial.print(v, HEX);
}

void hexdump(uint8_t *buf, uint8_t sz) {
  Serial.print(sz, DEC); Serial.print(" bytes:\n");
  for (uint8_t i = 0; i < sz; i += 16) {
    Serial.print(i);
    Serial.print("\t");
    for (uint8_t j = 0; j < 16 && (i+j)<sz; j++) {
      Serial.print(buf[i+j], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
  Serial.print("\n");
}
