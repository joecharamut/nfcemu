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
