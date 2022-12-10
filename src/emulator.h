#pragma once
#include <avr/io.h>

namespace NfcEmu {

typedef enum {
  ST_IDLE,
  ST_READY_CL1,
  ST_READY_CL2,
  ST_READY_CL3,
  ST_ACTIVE,
  ST_SLEEP,
} emu_state_t;

class Emulator {
public:
  void setup(uint8_t *storage, uint16_t storageSize);
  void tick();

  void setUid(uint8_t *uid, uint8_t uidSize);

private:
  uint8_t *storage;
  uint16_t storageSize;

  uint8_t nfcid1Size;
  uint8_t nfcid[3][5];

  uint8_t buffer[64];

  emu_state_t state;

  uint8_t rxMiller();
  void txManchester(const uint8_t *buf, uint8_t count);
  uint8_t communicate();

  void handleIdle();
  void handleReady1();
  void handleReady2();
  void handleReady3();
  void handleActive();
  void handleSleep();
};

};
