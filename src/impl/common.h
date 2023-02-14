#pragma once

#include <stdint.h>

typedef void (*PhyReceiveFnPtr)(uint8_t);

/// @brief Physical Data Layer functions
class PhyInterface {
public:
  /// @brief Setup phy layer (peripherals, etc)
  virtual void begin() = 0;

  /// @brief Transmit some data
  /// @param buf data buffer
  /// @param count number of bytes
  /// @param skipBits TODO
  virtual void transmit(const uint8_t *buf, uint8_t count, uint8_t skipBits = 0) = 0;

  /// @brief Try to receive some data
  /// @return number of bytes received
  virtual uint8_t receive() = 0;
  
  /// @brief Set callback for received data
  /// @param fn the callback function
  virtual void onReceive(PhyReceiveFnPtr fn) = 0;

  /// @brief Read one byte from buffer
  /// @return a byte
  virtual uint8_t read() = 0;

  /// @brief Get a pointer to the received data buffer
  /// @return pointer to the received data buffer
  virtual uint8_t *getBuffer() = 0;
};

