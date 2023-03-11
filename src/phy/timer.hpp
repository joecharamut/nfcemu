#pragma once

#include <stdint.h>
#include <avr/io.h>

class TimerA {
public:
  static void setup() {

  }
};

class TimerB {
public:
  /// @brief Setup TCB0 to capture when the comparator detects a falling edge (for modified miller decoding)
  static void setup() {
    // count mode periodic int mode
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;

    // set divider to CLK/1 and enable
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  }

  static void setTop(uint16_t val) {
    TCB0.CCMP = val;
  }

  static bool interrupted() {
    return TCB0.INTFLAGS & TCB_CAPT_bm;
  }

  static void clearInterrupt() {
    TCB0.INTFLAGS |= TCB_CAPT_bm;
  }

  static uint16_t count() {
    return TCB0.CNT;
  }

  static void clear() {
    clearInterrupt();
    TCB0.CNT = 0;
  }

  static uint16_t value() {
    return TCB0.CCMP;
  }
};
