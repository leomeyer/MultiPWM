// Example for MultiPWM: A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// https://github.com/leomeyer/MultiPWM
//
// Copyright (c) Leo Meyer, leo@leomeyer.de
//
// This file is in the public domain.

// MultiPWM BareMinimum

#include "MultiPWM.h"

void setup() {
  if (MultiPWM.begin() >= 0) {
	  // set builtin LED brightness to about 1%
    MultiPWM.addChannel(LED_BUILTIN, MultiPWM.maximum() / 100);
    MultiPWM.apply();
  }
}

void loop() {}
