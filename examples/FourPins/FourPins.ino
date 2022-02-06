// Example for MultiPWM: A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// https://github.com/leomeyer/MultiPWM
//
// Copyright (c) Leo Meyer, leo@leomeyer.de
//
// This file is in the public domain.

// MultiPWM FourPins example

#include "MultiPWM.h"

void setup() {
  Serial.begin(9600);
  Serial.println("MultiPWM FourPins example");
  
  if (MultiPWM.begin() >= 0) {
    MultiPWM.set(MultiPWM.addChannel(4), 10);
    MultiPWM.setf(MultiPWM.addChannel(5), 0.1f);
    MultiPWM.setPercent(MultiPWM.addChannel(6), 20);
    MultiPWM.set(MultiPWM.addChannel(7), MultiPWM.maximum());
    MultiPWM.apply();
  }
}

void loop() {}