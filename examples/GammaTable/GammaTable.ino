// Example for MultiPWM: A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// https://github.com/leomeyer/MultiPWM
//
// Copyright (c) Leo Meyer, leo@leomeyer.de
//
// This file is in the public domain.

// MultiPWM gamma table example

#include "MultiPWM.h"

void setup() {
  Serial.begin(9600);
  Serial.println(F("MultiPWM gamma table example"));

  MultiPWM.setGammaTable(MultiPWM.DefaultGamma);
}

void loop() {
  for (uint8_t resolution = MultiPWM.R_8BITS; resolution <= MultiPWM.R_16BITS; resolution++) {
    Serial.print(F("Resolution: "));
    Serial.print(resolution);
    Serial.print(F(" bits, frequency: "));

    long f = MultiPWM.begin(resolution);
    if (f < 0) {
      Serial.print(F("Error: "));
      Serial.println(f);
      continue;
    }
    Serial.print(f);
    Serial.println(F(" Hz"));

    int8_t e = MultiPWM.addChannel(LED_BUILTIN);
    if (e < 0) {
      Serial.print(F("Error adding channel: "));
      Serial.println(e);
      continue;
    }
    
    for (uint16_t v = 0; v <= MultiPWM.maximum(); v++) {
      MultiPWM.set(0, v).apply();
      delay(5);
    }

    MultiPWM.reset();
  }
}
