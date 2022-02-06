// Example for MultiPWM: A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// https://github.com/leomeyer/MultiPWM
//
// Copyright (c) Leo Meyer, leo@leomeyer.de
//
// This file is in the public domain.

// MultiPWM performance test

// use all available channels
#define MULTIPWM_MAXCHANNELS NUM_DIGITAL_PINS
#define MULTIPWM_MEASURE_OVERHEAD

// #define MULTIPWM_DEBUG_SERIAL   Serial
#include "MultiPWM.h"

uint32_t rawSpeed;

#pragma GCC push_options
#pragma GCC optimize ("O0")
// counts loop iterations per second (not entirely accurate)
uint32_t getSpeed(uint16_t frequency) {
  unsigned long start = micros();
  uint32_t iterations = 0;
  // lower frequencies require more accurate measurements
  uint32_t iters = F_CPU / 10000;
  if (frequency < 500)
    iters /= 10;
  if (frequency < 50)
    iters /= 10;
  if (frequency < 5)
    iters /= 10;
  // busy wait for approximately one second
  do {
    for (uint32_t i = 0; i < iters; i++)
      iterations++;
  } while (micros() - start < 1000000UL);
  return iterations;
}
#pragma GCC pop_options

void setup() {
  Serial.begin(9600);

  Serial.println(F("Testing MultiPWM approximate CPU load"));
  Serial.print(F("CPU speed: "));
  Serial.print(F_CPU);
  Serial.println(F(" Hz"));

  MultiPWM.setGammaTable(MultiPWM.DefaultGamma);

  randomSeed(analogRead(0));
}

void loop() {
  for (Prescaler prescaler = MultiPWM.P_8; prescaler <= MultiPWM.P_1024; prescaler++) {
    for (uint8_t resolution = MultiPWM.R_8BITS; resolution <= MultiPWM.R_16BITS; resolution++) {
      if (&prescaler < 0)
        continue;
        
      Serial.print(F("MultiPWM (prescaler: "));
      Serial.print(&prescaler);
      Serial.print(F(" ["));
      Serial.print((int)prescaler);
      Serial.print(F("], resolution: "));
      Serial.print(resolution);
      Serial.print(F(" bits) -> "));

      int16_t f = MultiPWM.begin(resolution, prescaler);
      if (f < 0) {
        Serial.print(F("error "));
        Serial.println(f);
        continue;
      }

      Serial.print(F("frequency: "));
      float freq = MultiPWM.getFrequency();
      Serial.print(freq);
      Serial.println(F(" Hz"));

      // measure speed without any channels
      rawSpeed = getSpeed(f);

      Serial.print(F("CPU speed comparison value: "));
      Serial.println(rawSpeed);
      
      for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
        // add channel with "random" value
        int16_t e = MultiPWM.addChannel(pin, (uint16_t)random(1, MultiPWM.maximum()));
        if (e < 0) {
          Serial.print(F("Error adding channel for pin "));
          Serial.print((int)pin);
          Serial.print(F(": "));
          Serial.println(e);
          continue;
        }
        
        MultiPWM.apply();

        // measure speed while PWM is running
        uint32_t speed = getSpeed(f);

        float load = 100.0f - (1.0f * speed / rawSpeed * 100.0f);
        Serial.print(F("Channels: "));
        Serial.print(MultiPWM.size());
        Serial.print(F(" ("));
        Serial.print(speed);
        Serial.print(F(") -> CPU load: "));
        Serial.print(load);
        Serial.println(F(" %"));

        #ifdef MULTIPWM_MEASURE_OVERHEAD
        float isrLoad = 100.0f * __MultiPWM::maxOverhead / (MultiPWM.maximum() + 1.0f);
        Serial.print(F("Interrupt overhead: "));
        Serial.print(__MultiPWM::maxOverhead);
        Serial.print(F(" tick(s), ISR load: "));
        Serial.print(isrLoad);
        Serial.println(F(" %"));
        #endif
        if (load >= 75)
          break;
      }

      MultiPWM.reset();
    }    
  }
}
