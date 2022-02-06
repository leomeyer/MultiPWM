#ifndef __MULTIPWM_H
#define __MULTIPWM_H

// MultiPWM: A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// Copyright (c) Leo Meyer, leo@leomeyer.de
// https://github.com/leomeyer/MultiPWM
// 
// v1.0 2022/02/06: Unoptimized but functional version, tested on Uno, Mega and Duemilanove
//
// BSD 3-Clause License
// 
// MultiPWM - A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
// Copyright (c) 2022, Leo Meyer <leo@leomeyer.de>
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// function result codes
#define MULTIPWM_ERROR              -1
#define MULTIPWM_TOO_MANY_CHANNELS  -2
#define MULTIPWM_INVALID_PIN        -3
#define MULTIPWM_INVALID_PORT       -4
#define MULTIPWM_INVALID_PRESCALER  -5
#define MULTIPWM_INVALID_RESOLUTION -6

#ifdef __AVR__
#include <avr/interrupt.h>
#include <util/atomic.h>
#else
#error The MultiPWM library currently only works on an AVR architecture!
#endif

#define MULTIPWM_MAX_PIN  (NUM_DIGITAL_PINS - 1)

#if defined (__AVR_ATmega8__)
  #define TIMSK1 TIMSK
#endif

// avoid setting the overflow counter to 0 causing an interrupt
#define SET_COUNTER(counter)    TCNT1 = (counter);
#define GET_COUNTER             (TCNT1)

#ifndef MULTIPWM_MAXCHANNELS
#define MULTIPWM_MAXCHANNELS    4
#endif

#if (MULTIPWM_MAXCHANNELS < 1) || (MULTIPWM_MAXCHANNELS > NUM_DIGITAL_PINS)
#error To use MultiPWM MULTIPWM_MAXCHANNELS must be at least 1 and at most the number of pins on the board!
#endif

#if (MULTIPWM_MAXCHANNELS > 127)
#define MULTIPWM_MAXCHANNELS    127
#endif

enum class Prescaler : uint8_t {
  PRE_INVALID = 0,
  PRE_1 = 1,
  PRE_8 = 2,
  PRE_64 = 3,
  PRE_256 = 4,
  PRE_1024 = 5,
};

inline Prescaler operator++(Prescaler& p, int) {
  uint8_t v = (uint8_t)p;
  if (v >= (uint8_t)Prescaler::PRE_1024) {
    p = Prescaler(0);
  } else {
    p = Prescaler(v + 1);
  }
  return p;
}

inline bool operator==(Prescaler p, Prescaler other) {
  return (uint8_t)p == (uint8_t)other;
}

inline bool operator<(Prescaler p, Prescaler other) {
  if ((uint8_t)p == 0)
    return false;
  if (p == Prescaler::PRE_1024)
    return other != Prescaler::PRE_1024;
  return (uint8_t)p < (uint8_t)other;
}

inline bool operator>(Prescaler p, Prescaler other) {
  if ((uint8_t)p == 0)
    return false;
  if (p == Prescaler::PRE_1)
    return other != Prescaler::PRE_1;
  return (uint8_t)p >= (uint8_t)other;
}

bool operator<=(Prescaler p, Prescaler other) {
  if ((uint8_t)p == 0)
    return false;
  if (p == Prescaler::PRE_1024)
    return other == Prescaler::PRE_1024;
  return (uint8_t)p <= (uint8_t)other;
}

inline bool operator>=(Prescaler p, Prescaler other) {
  if ((uint8_t)p == 0)
    return false;
  if (p == Prescaler::PRE_1)
    return other == Prescaler::PRE_1;
  return (uint8_t)p >= (uint8_t)other;
}

inline int16_t operator&(Prescaler& p) {
  switch (p) {
    case Prescaler::PRE_1: return 1;
    case Prescaler::PRE_8: return 8;
    case Prescaler::PRE_64: return 64;
    case Prescaler::PRE_256: return 256;
    case Prescaler::PRE_1024: return 1024;
    default: return MULTIPWM_INVALID_PRESCALER;
  }
}

namespace __MultiPWM {

static uint8_t DEFAULT_GAMMA[256] = {
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
 90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
 115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
 144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
 177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
 215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

// indexed port registers
static constexpr volatile uint8_t *ports[16] = {NOT_A_PORT,
  #ifdef PORTA
  &PORTA,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTB
  &PORTB,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTC
  &PORTC,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTD
  &PORTD,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTE
  &PORTE,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTF
  &PORTF,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTG
  &PORTG,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTH
  &PORTH,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTI
  &PORTI,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTJ
  &PORTJ,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTK
  &PORTK,
  #else
  NOT_A_PORT,
  #endif
  #ifdef PORTL
  &PORTL,
  #else
  NOT_A_PORT,
  #endif
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT
};

// instruction buffer length: channel count byte + initial channel states + next timer values (2 byte) and next pindef (or 0)
#define MULTIPWM_INSTRBUFLEN (1 + (MULTIPWM_MAXCHANNELS) + (MULTIPWM_MAXCHANNELS + 1) * 3)

// control bitmask definitions
static const uint8_t CONTROL_SWITCH = 1;
static const uint8_t CONTROL_OFF = 2;

static Prescaler pre;
static uint8_t res = 0;                           // resolution
static uint16_t maxValue = 0;                     // maximum channel value according to resolution
static volatile uint16_t minCounter = 0;          // minimum counter value according to resolution
static uint8_t pindefs[MULTIPWM_MAXCHANNELS];
static uint16_t values[MULTIPWM_MAXCHANNELS];
static uint8_t instr1[MULTIPWM_INSTRBUFLEN];      // instruction buffer 1
static uint8_t instr2[MULTIPWM_INSTRBUFLEN];      // instruction buffer 2
static uint8_t channels = 0;                      // number of active channels
static volatile uint8_t control = CONTROL_OFF;    // interrupt routine control byte
static volatile uint8_t *active = NULL;           // pointer to active instruction buffer
static volatile uint8_t ic = 0;                   // current instruction counter
static const uint8_t *gammaTable = NULL;          // optional gamma correction table

#ifdef MULTIPWM_MEASURE_OVERHEAD
static volatile uint16_t overhead = 0;
static volatile uint16_t maxOverhead = 0;
#endif

inline void pinOn(uint8_t pinDef) {
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Turning on bit "));
  (MULTIPWM_DEBUG_SERIAL).print((int)(pinDef & 0b00000111));
  (MULTIPWM_DEBUG_SERIAL).print(F(" on port "));
  (MULTIPWM_DEBUG_SERIAL).println((int)((pinDef >> 3) & 0b00001111));
  #endif

  *(ports[(pinDef >> 3) & 0b00001111]) |= (1 << (pinDef & 0b00000111));
}

inline void pinOff(uint8_t pinDef) {
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Turning off bit "));
  (MULTIPWM_DEBUG_SERIAL).print((int)(pinDef & 0b00000111));
  (MULTIPWM_DEBUG_SERIAL).print(F(" on port "));
  (MULTIPWM_DEBUG_SERIAL).println((int)((pinDef >> 3) & 0b00001111));
  #endif

  *(ports[(pinDef >> 3) & 0b00001111]) &= ~(1 << (pinDef & 0b00000111));
}

/////////////////////////////////
// MultiPWM management class
/////////////////////////////////
class MultiPWM {
private:

  uint16_t getGammaValue(uint16_t val);
  
public:

  static Prescaler P(uint16_t p) {
    switch (p) {
      case 1 : return P_1;
      case 8 : return P_8;
      case 64: return P_64;
      case 256: return P_256;
      case 1024: return P_1024;
      default: return Prescaler::PRE_INVALID;
    }
  };
  
  static const Prescaler P_1 = Prescaler::PRE_1;
  static const Prescaler P_8 = Prescaler::PRE_8;
  static const Prescaler P_64 = Prescaler::PRE_64;
  static const Prescaler P_256 = Prescaler::PRE_256;
  static const Prescaler P_1024 = Prescaler::PRE_1024;
  
  static const uint8_t R_8BITS = 8;
  static const uint8_t R_9BITS = 9;
  static const uint8_t R_10BITS = 10;
  static const uint8_t R_11BITS = 11;
  static const uint8_t R_12BITS = 12;
  static const uint8_t R_13BITS = 13;
  static const uint8_t R_14BITS = 14;
  static const uint8_t R_15BITS = 15;
  static const uint8_t R_16BITS = 16;

  static constexpr const uint8_t *const DefaultGamma = DEFAULT_GAMMA;

  /* Initialize MultiPWM with the given resolution and prescaler.
   * The prescaler value may be one of the constants P_1, P_8, P_64, P_256 or P_1024.
   * Returns the approximate PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  long begin(uint8_t resolution, Prescaler prescaler);

  /* Initialize MultiPWM with the given resolution and prescaler.
   * The prescaler value may be 1, 8, 64, 256, or 1024.
   * Returns the approximate PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  inline long begin(uint8_t resolution, uint16_t prescaler) {
    return begin(resolution, P(prescaler));
  }

  /* Initialize MultiPWM with the given resolution and a prescaler of 64.
   * Returns the approximate PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  inline long begin(uint8_t resolution) {
    return begin(resolution, P_64);
  }

  /* Initialize MultiPWM with the standard resolution of 10 bits and a prescaler of 64.
   * Returns the approximate PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  inline long begin() {
    return begin(R_10BITS, P_64);
  }

  /* Set the gamma table to be used (an array of 256 uint_8 values).
   * NULL disables the gamma table.
   */
  inline void setGammaTable(const uint8_t *table) {
    gammaTable = table;
  }

  /* Calculate the accurate frequency for the specified combination of resolution and prescaler.
   * The prescaler value may be one of the constants P_1, P_8, P_64, P_256 or P_1024.
   * Returns the PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  float getFrequency(uint8_t resolution, Prescaler prescaler);

  /* Calculate the accurate frequency for the specified combination of resolution and prescaler.
   * The prescaler value may be 1, 8, 64, 256, or 1024.
   * Returns the PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  inline float getFrequency(uint8_t resolution, uint16_t prescaler) {
    return getFrequency(resolution, P(prescaler));
  }

  /* Get the current frequency of MultiPWM. Must be called after begin().
   * Returns the PWM period frequency in Hz or a value less than 0
   * in case of an error.
   */
  inline float getFrequency() {
    return getFrequency(res, pre);
  }

  /* Add a PWM channel for the specified pin. Optionally specify a value (default = 0) and
   * whether the channel should be inverted (default: false).
   * Returns the channel number or a value less than 0 in case of an error.
   */
  int8_t addChannel(uint8_t pin, uint16_t value = 0, bool inverted = false);
  
  /* Add a PWM channel for the specified pin. Specify a relative value and
   * whether the channel should be inverted (default: false).
   * Returns the channel number or a value less than 0 in case of an error.
   */
  inline int8_t addChannel(uint8_t pin, float value, bool inverted = false) {
    return addChannel(pin, (uint16_t)(value * maxValue), inverted);
  }
  
  /* Add an inverted PWM channel for the specified pin. Optionally specify a value (default = 0).
   * Returns the channel number or a value less than 0 in case of an error.
   */
  inline int8_t addInvertedChannel(uint8_t pin, uint16_t value = 0) {
    return addChannel(pin, value, true);
  }

  /* Add an inverted PWM channel for the specified pin. Optionally specify a relative value.
   * Returns the channel number or a value less than 0 in case of an error.
   */
  inline int8_t addInvertedChannel(uint8_t pin, float value) {
    return addInvertedChannel(pin, (uint16_t)(value * maxValue));
  }

  /* Set the integer PWM value for the channel. 
   * The value is limited to the integer range between 0 and maximum().
   */
  MultiPWM& set(uint8_t channel, uint16_t value);

  /* Set the relative PWM value for the channel. 
   * The value is a floating point number between 0 and 1.
   */
  inline MultiPWM& setf(uint8_t channel, float value) {
    return set(channel, (uint16_t)(value * maxValue));
  }

  /* Set the percent PWM value for the channel. 
   * The value is a floating point number between 0 and 100.
   */
  inline MultiPWM& setPercent(uint8_t channel, float percent) {
    return set(channel, (percent / 100.0f) * maxValue);
  }

  /* Return the current integer PWM value for the channel. 
   * Returns 0 if the specified channel is not valid.
   */
  inline uint16_t get(uint8_t channel) {
    return (channel < channels ? values[channel] : 0);
  }

  /* Return the current relative PWM value for the channel. 
   * Returns 0 if the specified channel is not valid.
   */
  inline float getf(uint8_t channel) {
    return 1.0f * (channel < channels ? values[channel] : 0) / (float)maxValue;
  }

  /* Apply the currently set channel values to the PWM output routine.
   * Returns the wait time (in counter ticks) until the first channel operation.
   */
  uint16_t apply();

  /* Apply the currently set channel values to the PWM output routine.
   * Returns the wait time (in counter ticks) until the first channel operation.
   */
  inline uint16_t start() { return apply(); }

  /* Return the value of the current prescaler constant.
   */
  inline Prescaler getPrescaler() { return pre; }

  /* Return the value of the current resolution.
   */
  inline uint8_t getResolution() { return res; }

  /* Return the maximum integer value according to the current resolution.
   */
  inline uint16_t maximum() { return maxValue; }

  /* Return the number of defined channels.
   */
  inline uint8_t size() { return channels; }

  /* Return true if MultiPWM has been initialized using begin(...) and channel values
   * have been applied.
   */
  inline bool isRunning() {  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return active != NULL;
    }
  }

  /* Stop the PWM. If the parameter is true, output pins keep their current state.
   * Otherwise all outputs are turned off.
   * Channel values are preserved. You can resume operation by calling apply().
   * This function enables global interrupts.
   */
  void stop(bool keep = false);

  /* Stop the PWM, reset resolution and prescaler and remove all channels. 
   * If the parameter is true, output pins keep their current state.
   * Otherwise all outputs are turned off.
   * To resume operation, MultiPWM must be initialized again using a begin(...) function.
   * This function enables global interrupts.
   */
  void reset(bool keep = false);
};

/////////////////////////////////
// Method implementations
/////////////////////////////////

uint16_t MultiPWM::getGammaValue(uint16_t val) {
  uint8_t r = (res - R_8BITS);
  if (val == 0)
    return gammaTable[0] << r;
  uint8_t p1 = val >> r;
  // exact value?
  if ((uint16_t)p1 << r == val)
    return gammaTable[p1] << r;
  else {
    // optimize frequent case: equal neighbored values
    if ((p1 < 255) && (gammaTable[p1] == gammaTable[p1 + 1]))
      return gammaTable[p1] << r;
    // interpolate between values
    int32_t yDiff = (int32_t)(p1 < 255 ? gammaTable[p1 + 1] << r : maxValue) * 256 - ((int32_t)gammaTable[p1] << r) * 256;
    int8_t point = val % (1 << r);
    int32_t v = (yDiff * point) >> r;
    return (((int32_t)gammaTable[p1] << r) * 256 + v) / 256;
  }
}

long MultiPWM::begin(uint8_t resolution, Prescaler prescaler) {
  if (isRunning())
    return MULTIPWM_ERROR;
  if ((resolution < R_8BITS) || (resolution > R_16BITS))
    return MULTIPWM_INVALID_RESOLUTION;

  int16_t p = &prescaler;
  if (p <= 0)
    return MULTIPWM_INVALID_PRESCALER;

  pre = prescaler;
  res = resolution;
  maxValue = (1UL << resolution) - 1;
  // calulate minimum counter value depending on resolution
  minCounter = 65535 - maxValue;
  if (minCounter == 0)
    minCounter = 1;
  uint64_t period = F_CPU / p / (maxValue + 1UL);
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: period: "));
  (MULTIPWM_DEBUG_SERIAL).print((long)period);
  (MULTIPWM_DEBUG_SERIAL).print(F(" Hz, maxValue: "));
  (MULTIPWM_DEBUG_SERIAL).print(maxValue);
  (MULTIPWM_DEBUG_SERIAL).print(F(", minCounter: "));
  (MULTIPWM_DEBUG_SERIAL).println(minCounter);
  #endif

  #ifndef MULTIPWM_DEBUG_SERIAL
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // initialize timer 1
    TCCR1A = 0;
    // set prescaler
    TCCR1B = ((uint8_t)prescaler & 0b00000111);
    // enable timer interrupt on TCNT1 overflow
    TIMSK1 |= (1 << TOIE1);
    TCNT1 = minCounter;
  }
  #endif
  
  return period;
}

float MultiPWM::getFrequency(uint8_t resolution, Prescaler prescaler) {
  float pre = &prescaler;
  if (pre <= 0)
    return MULTIPWM_INVALID_PRESCALER;
  return (1.0f * F_CPU) / pre / (1UL << resolution);
}

int8_t MultiPWM::addChannel(uint8_t pin, uint16_t value, bool inverted) {
  if (channels >= MULTIPWM_MAXCHANNELS) {
    #ifdef MULTIPWM_DEBUG_SERIAL
    (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Maximum number of channels exceeded: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)(MULTIPWM_MAXCHANNELS));
    #endif
  
    return MULTIPWM_TOO_MANY_CHANNELS;
  }
  if (pin > MULTIPWM_MAX_PIN) {
    #ifdef MULTIPWM_DEBUG_SERIAL
    (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Invalid pin: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)pin);
    #endif
  
    return MULTIPWM_INVALID_PIN;
  }
  uint8_t port = digitalPinToPort(pin);
  if ((port > sizeof(ports) / 2) || (ports[port] == NOT_A_PORT)) {
    #ifdef MULTIPWM_DEBUG_SERIAL
    (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Invalid port for pin: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)pin);
    #endif
  
    return MULTIPWM_INVALID_PORT;
  }
  
  uint8_t bitmask = digitalPinToBitMask(pin);
  int8_t bit = -1;
  while (bitmask > 0) {
    bit++;
    bitmask >>= 1;
  }
  pinMode(pin, OUTPUT);
  pindefs[channels] = (inverted ? 0b10000000 : 0) | (port << 3) | bit;
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: pin "));
  (MULTIPWM_DEBUG_SERIAL).print((int)pin);
  (MULTIPWM_DEBUG_SERIAL).print(F(" = port "));
  (MULTIPWM_DEBUG_SERIAL).print((int)port);
  (MULTIPWM_DEBUG_SERIAL).print(F(", bit "));
  (MULTIPWM_DEBUG_SERIAL).print((int)bit);
  (MULTIPWM_DEBUG_SERIAL).print(F(", pindef "));
  (MULTIPWM_DEBUG_SERIAL).println((int)pindefs[channels]);
  #endif
  
  values[channels] = value & maxValue;
  return channels++;
}

MultiPWM& MultiPWM::set(uint8_t channel, uint16_t value) {
  if (channel > channels) {
    #ifdef MULTIPWM_DEBUG_SERIAL
    (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Trying to set value for invalid channel: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)channel);
    #endif
  
    return *this;
  }
  values[channel] = value & maxValue;
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: Value set for channel "));
  (MULTIPWM_DEBUG_SERIAL).print((int)channel);
  (MULTIPWM_DEBUG_SERIAL).print(F(" to "));
  (MULTIPWM_DEBUG_SERIAL).println(values[channel]);
  #endif
  
  return *this;
}

uint16_t MultiPWM::apply() {
  // not initalized?
  if ((res == 0) || (pre == Prescaler::PRE_INVALID))
    return minCounter;
  // no channels?
  if (channels == 0)
    return minCounter;

  // clear switch bit in case it is still set (called too frequently)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    control &= ~CONTROL_SWITCH;
  }

  // prepare values along with pindefs
  uint16_t sortedValues[channels];
  uint8_t sortedDefs[channels];
  uint8_t usedChannels = 0;
  for (uint8_t c = 0; c < channels; c++) {
    uint16_t val = values[c];
    // use gamma table?
    if ((gammaTable != NULL) && (val < maxValue)) {
      if (res == R_8BITS)
        val = gammaTable[val];
      else {
        val = getGammaValue(val);
      }
    }
    // skip zero and maximum channels
    if ((val == 0) || (val == maxValue))
      continue;
    sortedValues[usedChannels] = maxValue - val;
    sortedDefs[usedChannels] = pindefs[c];
    usedChannels++;
  }
  // insertion sort
  for (uint8_t i = 1; i < usedChannels; i++) {
    uint16_t v = sortedValues[i];
    uint8_t d = sortedDefs[i];
    uint8_t j = i;
    for (; j > 0 && sortedValues[j-1] > v; j--) {
      sortedValues[j] = sortedValues[j-1];
      sortedDefs[j] = sortedDefs[j-1];
    }
    sortedValues[j] = v;
    sortedDefs[j] = d;
  }
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).println(F("MultiPWM sorted values: "));
  for (uint8_t x = 0; x < usedChannels; x++) {
   (MULTIPWM_DEBUG_SERIAL).print((int)x);
   (MULTIPWM_DEBUG_SERIAL).print(F(": "));
   (MULTIPWM_DEBUG_SERIAL).print((int)sortedDefs[x]);
   (MULTIPWM_DEBUG_SERIAL).print(F(" = "));
   (MULTIPWM_DEBUG_SERIAL).println(sortedValues[x]);
  }
  #endif

  // build instructions for interrupt routine
  // unguarded access to active is safe because the switch bit has been cleared
  uint8_t *instr = (active == instr1 ? instr2 : instr1);
  uint8_t lic = 0;   // local instruction counter

  // the first byte is the number of channels in this instruction set
  instr[lic++] = channels;
  // output initial channel state instructions
  for (uint8_t c = 0; c < channels; c++) {
    // maximum/always on?
    if (values[c] == maxValue)
      // switch pin on
      instr[lic++] = pindefs[c] | 0b10000000;
    else {
      // inverted?
      if ((pindefs[c] & 0b10000000) == 0b10000000)
        // switch pin on
        instr[lic++] = pindefs[c];
      else
        // switch pin off
        instr[lic++] = pindefs[c] & 0b01111111;
      }
  }

  uint16_t step = 0;
  uint16_t *wait = (uint16_t *)&instr[lic];
  // determine wait time for the channels
  for (uint8_t c = 0; c < usedChannels; c++) {
    *wait = minCounter + (maxValue - (sortedValues[c] - step));
    lic += 2;
    step = sortedValues[c];
    // output switch command for this pin
    // inverted?
    if ((sortedDefs[c] & 0b10000000) == 0b10000000)
      // switch pin off
      instr[lic++] = sortedDefs[c] & 0b01111111;
    else
      // switch pin on
      instr[lic++] = sortedDefs[c] | 0b10000000;
    
    #ifdef MULTIPWM_DEBUG_SERIAL
    (MULTIPWM_DEBUG_SERIAL).println(F("MultiPWM counter for channel: "));
    (MULTIPWM_DEBUG_SERIAL).print((int)c);
    (MULTIPWM_DEBUG_SERIAL).print(F(": "));
    (MULTIPWM_DEBUG_SERIAL).print(*wait);
    (MULTIPWM_DEBUG_SERIAL).print(F(", step: "));
    (MULTIPWM_DEBUG_SERIAL).println(step);
    #endif
  
    wait = (uint16_t *)&instr[lic];
  }
  // time to wait until the end of the period (instruction rollover)
  *wait = 65535 - (maxValue - step);
  lic += 2;
  instr[lic] = 0;   // end of instructions
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).println(F("MultiPWM instructions: "));
  (MULTIPWM_DEBUG_SERIAL).print(F("Channels: "));
  (MULTIPWM_DEBUG_SERIAL).println((int)instr[0]);
  uint8_t x = 0;
  for (; x < instr[0]; x++) {
    (MULTIPWM_DEBUG_SERIAL).print(F("Channel "));
    (MULTIPWM_DEBUG_SERIAL).print((int)x);
    (MULTIPWM_DEBUG_SERIAL).print(F(" initial state: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)instr[x + 1]);
  }
  do {
     uint16_t *next = (uint16_t *)&instr[++x];
     x += 2;
    (MULTIPWM_DEBUG_SERIAL).print(F("Wait: "));
    (MULTIPWM_DEBUG_SERIAL).print(*next);
    (MULTIPWM_DEBUG_SERIAL).print(F(", pindef: "));
    (MULTIPWM_DEBUG_SERIAL).println((int)instr[x]);
  } while (instr[x] != 0);
  #endif
  
  // activate and switchover
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    control = CONTROL_SWITCH;
  }
  // return first wait time after channel instructions
  return *((uint16_t *)&instr[1 + channels]);
}

void MultiPWM::stop(bool keep) {
  if (!isRunning())
    return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    control = CONTROL_OFF;
  // trigger interrupt at next cycle
  SET_COUNTER(65535);
  }
  // enable interrupts
  sei();
  // wait until PWM has stopped or 2 ms have passed
  uint32_t stopTime = millis();
  while (isRunning() && (millis() - stopTime < 2));
  if (!isRunning() && !keep) {
    // turn outputs off
    for (uint8_t c = 0; c < channels; c++) {
    pinOff(pindefs[c] & 0b01111111);
    }
  }
}

void MultiPWM::reset(bool keep) {
  stop(keep);
  channels = 0;
  res = 0;
  pre = Prescaler::PRE_INVALID;
}

///////////////////////
// main PWM routine
///////////////////////
// needs to be compiled with -O3 for stability reasons
#pragma GCC push_options
#pragma GCC optimize ("O3")
inline uint16_t update(void) {
  // switch off?
  if ((control & CONTROL_OFF) == CONTROL_OFF) {
    active = NULL;
    // reset instruction counter
    ic = 0;
  }
start:
  // start of instructions?
  if (ic == 0) {
    #ifdef MULTIPWM_MEASURE_OVERHEAD
    if (maxOverhead < overhead)
      maxOverhead = overhead;
    overhead = 0;
    #endif
    
    // switchover?
    if ((control & CONTROL_SWITCH) == CONTROL_SWITCH) {
      #ifdef MULTIPWM_DEBUG_SERIAL
      (MULTIPWM_DEBUG_SERIAL).println(F("MultiPWM: Switching over"));
      #endif

      control &= ~CONTROL_SWITCH;
      // switch to other instruction set
      if (active == instr1)
        active = instr2;
      else
        active = instr1;
      #ifdef MULTIPWM_MEASURE_OVERHEAD
          maxOverhead = 0;
      #endif
    }

    if (active == NULL) {
      #ifdef MULTIPWM_DEBUG_SERIAL
      (MULTIPWM_DEBUG_SERIAL).println(F("MultiPWM: No active instructions"));
      return minCounter;
      #else
      #ifdef MULTIPWM_MEASURE_OVERHEAD
      overhead += TCNT1;
      #endif
      SET_COUNTER(minCounter)
      return 0;
      #endif
    }

    // set initial channel state at period start
    // the first byte is the number of channels in this instruction set
    uint8_t count = active[0];
    for (uint8_t c = 0; c < count; c++) {
      ic++;
      // pin on?
      if ((active[ic] & 0b10000000) == 0b10000000) {
        pinOn(active[ic] & 0b01111111);
      } else {
        pinOff(active[ic]);
      }
    }
    ic++;
    uint16_t next = *((uint16_t *)&active[ic]);
    ic += 2;
  
    #ifdef MULTIPWM_DEBUG_SERIAL
    return next;
    #else
    #ifdef MULTIPWM_MEASURE_OVERHEAD
    overhead += TCNT1;
    #endif
    SET_COUNTER(next)
    return 0;
    #endif
  }
  
  // get next instruction
  uint8_t i = active[ic];
  // end of instructions?
  if (i == 0) {
    ic = 0;
    // immediately start over
    goto start;
  }
  // switch pin
  #ifdef MULTIPWM_DEBUG_SERIAL
  (MULTIPWM_DEBUG_SERIAL).print(F("MultiPWM: ic = "));
  (MULTIPWM_DEBUG_SERIAL).print((int)ic);
  (MULTIPWM_DEBUG_SERIAL).print(F(", switching pindef: "));
  (MULTIPWM_DEBUG_SERIAL).println((int)i);
  #endif
  
  // inverted?
  if ((i & 0b10000000) == 0b10000000)
    pinOn(i);
  else
    pinOff(i & 0b01111111);
  ic++;
  uint16_t next = *((uint16_t *)&active[ic]);
  ic += 2;
  
  #ifdef MULTIPWM_DEBUG_SERIAL
  return next;
  #else
  #ifdef MULTIPWM_MEASURE_OVERHEAD
  overhead += TCNT1;
  #endif
  SET_COUNTER(next)
  return 0;
  #endif
}
#pragma GCC pop_options

}   // namespace __MultiPWM

#if defined(__AVR__)
ISR(TIMER1_OVF_vect)
{
  #ifndef MULTIPWM_DEBUG_SERIAL
  __MultiPWM::update();
  #endif
}
#endif

static __MultiPWM::MultiPWM MultiPWM;

#endif  // __MULTIPWM_H
