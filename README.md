# MultiPWM
A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins

# Library Information
  - Name :: MultiPWM
  - Version :: 1.0
  - License :: BSD
  - URL :: https://github.com/leomeyer/MultiPWM
  - Author :: Leo Meyer
  - Email :: leo@leomeyer.de

The MultiPWM library implements a software PWM for AVR Arduinos with up to 16 bits resolution on any combination of digital output pins
(e. g. Arduino Uno: up to 20 pins, Arduino Mega: up to 70 pins) that works with avr and megaavr processors.

It uses the 16-bit timer TIMER1 on AVR processors. It cannot be used with libraries or sketches that also use TIMER1.
The maximum PWM frequency depends on the CPU. On an Arduino UNO, for example, the maximum usable PWM frequency is 7812.5 Hz.
For LEDs, however, a PWM frequency of about 250 Hz is usually enough. Less than that will cause the LEDs to flicker, especially
when the LEDs or the observer move.

MultiPWM is quite CPU-efficient. Processor usage is dependent on resolution (lower resolutions will create more load) and the
number of output channels. At a PWM frequency of 244 Hz a typical RGB LED setup with three channels on an Arduino Uno will use 
about 1.2 % of CPU (10 bits). Using all 20 Uno pins at 244 Hz at 10 bits will consume about 7.5 % of CPU time. At 8 bits (a resolution provided 
for compatibility reasons) 20 pins on the Uno will consume about 7 % of CPU time (prescaler 256, 244 Hz).

MultiPWM supports nonlinearity compensation using 8-bit gamma tables with linear interpolation for higher resolutions.

## Installation

You can install the downloaded library archive using the Arduino IDE's menu item "Sketch -> Include Library... -> Add .ZIP library...".
To install it manually, unzip the downloaded library archive in the Arduino libraries folder. See here for the library folder location:
[https://support.arduino.cc/hc/en-us/articles/4411202655634-Find-Arduino-IDE-files-and-folders](https://support.arduino.cc/hc/en-us/articles/4411202655634-Find-Arduino-IDE-files-and-folders)

Manual installation requires an IDE restart. These methods will also allow you to open the provided MultiPWM examples from the Arduino IDE. 

Alternatively, you can simply add the `MultiPWM.h` file to a sketch folder. After opening the sketch with the IDE the `.h` file will
be opened in a separate tab and compile along with your sketch.

## Usage

For getting started quickly, see the BareMinimum example sketch:

```
#include "MultiPWM.h"

void setup() {
  if (MultiPWM.begin() >= 0) {
    // set builtin LED brightness to about 1%
    MultiPWM.addChannel(LED_BUILTIN, MultiPWM.maximum() / 100);
    MultiPWM.apply();
  }
}

void loop() {}
```

1. Optionally, set the maximum number of pins that you want to have controlled by this library using e. g.

```
#define MULTIPWM_MAXCHANNELS    8
```

The default number of channels is 4. More channels use more RAM for the internal control structures. You should set this number as low as possible
if you need to keep memory usage low.

Include the library using:

```
#include "MultiPWM.h"
```

2. Initialize the MultiPWM object with an optional resolution (`MultiPWM.R_8BITS` to `MultiPWM.R_16BITS`, default: `MultiPWM.R_10BITS`)
and an optional prescaler value (`MultiPWM.P_8`, `MultiPWM.P_64`, `MultiPWM.P_256` or `MultiPWM.P_1024`, default: `MultiPWM.P_64`).

```
long frequency = MultiPWM.begin();
```

The `begin()` function returns the approximate frequency of the PWM period. For large resolution and/or prescaler values the result
may be 0 indicating less than 1 Hz. For example, on a CPU with 16 MHz clock P_1024 and R_16BITS will yield a PWM period of about 0.25 (4 s).
The frequency of the default values (10 bits, prescaler 64) on a 16 MHz CPU is about 244 Hz which is a good value for LEDs.
You can use the function `getFrequency()` to calculate the exact frequency (this will increase code size due to floating point
calculations, though):

```
float freq = MultiPWM.getFrequency();
```

This method returns the currently initialized frequency. To calculate a combination of resolutions and prescalers, use

```
float freq = MultiPWM.getFrequency(MultiPWM.R_16BITS, MultiPWM.P_1024);
```

If the result is negative it indicates an error (see "function result codes" below).

Higher PWM frequencies (i. e. lower resolutions and/or lower prescalers) will result in more CPU load. See "Accuracy" below.

3. Add an arbitrary number of channels (up to the value of NUM_DIGITAL_PINS defined by the Arduino board library) specifying the number
of the Arduino pin to use for this channel:

```
int8_t led = MultiPWM.addChannel(LED_BUILTIN);
```

Optionally, specify a channel value (default = 0):

```
int8_t led = MultiPWM.addChannel(LED_BUILTIN, 100);
```

The `addChannel()` functions returns the number of the channel (0 to 127 maximum). If a negative number is returned it indicates an error.
Channels can be added at any time during operation, however, they cannot be removed individually.
The `addChannel()` function configures the specified pin as a digital output (if it is valid).
Normal channel operation is being switched off at the start of each period (unless the value is at maximum). To invert the behavior
you can use the `addInvertedChannel()` method, e. g.:

```
int8_t led = MultiPWM.addInvertedChannel(LED_BUILTIN);
```

or

```
int8_t led = MultiPWM.addInvertedChannel(LED_BUILTIN, 100);
```

or

```
int8_t led = MultiPWM.addChannel(LED_BUILTIN, 100, true);
```

There are also float versions of the `addChannel()` and `addInvertedChannel()` methods that allow specifying a relative value between 0 and 1.

4. Use the channel number returned by `addChannel()` to set the PWM value for this pin:

```
MultiPWM.set(led, 2047);
```

You can use `setf()` to specify a float value between 0 and 1:

```
MultiPWM.setf(led, 0.5f);
```

or `setPercent()` to set a percentage between 0 and 100:

```
MultiPWM.setPercent(led, 45.5);
```

The values set like this do not become active immediately. Changes must be activated using the `apply()` function:

```
MultiPWM.apply();
```

Some method calls can be chained, e. g.:

```
MultiPWM.set(0, 100).set(1, 200).set(2, 300).apply();
```

5. To stop the PWM, use

```
MultiPWM.stop();
```

This will set all outputs to 0 (off). If you want to keep the outputs in their current state, use

```
MultiPWM.stop(true);
```

The current channel values are preserved. To restart the PWM, call `apply()` again.

6. Obtaining PWM information

Resolution and prescaler values can be obtained from `getResolution()` and `getPrescaler()`, respectively.
The current number of channels can be queried using `size()`.
Whether the PWM is operating can be queried using `isRunning()`.
The maximum value for the specified resolution is returned by `maximum()`.

7. To stop the PWM and remove all channels use

```
MultiPWM.reset();
```

or

```
MultiPWM.reset(true);
```

to keep the outputs in their current state.
To continue after a reset you need to initialize the PWM using `begin()` again with the same or different parameters.

8. MultiPWM supports gamma correction tables to compensate for LED brightness nonlinearity. A gamma correction
table is an array of 256 uint8_t values:

```
uint8_t gammaTable[256] = {v1, v2, ...};
```

You can set a gamma table using the `setGammaTable()` function:

```
MultiPWM.setGammaTable(gammaTable);
```

You can also use the gamma table provided by MultiPWM using:

```
MultiPWM.setGammaTable(MultiPWM.DefaultGamma);
```

For higher resolutions than 8 bits the gamma values are linearly interpolated.

## Error codes

The following error codes are defined:

```
#define MULTIPWM_ERROR              -1
#define MULTIPWM_TOO_MANY_CHANNELS  -2
#define MULTIPWM_INVALID_PIN        -3
#define MULTIPWM_INVALID_PORT       -4
#define MULTIPWM_INVALID_PRESCALER  -5
#define MULTIPWM_INVALID_RESOLUTION -6
```

## Accuracy

MultiPWM, being a software PWM library, is not as accurate as hardware PWM. It uses TIMER1 which has a lower priority than
TIMER0. Frequent interrupts or long service routines will increase the inaccuracy of MultiPWM, as will frequent
calls to `delay()`, `millis()`, `micros()` or other functions that disable interrupts.

Two channels sharing the same PWM value will not be switched simultaneously. The second channel will be served when the
next TIMER1 interrupt occurs. Other interrupts may delay execution in between.

All channels are switched to their initial state at the start of a PWM period. During the subsequent steps the channels
are switched as necessary. Changing channel values and calling `apply()` will not be effective immediately. Changes will
be made at the start of the next PWM period. Consequently, the channels will remain in-phase (within the limits of 
general accuracy).

At a prescaler of 8 and resolutions below 10 bits, if too many channels are used the interrupt service routine may take 
more time than what fits into the PWM period (Uno: 9 channels at 8 bits or 7812.50 Hz, 18 channels at 9 bits or 3906.25 Hz).
Performance measurement shows this as an ISR load greater than 100 %. The library can still be used with these setups
but the PWM frequency will suffer. If you require the PWM frequency to remain approximately the same it is recommended
not to use that many channels.

There is still room for optimization but as the primary use case of this library is for LEDs where frequencies are typically
much lower it is usable as of now.	

## Reporting bugs

Please report bugs at the repository issues page: https://github.com/leomeyer/MultiPWM/issues

## License

BSD 3-Clause License

MultiPWM - A CPU-efficient AVR Arduino software PWM library for resolutions of 8 to 16 bits on an arbitrary number of digital output pins
Copyright (c) 2022, Leo Meyer <leo@leomeyer.de>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

