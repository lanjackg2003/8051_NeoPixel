# Adafruit NeoPixel Library [![Build Status](https://github.com/adafruit/Adafruit_NeoPixel/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_NeoPixel/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_NeoPixel/html/index.html)

Arduino library for controlling single-wire-based LED pixels and strip such as the [Adafruit 60 LED/meter Digital LED strip][strip], the [Adafruit FLORA RGB Smart Pixel][flora], the [Adafruit Breadboard-friendly RGB Smart Pixel][pixel], the [Adafruit NeoPixel Stick][stick], and the [Adafruit NeoPixel Shield][shield].

After downloading, rename folder to 'Adafruit_NeoPixel' and install in Arduino Libraries folder. Restart Arduino IDE, then open File->Sketchbook->Library->Adafruit_NeoPixel->strandtest sketch.

Compatibility notes: Port A is not supported on any AVR processors at this time

---

## 🆕 8051/C51 Platform Support

**This repository is a modified version of Adafruit NeoPixel Library**, adding support for **8051 microcontrollers** (including CA51F005 and other C51-compatible chips) using **Keil C51 compiler**.

### What's New

This fork extends the original Adafruit NeoPixel library with:

- ✅ **Pure C API** - Complete C port of the NeoPixel library, compatible with non-C++ environments
- ✅ **8051/C51 Support** - Optimized for Keil C51 compiler compatibility
- ✅ **Custom Driver Interface** - Flexible driver registration system for platform-specific hardware implementations
- ✅ **Memory Efficient** - Suitable for resource-constrained microcontrollers

### Key Features for 8051/C51 Platforms

- Optimized for Keil C51 compiler compatibility
- Support for custom hardware drivers via driver registration interface
- Memory-efficient implementation suitable for resource-constrained MCUs
- PWM-based and bit-banging driver examples provided

[flora]:  http://adafruit.com/products/1060
[strip]:  http://adafruit.com/products/1138
[pixel]:  http://adafruit.com/products/1312
[stick]:  http://adafruit.com/products/1426
[shield]: http://adafruit.com/products/1430

---

## Installation

### First Method

![image](https://user-images.githubusercontent.com/36513474/68967967-3e37f480-0803-11ea-91d9-601848c306ee.png)

1. In the Arduino IDE, navigate to Sketch > Include Library > Manage Libraries
1. Then the Library Manager will open and you will find a list of libraries that are already installed or ready for installation.
1. Then search for Neopixel strip using the search bar.
1. Click on the text area and then select the specific version and install it.

### Second Method

1. Navigate to the [Releases page](https://github.com/adafruit/Adafruit_NeoPixel/releases).
1. Download the latest release.
1. Extract the zip file
1. In the Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library

## Features

- ### Simple to use

  Controlling NeoPixels “from scratch” is quite a challenge, so we provide a library letting you focus on the fun and interesting bits.

- ### Give back

  The library is free; you don’t have to pay for anything. Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

- ### Supported Chipsets

  We have included code for the following chips - sometimes these break for exciting reasons that we can't control in which case please open an issue!

  - AVR ATmega and ATtiny (any 8-bit) - 8 MHz, 12 MHz and 16 MHz
  - Teensy 3.x and LC
  - Arduino Due
  - Arduino 101
  - Arm® Cortex®-M7/M4 - RENESAS/STM (Arduino UNO R4, Arduino Portenta H7, Arduino Giga R1)
  - ATSAMD21 (Arduino Zero/M0 and other SAMD21 boards) @ 48 MHz
  - ATSAMD51 @ 120 MHz
  - Adafruit STM32 Feather @ 120 MHz
  - ESP8266 any speed
  - ESP32 any speed
  - Nordic nRF52 (Adafruit Feather nRF52), nRF51 (micro:bit)
  - Infineon XMC1100 BootKit @ 32 MHz
  - Infineon XMC1100 2Go @ 32 MHz
  - Infineon XMC1300 BootKit  @ 32 MHz
  - Infineon XMC4700 RelaxKit, XMC4800 RelaxKit, XMC4800 IoT Amazon FreeRTOS Kit @ 144 MHz
  - Sipeed Maix Bit (K210 processor)
  - **8051/C51 microcontrollers** (including CA51F005 and other C51-compatible chips) - *Added in this fork*

  Check forks for other architectures not listed here!

- ### GNU Lesser General Public License

  Adafruit_NeoPixel is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

## Functions

- begin()
- updateLength()
- updateType()
- show()
- delay_ns()
- setPin()
- setPixelColor()
- fill()
- ColorHSV()
- getPixelColor()
- setBrightness()
- getBrightness()
- clear()
- gamma32()

## Examples

There are many examples implemented in this library. One of the examples is below. You can find other examples [here](https://github.com/adafruit/Adafruit_NeoPixel/tree/master/examples)

### Simple (Arduino/C++)

```Cpp
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN        6
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

void setup() {
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pixels.begin();
}

void loop() {
  pixels.clear();

  for(int i=0; i<NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    pixels.show();
    delay(DELAYVAL);
  }
}
```

---

## 📚 8051/C51 Usage Guide

### Installation for 8051/C51 (Keil C51)

1. Add `NeoPixel.h` and `NeoPixel.c` to your Keil C51 project
2. Include the header file: `#include "NeoPixel.h"`
3. Implement a custom driver function for your hardware platform
4. Register your driver using `neopixel_register_driver()`

### Pure C API Functions

This fork provides a complete Pure C API with the following functions:

#### Core Functions
- `neopixel_init()` - Initialize a NeoPixel strip object
- `neopixel_begin()` - Configure NeoPixel pin for output
- `neopixel_show()` - Transmit pixel data to NeoPixels
- `neopixel_clear()` - Fill the whole strip with black/off
- `neopixel_deinit()` - Deallocate NeoPixel object

#### Color Functions
- `neopixel_setPixelColor()` - Set a pixel's color using RGB components
- `neopixel_setPixelColorRGBW()` - Set a pixel's color using RGBW components
- `neopixel_setPixelColor32()` - Set a pixel's color using a 32-bit packed value
- `neopixel_getPixelColor()` - Query the color of a previously-set pixel
- `neopixel_fill()` - Fill all or part of the strip with a color
- `neopixel_rainbow()` - Fill strip with rainbow colors

#### Utility Functions
- `neopixel_setBrightness()` / `neopixel_getBrightness()` - Brightness control
- `neopixel_updateLength()` - Change the length of a strip
- `neopixel_updateType()` - Change the pixel format
- `neopixel_setPin()` / `neopixel_getPin()` - Pin management
- `neopixel_numPixels()` - Get number of pixels
- `neopixel_getPixels()` - Get pointer to pixel buffer
- `neopixel_canShow()` - Check if show() can be called

#### Color Conversion Functions
- `neopixel_Color()` - Convert RGB to packed 32-bit color
- `neopixel_ColorRGBW()` - Convert RGBW to packed 32-bit color
- `neopixel_ColorHSV()` - Convert HSV to packed RGB color
- `neopixel_gamma8()` / `neopixel_gamma32()` - Gamma correction
- `neopixel_sine8()` - 8-bit sine wave function
- `neopixel_str2order()` - Convert string to color order constant

#### Custom Driver Registration (8051/C51 Specific)
- `neopixel_register_driver()` - Register a custom hardware driver function

### Custom Driver Interface

For 8051/C51 platforms, you need to implement a custom driver function that handles the low-level timing requirements of WS2812B/NeoPixel communication.

#### Driver Function Signature

```c
typedef void (*neopixel_driver_func_t)(neopixel_driver_params_t *params);
```

#### Driver Parameters Structure

```c
typedef struct {
  int16_t pin;        // GPIO pin number used for data output
  uint8_t *pixels;    // Pointer to pixel data buffer (RGB or RGBW format)
  uint16_t numBytes;  // Number of bytes in the pixel buffer
  bool is800KHz;      // true for 800 KHz (WS2812B), false for 400 KHz (WS2811)
  void *userData;     // Optional user data pointer (can be NULL)
} neopixel_driver_params_t;
```

#### Example: 8051/C51 Usage

```c
#include "NeoPixel.h"

#define PIN        6
#define NUMPIXELS 16

NeoPixel strip;

// Custom driver function for your 8051 hardware platform
// This function must implement the precise timing required by WS2812B
void my_ws2812b_driver(neopixel_driver_params_t *params) {
	// TODO: Implement PWM-based driver
	// Example steps:
	// 1. Configure PWM channel for params->pin
	// 2. Set PWM frequency based on params->is800KHz
	// 3. Use DMA or buffer to send params->pixels data
	// 4. Generate reset pulse
	// Access parameters: params->pin, params->pixels, params->numBytes,
	//                    params->is800KHz, params->userData
}

void main() {
    // Initialize NeoPixel strip
    neopixel_init(&strip, NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

    // Register custom driver (required for 8051/C51 platforms)
    neopixel_register_driver(&strip, my_ws2812b_driver, NULL);

    // Begin the strip
    neopixel_begin(&strip);

    // Set pixel colors
    for (uint16_t i = 0; i < NUMPIXELS; i++) {
        neopixel_setPixelColor(&strip, i, 0, 150, 0);  // Green
    }

    // Show the pixels (this will call your custom driver function)
    neopixel_show(&strip);

    // Example: Create rainbow effect
    neopixel_rainbow(&strip, 0, 1, 255, 255, true);
    neopixel_show(&strip);
}
```

## Contributing

If you want to contribute to this project:

- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell others about this library
- Contribute new protocols

Please read [CONTRIBUTING.md](https://github.com/adafruit/Adafruit_NeoPixel/blob/master/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

### Roadmap

The PRIME DIRECTIVE is to maintain backward compatibility with existing Arduino sketches -- many are hosted elsewhere and don't track changes here, some are in print and can never be changed!

Please don't reformat code for the sake of reformatting code. The resulting large "visual diff" makes it impossible to untangle actual bug fixes from merely rearranged lines. Also, don't bother with PRs for timing adjustments "to better match the datasheet," because the datasheet isn't really true to begin with.

Things I'd Like To Do But There's No Official Timeline So Please Don't Count On Any Of This Ever Being Canonical:

- 400 KHz support can be removed, turns out it was never actually necessary; even the earliest NeoPixels can ingest 800 KHz data. Of course the #defines should remain so old sketches still compile, but both can be set to 0 and would have no effect on anything.
- For the show() function (with all the delicate pixel timing stuff), break out each architecture into separate source files rather than the current unmaintainable tangle of #ifdef statements!
- Please don't use updateLength() or updateType() in new code. They should not have been implemented this way (use the C++ 'new' operator with the regular constructor instead) and are only sticking around because of the Prime Directive. setPin() is OK for now though, it's a trick we can use to 'recycle' pixel memory across multiple strips.
- In the M0 and M4 code, use the hardware systick counter for bit timing rather than hand-tweaked NOPs (a temporary kludge at the time because I wasn't reading systick correctly). (As of 1.4.2, systick is used on M4 devices and it appears to be overclock-compatible. Not for M0 yet, which is why this item is still here.)
- As currently written, brightness scaling is still a "destructive" operation -- pixel values are altered in RAM and the original value as set can't be accurately read back, only approximated, which has been confusing and frustrating to users. It was done this way at the time because NeoPixel timing is strict, AVR microcontrollers (all we had at the time) are limited, and assembly language is hard. All the 32-bit architectures should have no problem handling nondestructive brightness scaling -- calculating each byte immediately before it's sent out the wire, maintaining the original set value in RAM -- the work just hasn't been done. There's a fair chance even the AVR code could manage it with some intense focus. (The DotStar library achieves nondestructive brightness scaling because it doesn't have to manage data timing so carefully...every architecture, even ATtiny, just takes whatever cycles it needs for the multiply/shift operations.)

## Credits

This library is written by Phil "Paint Your Dragon" Burgess for Adafruit Industries, with contributions by PJRC, Michael Miller and other members of the open source community.

**8051/C51 port and enhancements by JackLan <lanjackg2003@gmail.com>**

This fork adds Pure C API and 8051/C51 platform support to the original Adafruit NeoPixel library.

## License

Adafruit_NeoPixel is free software: you can redistribute it and/or  modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Adafruit_NeoPixel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [GNU Lesser General Public License](https://www.gnu.org/licenses/lgpl-3.0.en.html) for more details.
You should have received a copy of the GNU Lesser General Public License along with NeoPixel.  If not, see [this](https://www.gnu.org/licenses/)
