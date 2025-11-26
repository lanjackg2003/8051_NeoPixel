## 🆕 8051/C51 Platform Support

**This repository is a modified version of [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel)**, adding support for **8051 microcontrollers** (including CA51F005 and other C51-compatible chips) using **Keil C51 compiler**.

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

---

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
