/*!
 * @file NeoPixel.h
 *
 * Pure C implementation of Adafruit NeoPixel library
 * This is a C port of the C++ NeoPixel library for the Arduino platform,
 * allowing a broad range of microcontroller boards (most AVR boards,
 * many ARM devices, ESP8266 and ESP32, 8051 microcontrollers (including
 * CA51F005 and other C51-compatible chips), among others) to control
 * Adafruit NeoPixels, FLORA RGB Smart Pixels and compatible devices --
 * WS2811, WS2812, WS2812B, SK6812, etc.
 *
 * Special features for 8051/C51 platforms:
 * - Optimized for Keil C51 compiler compatibility
 * - Support for custom hardware drivers via driver registration interface
 * - Memory-efficient implementation suitable for resource-constrained MCUs
 * - PWM-based and bit-banging driver examples provided
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * Written by Phil "Paint Your Dragon" Burgess for Adafruit Industries,
 * with contributions by PJRC, Michael Miller and other members of the
 * open source community.
 *
 * 8051/C51 port and enhancements by JackLan <lanjackg2003@gmail.com>
 *
 * This file is part of the NeoPixel library.
 *
 * NeoPixel is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * NeoPixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with NeoPixel.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

// Keil C51 compatibility
#ifdef __C51__
#define C51_COMPILER
// Note: bit type cannot be used in structs, use unsigned char instead
// defines.h defines bool as _Bool, but we use unsigned char for compatibility
 #ifndef USE_ARIES_SDK
    #include <stdint.h>
#else
    #include "aries_sdk.h"
#endif
#include <stdlib.h>
#ifndef true
#define true  1
#endif
#ifndef false
#define false 0
#endif
// C51 memory model keywords
#define PROGMEM code
extern void *heap_alloc(size_t size);
extern void heap_free(void *ptr);
#else
// Standard C compilers
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#ifndef PROGMEM
#define PROGMEM
#endif
#endif

#ifdef ARDUINO
#include <Arduino.h>

#ifdef USE_TINYUSB // For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

#endif

#ifdef TARGET_LPC1768
#include <Arduino.h>
#endif

#if defined(ARDUINO_ARCH_RP2040)
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "rp2040_pio.h"
#endif

// The order of primary colors in the NeoPixel data stream can vary among
// device types, manufacturers and even different revisions of the same
// item.  The third parameter to the NeoPixel constructor encodes
// the per-pixel byte offsets of the red, green and blue primaries (plus
// white, if present) in the data stream -- the following #defines provide
// an easier-to-use named version for each permutation. e.g. NEO_GRB
// indicates a NeoPixel-compatible device expecting three bytes per pixel,
// with the first byte transmitted containing the green value, second
// containing red and third containing blue. The in-memory representation
// of a chain of NeoPixels is the same as the data-stream order; no
// re-ordering of bytes is required when issuing data to the chain.
// Most of these values won't exist in real-world devices, but it's done
// this way so we're ready for it (also, if using the WS2811 driver IC,
// one might have their pixels set up in any weird permutation).

// Bits 5,4 of this value are the offset (0-3) from the first byte of a
// pixel to the location of the red color byte.  Bits 3,2 are the green
// offset and 1,0 are the blue offset.  If it is an RGBW-type device
// (supporting a white primary in addition to R,G,B), bits 7,6 are the
// offset to the white byte...otherwise, bits 7,6 are set to the same value
// as 5,4 (red) to indicate an RGB (not RGBW) device.
// i.e. binary representation:
// 0bWWRRGGBB for RGBW devices
// 0bRRRRGGBB for RGB

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:        W          R          G          B
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2)) ///< Transmit as R,G,B
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1)) ///< Transmit as R,B,G
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2)) ///< Transmit as G,R,B
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1)) ///< Transmit as G,B,R
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0)) ///< Transmit as B,R,G
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0)) ///< Transmit as B,G,R

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3)) ///< Transmit as W,R,G,B
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2)) ///< Transmit as W,R,B,G
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3)) ///< Transmit as W,G,R,B
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2)) ///< Transmit as W,G,B,R
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1)) ///< Transmit as W,B,R,G
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1)) ///< Transmit as W,B,G,R

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3)) ///< Transmit as R,W,G,B
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2)) ///< Transmit as R,W,B,G
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3)) ///< Transmit as R,G,W,B
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2)) ///< Transmit as R,G,B,W
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1)) ///< Transmit as R,B,W,G
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1)) ///< Transmit as R,B,G,W

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3)) ///< Transmit as G,W,R,B
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2)) ///< Transmit as G,W,B,R
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3)) ///< Transmit as G,R,W,B
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2)) ///< Transmit as G,R,B,W
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1)) ///< Transmit as G,B,W,R
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1)) ///< Transmit as G,B,R,W

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0)) ///< Transmit as B,W,R,G
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0)) ///< Transmit as B,W,G,R
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0)) ///< Transmit as B,R,W,G
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0)) ///< Transmit as B,R,G,W
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0)) ///< Transmit as B,G,W,R
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0)) ///< Transmit as B,G,R,W

// Add NEO_KHZ400 to the color order value to indicate a 400 KHz device.
// All but the earliest v1 NeoPixels expect an 800 KHz data stream, this is
// the default if unspecified. Because flash space is very limited on ATtiny
// devices (e.g. Trinket, Gemma), v1 NeoPixels aren't handled by default on
// those chips, though it can be enabled by removing the ifndef/endif below,
// but code will be bigger. Conversely, can disable the NEO_KHZ400 line on
// other MCUs to remove v1 support and save a little space.

#define NEO_KHZ800 0x0000 ///< 800 KHz data transmission
#ifndef __AVR_ATtiny85__
#define NEO_KHZ400 0x0100 ///< 400 KHz data transmission
#endif

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#ifdef NEO_KHZ400
typedef uint16_t neoPixelType; ///< 3rd arg to NeoPixel constructor
#else
typedef uint8_t neoPixelType; ///< 3rd arg to NeoPixel constructor
#endif

/*!
  @brief   Structure containing parameters for WS2812B driver function.
*/
typedef struct {
  int16_t pin;        ///< GPIO pin number used for data output
  uint8_t *pixels;    ///< Pointer to pixel data buffer (RGB or RGBW format)
  uint16_t numBytes;  ///< Number of bytes in the pixel buffer
  bool is800KHz;      ///< true for 800 KHz (WS2812B), false for 400 KHz (WS2811)
  void *userData;     ///< Optional user data pointer (can be NULL)
} neopixel_driver_params_t;

/*!
  @brief   WS2812B/NeoPixel hardware driver function pointer type.
  @param   params    Pointer to driver parameters structure

  This function should transmit the pixel data to the WS2812B/NeoPixel strip
  using the platform-specific hardware (PWM, SPI, bit-banging, etc.).
  The function is responsible for:
  - Sending data with correct timing (0.3us/0.9us for '0', 0.6us/0.6us for '1')
  - Sending reset pulse (>50us LOW) after all data
  - Handling interrupts if needed for precise timing
*/
typedef void (*neopixel_driver_func_t)(neopixel_driver_params_t *params);

/*!
    @brief  Structure that stores state for interacting with
            Adafruit NeoPixels and compatible devices.
*/
struct NeoPixel {
#ifdef NEO_KHZ400 // If 400 KHz NeoPixel support enabled...
  bool is800KHz; ///< true if 800 KHz pixels
#endif

  bool begun;         ///< true if begin() previously called successfully
  uint16_t numLEDs;   ///< Number of RGB LEDs in strip
  uint16_t numBytes;  ///< Size of 'pixels' buffer below
  int16_t pin;        ///< Output pin number (-1 if not yet set)
  uint8_t brightness; ///< Strip brightness 0-255 (stored as +1)
  uint8_t *pixels;    ///< Holds LED color values (3 or 4 bytes each)
  uint8_t rOffset;    ///< Red index within each 3- or 4-byte pixel
  uint8_t gOffset;    ///< Index of green byte
  uint8_t bOffset;    ///< Index of blue byte
  uint8_t wOffset;    ///< Index of white (==rOffset if no white)
  uint32_t endTime;   ///< Latch timing reference

#ifdef __AVR__
  volatile uint8_t *port; ///< Output PORT register
  uint8_t pinMask;        ///< Output PORT bitmask
#endif

#if defined(ARDUINO_ARCH_STM32) || \
    defined(ARDUINO_ARCH_ARDUINO_CORE_STM32) || \
    defined(ARDUINO_ARCH_CH32) || \
    defined(_PY32_DEF_)
  void *gpioPort; ///< Output GPIO PORT
  uint32_t gpioPin;       ///< Output GPIO PIN
#endif

#if defined(ARDUINO_ARCH_RP2040)
  void *pio;              ///< PIO instance
  uint32_t pio_sm;        ///< State machine number
  uint32_t pio_program_offset; ///< Program offset
#endif

  // External driver function pointer (for custom hardware implementations)
  neopixel_driver_func_t driver_func; ///< Platform-specific driver function
  void *driver_user_data;             ///< User data passed to driver function
};

// Typedef for convenience (after struct definition for C51 compatibility)
typedef struct NeoPixel NeoPixel;

/*!
  @brief   Initialize a NeoPixel strip object.
  @param   strip  Pointer to NeoPixel structure
  @param   n      Number of NeoPixels in strand.
  @param   p      Arduino pin number which will drive the NeoPixel data in.
  @param   t      Pixel type -- add together NEO_* constants defined in
                  NeoPixel.h, for example NEO_GRB+NEO_KHZ800 for
                  NeoPixels expecting an 800 KHz (vs 400 KHz) data stream
                  with color bytes expressed in green, red, blue order per
                  pixel.
  @return  Pointer to initialized NeoPixel object, or NULL on failure.
*/
NeoPixel* neopixel_init(NeoPixel *strip, uint16_t n, int16_t p, neoPixelType t);

/*!
  @brief   Initialize an empty NeoPixel strip object (deprecated).
  @param   strip  Pointer to NeoPixel structure
  @return  Pointer to initialized NeoPixel object.
*/
NeoPixel* neopixel_init_empty(NeoPixel *strip);

/*!
  @brief   Deallocate NeoPixel object, set data pin back to INPUT.
  @param   strip  Pointer to NeoPixel structure
*/
void neopixel_deinit(NeoPixel *strip);

/*!
  @brief   Configure NeoPixel pin for output.
  @param   strip  Pointer to NeoPixel structure
  @returns False if we weren't able to claim resources required
*/
bool neopixel_begin(NeoPixel *strip);

/*!
  @brief   Transmit pixel data in RAM to NeoPixels.
  @param   strip  Pointer to NeoPixel structure
*/
void neopixel_show(NeoPixel *strip);

/*!
  @brief   Set/change the NeoPixel output pin number.
  @param   strip  Pointer to NeoPixel structure
  @param   p      Arduino pin number (-1 = no pin).
*/
void neopixel_setPin(NeoPixel *strip, int16_t p);

/*!
  @brief   Set a pixel's color using separate red, green and blue
           components. If using RGBW pixels, white will be set to 0.
  @param   strip  Pointer to NeoPixel structure
  @param   n      Pixel index, starting from 0.
  @param   r      Red brightness, 0 = minimum (off), 255 = maximum.
  @param   g      Green brightness, 0 = minimum (off), 255 = maximum.
  @param   b      Blue brightness, 0 = minimum (off), 255 = maximum.
*/
void neopixel_setPixelColor(NeoPixel *strip, uint16_t n, uint8_t r, uint8_t g, uint8_t b);

/*!
  @brief   Set a pixel's color using separate red, green, blue and white
           components (for RGBW NeoPixels only).
  @param   strip  Pointer to NeoPixel structure
  @param   n      Pixel index, starting from 0.
  @param   r      Red brightness, 0 = minimum (off), 255 = maximum.
  @param   g      Green brightness, 0 = minimum (off), 255 = maximum.
  @param   b      Blue brightness, 0 = minimum (off), 255 = maximum.
  @param   w      White brightness, 0 = minimum (off), 255 = maximum, ignored
                  if using RGB pixels.
*/
void neopixel_setPixelColorRGBW(NeoPixel *strip, uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/*!
  @brief   Set a pixel's color using a 32-bit 'packed' RGB or RGBW value.
  @param   strip  Pointer to NeoPixel structure
  @param   n      Pixel index, starting from 0.
  @param   c      32-bit color value. Most significant byte is white (for RGBW
                  pixels) or ignored (for RGB pixels), next is red, then green,
                  and least significant byte is blue.
*/
void neopixel_setPixelColor32(NeoPixel *strip, uint16_t n, uint32_t c);

/*!
  @brief   Fill all or part of the NeoPixel strip with a color.
  @param   strip  Pointer to NeoPixel structure
  @param   c      32-bit color value. Most significant byte is white (for
                  RGBW pixels) or ignored (for RGB pixels), next is red,
                  then green, and least significant byte is blue. If all
                  arguments are unspecified, this will be 0 (off).
  @param   first  Index of first pixel to fill, starting from 0. Must be
                  in-bounds, no clipping is performed. 0 if unspecified.
  @param   count  Number of pixels to fill, as a positive value. Passing
                  0 or leaving unspecified will fill to end of strip.
*/
void neopixel_fill(NeoPixel *strip, uint32_t c, uint16_t first, uint16_t count);

/*!
  @brief   Adjust output brightness.
  @param   strip  Pointer to NeoPixel structure
  @param   b      Brightness setting, 0=minimum (off), 255=brightest.
*/
void neopixel_setBrightness(NeoPixel *strip, uint8_t b);

/*!
  @brief   Fill the whole NeoPixel strip with 0 / black / off.
  @param   strip  Pointer to NeoPixel structure
*/
void neopixel_clear(NeoPixel *strip);

/*!
  @brief   Change the length of a previously-declared NeoPixel strip object.
  @param   strip  Pointer to NeoPixel structure
  @param   n      New length of strip, in pixels.
*/
void neopixel_updateLength(NeoPixel *strip, uint16_t n);

/*!
  @brief   Change the pixel format of a previously-declared NeoPixel strip object.
  @param   strip  Pointer to NeoPixel structure
  @param   t      Pixel type -- add together NEO_* constants defined in
                  NeoPixel.h
*/
void neopixel_updateType(NeoPixel *strip, neoPixelType t);

/*!
  @brief   Check whether a call to show() will start sending data
           immediately or will 'block' for a required interval.
  @param   strip  Pointer to NeoPixel structure
  @return  1 or true if show() will start sending immediately, 0 or false
           if show() would block (meaning some idle time is available).
*/
bool neopixel_canShow(NeoPixel *strip);

/*!
  @brief   Get a pointer directly to the NeoPixel data buffer in RAM.
  @param   strip  Pointer to NeoPixel structure
  @return  Pointer to NeoPixel buffer (uint8_t* array).
*/
uint8_t* neopixel_getPixels(NeoPixel *strip);

/*!
  @brief   Retrieve the last-set brightness value for the strip.
  @param   strip  Pointer to NeoPixel structure
  @return  Brightness value: 0 = minimum (off), 255 = maximum.
*/
uint8_t neopixel_getBrightness(NeoPixel *strip);

/*!
  @brief   Retrieve the pin number used for NeoPixel data output.
  @param   strip  Pointer to NeoPixel structure
  @return  Arduino pin number (-1 if not set).
*/
int16_t neopixel_getPin(NeoPixel *strip);

/*!
  @brief   Return the number of pixels in a NeoPixel strip object.
  @param   strip  Pointer to NeoPixel structure
  @return  Pixel count (0 if not set).
*/
uint16_t neopixel_numPixels(NeoPixel *strip);

/*!
  @brief   Query the color of a previously-set pixel.
  @param   strip  Pointer to NeoPixel structure
  @param   n      Index of pixel to read (0 = first).
  @return  'Packed' 32-bit RGB or WRGB value. Most significant byte is white
           (for RGBW pixels) or 0 (for RGB pixels), next is red, then green,
           and least significant byte is blue.
*/
uint32_t neopixel_getPixelColor(NeoPixel *strip, uint16_t n);

/*!
  @brief   Fill NeoPixel strip with one or more cycles of hues.
  @param   strip        Pointer to NeoPixel structure
  @param   first_hue    Hue of first pixel, 0-65535, representing one full
                        cycle of the color wheel.
  @param   reps         Number of cycles of the color wheel over the length
                        of the strip. Default is 1. Negative values can be
                        used to reverse the hue order.
  @param   saturation   Saturation (optional), 0-255 = gray to pure hue,
                        default = 255.
  @param   brightness   Brightness/value (optional), 0-255 = off to max,
                        default = 255.
  @param   gammify      If true (default), apply gamma correction to colors
                       for better appearance.
*/
void neopixel_rainbow(NeoPixel *strip, uint16_t first_hue, int8_t reps,
                      uint8_t saturation, uint8_t brightness, bool gammify);

// Static utility functions

/*!
  @brief   An 8-bit integer sine wave function.
  @param   x  Input angle, 0-255; 256 would loop back to zero.
  @return  Sine result, 0 to 255.
*/
uint8_t neopixel_sine8(uint8_t x);

/*!
  @brief   An 8-bit gamma-correction function for basic pixel brightness adjustment.
  @param   x  Input brightness, 0 (minimum or off/black) to 255 (maximum).
  @return  Gamma-adjusted brightness.
*/
uint8_t neopixel_gamma8(uint8_t x);

/*!
  @brief   Convert separate red, green and blue values into a single "packed" 32-bit RGB color.
  @param   r  Red brightness, 0 to 255.
  @param   g  Green brightness, 0 to 255.
  @param   b  Blue brightness, 0 to 255.
  @return  32-bit packed RGB value.
*/
uint32_t neopixel_Color(uint8_t r, uint8_t g, uint8_t b);

/*!
  @brief   Convert separate red, green, blue and white values into a single "packed" 32-bit WRGB color.
  @param   r  Red brightness, 0 to 255.
  @param   g  Green brightness, 0 to 255.
  @param   b  Blue brightness, 0 to 255.
  @param   w  White brightness, 0 to 255.
  @return  32-bit packed WRGB value.
*/
uint32_t neopixel_ColorRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/*!
  @brief   Convert hue, saturation and value into a packed 32-bit RGB color.
  @param   hue  An unsigned 16-bit value, 0 to 65535, representing one full
                loop of the color wheel.
  @param   sat  Saturation, 8-bit value, 0 (min or pure grayscale) to 255
                (max or pure hue). Default of 255 if unspecified.
  @param   val  Value (brightness), 8-bit value, 0 (min / black / off) to
                255 (max or full brightness). Default of 255 if unspecified.
  @return  Packed 32-bit RGB with the most significant byte set to 0.
*/
uint32_t neopixel_ColorHSV(uint16_t hue, uint8_t sat, uint8_t val);

/*!
  @brief   A gamma-correction function for 32-bit packed RGB or WRGB colors.
  @param   x  32-bit packed RGB or WRGB color.
  @return  Gamma-adjusted packed color.
*/
uint32_t neopixel_gamma32(uint32_t x);

/*!
  @brief  Convert pixel color order from string (e.g. "BGR") to NeoPixel color order constant.
  @param   v  Input string. Should be reasonably sanitized (a 3- or 4-character NUL-terminated string).
  @return  One of the NeoPixel color order constants (e.g. NEO_BGR).
*/
neoPixelType neopixel_str2order(const char *v);

/*!
  @brief   Register a custom hardware driver function for WS2812B/NeoPixel.
  @param   strip      Pointer to NeoPixel structure
  @param   driverFunc Function pointer to the hardware driver implementation
  @param   userData   Optional user data pointer (can be NULL) passed to driver function

  This function allows you to register a custom hardware driver for your specific
  platform. The driver function will be called by neopixel_show() to transmit
  pixel data to the LED strip.

  Example usage:
  @code
  void my_ws2812b_driver(neopixel_driver_params_t *params) {
    // Access parameters: params->pin, params->pixels, params->numBytes,
    //                    params->is800KHz, params->userData
    // Your custom implementation using PWM, SPI, bit-banging, etc.
  }

  neopixel_register_driver(&strip, my_ws2812b_driver, NULL);
  @endcode
*/
void neopixel_register_driver(NeoPixel *strip, neopixel_driver_func_t driverFunc, void *userData);

#ifdef __cplusplus
}
#endif

#endif // NEOPIXEL_H
