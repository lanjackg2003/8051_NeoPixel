/*!
 * @file NeoPixel.c
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
 * License along with NeoPixel. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "NeoPixel.h"

// C51 compatibility: memory management and utility functions
#ifdef C51_COMPILER
// C51 doesn't have standard memset, malloc, free
// These need to be implemented or use C51-specific functions
#include <string.h>  // C51 has some string functions
// For C51, we may need to use xdata or pdata for large buffers
// malloc/free may not be available, so we'll need static allocation or xdata
#else
#include <stdlib.h>
#include <string.h>
#endif

#if defined(TARGET_LPC1768)
#include <time.h>
#endif

#if defined(NRF52) || defined(NRF52_SERIES)
#include "nrf.h"
//#define NRF52_DISABLE_INT
#endif

#if defined(ARDUINO_ARCH_NRF52840)
#if defined __has_include
#if __has_include(<pinDefinitions.h>)
#include <pinDefinitions.h>
#endif
#endif
#endif

#if defined(ARDUINO_ARCH_MBED)
#include "mbed.h"
#endif

// Lookup tables (same as in original C++ version)
// For C51, these should be in code memory
static const uint8_t PROGMEM _NeoPixelSineTable[256] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170,
    173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211,
    213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240,
    241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254,
    254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251,
    250, 250, 249, 248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232,
    230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201, 198,
    196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162, 158, 155,
    152, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115, 112, 109,
    106, 103, 100, 97,  93,  90,  88,  85,  82,  79,  76,  73,  70,  67,  65,
    62,  59,  57,  54,  52,  49,  47,  44,  42,  40,  37,  35,  33,  31,  29,
    27,  25,  23,  21,  20,  18,  17,  15,  14,  12,  11,  10,  9,   7,   6,
    5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,   0,   0,   0,
    0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,   10,  11,
    12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,  37,
    40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
    79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112, 115, 118, 121,
    124};

static const uint8_t PROGMEM _NeoPixelGammaTable[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
    3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
    17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
    25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
    36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
    49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
    64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
    82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
    103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
    127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
    154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
    184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
    218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
    255};

// External platform-specific functions
#if defined(ESP8266)
extern void espShow(uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);
#elif defined(ESP32)
extern void espShow(uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
extern void espInit(void);
#endif
#endif

#if defined(KENDRYTE_K210)
extern void k210Show(uint8_t pin, uint8_t *pixels, uint32_t numBytes, bool is800KHz);
#endif

#if defined(ARDUINO_ARCH_PSOC6)
extern void psoc6_show(uint8_t pin, uint8_t *pixels, uint32_t numBytes, bool is800KHz);
#endif

// Platform-specific helper functions
#if defined(ARDUINO_ARCH_RP2040)
static bool rp2040claimPIO(NeoPixel *strip);
static void rp2040releasePIO(NeoPixel *strip);
static void rp2040Show(NeoPixel *strip, uint8_t *pixels, uint32_t numBytes);
#endif

// Forward declarations for platform-specific show implementations
// These will be implemented based on the original C++ code
static void neopixel_show_platform(NeoPixel *strip);

// C51 compatibility: helper functions
#ifdef C51_COMPILER
// C51-compatible memset (if not available)
static void neopixel_memset(void *s, int c, unsigned int n) {
  unsigned char *p = (unsigned char *)s;
  while (n--) {
    *p++ = (unsigned char)c;
  }
}
#define NEOPIXEL_MEMSET neopixel_memset
#else
#define NEOPIXEL_MEMSET memset
#endif

// Initialize a NeoPixel strip
NeoPixel* neopixel_init(NeoPixel *strip, uint16_t n, int16_t p, neoPixelType t) {
  if (!strip) return NULL;

  // Initialize structure
  NEOPIXEL_MEMSET(strip, 0, sizeof(NeoPixel));

  // Initialize driver function pointer to NULL (no driver registered by default)
  strip->driver_func = NULL;
  strip->driver_user_data = NULL;

#ifdef NEO_KHZ400
  strip->is800KHz = (t < 256); // 400 KHz flag is 1<<8
#endif

  strip->begun = false;
  strip->brightness = 0;
  strip->pixels = NULL;
  strip->endTime = 0;

  neopixel_updateType(strip, t);
  neopixel_updateLength(strip, n);
  neopixel_setPin(strip, p);

#if defined(ESP32)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  espInit();
#endif
#endif

  return strip;
}

// Initialize an empty NeoPixel strip (deprecated)
NeoPixel* neopixel_init_empty(NeoPixel *strip) {
  if (!strip) return NULL;

  NEOPIXEL_MEMSET(strip, 0, sizeof(NeoPixel));

#ifdef NEO_KHZ400
  strip->is800KHz = true;
#endif

  strip->begun = false;
  strip->numLEDs = 0;
  strip->numBytes = 0;
  strip->pin = -1;
  strip->brightness = 0;
  strip->pixels = NULL;
  strip->rOffset = 1;
  strip->gOffset = 0;
  strip->bOffset = 2;
  strip->wOffset = 1;
  strip->endTime = 0;

  return strip;
}

// Deinitialize a NeoPixel strip
void neopixel_deinit(NeoPixel *strip) {
  if (!strip) return;

#ifdef ARDUINO_ARCH_ESP32
  // Release RMT resources
  if (strip->pixels) {
    NEOPIXEL_MEMSET(strip->pixels, 0, strip->numBytes);
    strip->numLEDs = strip->numBytes = 0;
    neopixel_show(strip);
  }
#endif

#if defined(ARDUINO_ARCH_RP2040)
  rp2040releasePIO(strip);
#endif

#ifdef C51_COMPILER
  // C51: User manages memory allocation
  // Just clear the pointer, don't free
  heap_free(strip->pixels);
  strip->pixels = NULL;
#else
  if (strip->pixels) {
    free(strip->pixels);
    strip->pixels = NULL;
  }
#endif

  if (strip->pin >= 0) {
#ifdef ARDUINO
    pinMode(strip->pin, INPUT);
#endif
  }
}

// Begin using the strip
bool neopixel_begin(NeoPixel *strip) {
  if (!strip) return false;

  if (strip->pin < 0) {
    strip->begun = false;
    return false;
  }

#ifdef ARDUINO
  pinMode(strip->pin, OUTPUT);
  digitalWrite(strip->pin, LOW);
#else
  // For non-Arduino platforms, assume pin is already configured by user
  // User should configure GPIO pin as output before calling neopixel_begin()
#endif

#if defined(ARDUINO_ARCH_RP2040)
  rp2040releasePIO(strip);
  if (!rp2040claimPIO(strip)) {
    strip->begun = false;
    return false;
  }
#endif

  strip->begun = true;
  return true;
}

// Update length
void neopixel_updateLength(NeoPixel *strip, uint16_t n) {
  if (!strip) return;

#ifdef C51_COMPILER
  // C51: For now, we'll use the provided buffer
  // User should allocate buffer using xdata or pdata before calling init
  // Or use static allocation
  if (strip->pixels) {
    // In C51, if using malloc, it might be in xdata
    // For now, assume user manages memory
    heap_free(strip->pixels);
    strip->pixels = NULL;
  }
#else
  if (strip->pixels) {
    free(strip->pixels);
    strip->pixels = NULL;
  }
#endif

  strip->numBytes = n * ((strip->wOffset == strip->rOffset) ? 3 : 4);

#ifdef C51_COMPILER
  // C51: User should pre-allocate buffer or use static allocation
  // We'll assume strip->pixels is already allocated by user
  strip->pixels = (uint8_t *)heap_alloc(strip->numBytes);
  if (strip->pixels) {
    NEOPIXEL_MEMSET(strip->pixels, 0, strip->numBytes);
    strip->numLEDs = n;
  } else {
    strip->numLEDs = strip->numBytes = 0;
  }
#else
  strip->pixels = (uint8_t *)malloc(strip->numBytes);
  if (strip->pixels) {
    NEOPIXEL_MEMSET(strip->pixels, 0, strip->numBytes);
    strip->numLEDs = n;
  } else {
    strip->numLEDs = strip->numBytes = 0;
  }
#endif
}

// Update type
void neopixel_updateType(NeoPixel *strip, neoPixelType t) {
  bool oldThreeBytesPerPixel;
  bool newThreeBytesPerPixel;

  if (!strip) return;

  oldThreeBytesPerPixel = (strip->wOffset == strip->rOffset);

  strip->wOffset = (t >> 6) & 0x03;
  strip->rOffset = (t >> 4) & 0x03;
  strip->gOffset = (t >> 2) & 0x03;
  strip->bOffset = t & 0x03;

#ifdef NEO_KHZ400
  strip->is800KHz = (t < 256);
#endif

  if (strip->pixels) {
    newThreeBytesPerPixel = (strip->wOffset == strip->rOffset);
    if (newThreeBytesPerPixel != oldThreeBytesPerPixel) {
      neopixel_updateLength(strip, strip->numLEDs);
    }
  }
}

// Set pin
void neopixel_setPin(NeoPixel *strip, int16_t p) {
  if (!strip) return;

  if (strip->begun && (strip->pin >= 0)) {
#ifdef ARDUINO
    pinMode(strip->pin, INPUT);
#endif
  }

  strip->pin = p;

  if (strip->begun) {
#ifdef ARDUINO
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
#endif
  }

#ifdef __AVR__
  strip->port = portOutputRegister(digitalPinToPort(p));
  strip->pinMask = digitalPinToBitMask(p);
#endif

#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ARDUINO_CORE_STM32)
  strip->gpioPort = (void*)digitalPinToPort(p);
  // Note: gpioPin needs platform-specific conversion
  // This is a simplified version - actual implementation depends on platform
#endif

#if defined(ARDUINO_ARCH_CH32)
  // CH32 specific pin setup would go here
#endif
}

// Set pixel color (RGB)
void neopixel_setPixelColor(NeoPixel *strip, uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t *p;

  if (!strip || n >= strip->numLEDs) return;

  if (strip->brightness) {
    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;
  }

  if (strip->wOffset == strip->rOffset) {
    p = &strip->pixels[n * 3];
  } else {
    p = &strip->pixels[n * 4];
    p[strip->wOffset] = 0;
  }

  p[strip->rOffset] = r;
  p[strip->gOffset] = g;
  p[strip->bOffset] = b;
}

// Set pixel color (RGBW)
void neopixel_setPixelColorRGBW(NeoPixel *strip, uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  uint8_t *p;

  if (!strip || n >= strip->numLEDs) return;

  if (strip->brightness) {
    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;
    w = (w * strip->brightness) >> 8;
  }

  if (strip->wOffset == strip->rOffset) {
    p = &strip->pixels[n * 3];
  } else {
    p = &strip->pixels[n * 4];
    p[strip->wOffset] = w;
  }

  p[strip->rOffset] = r;
  p[strip->gOffset] = g;
  p[strip->bOffset] = b;
}

// Set pixel color (32-bit)
void neopixel_setPixelColor32(NeoPixel *strip, uint16_t n, uint32_t c) {
  uint8_t r, g, b, w;
  uint8_t *p;

  if (!strip || n >= strip->numLEDs) return;

  r = (uint8_t)(c >> 16);
  g = (uint8_t)(c >> 8);
  b = (uint8_t)c;

  if (strip->brightness) {
    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;
  }

  if (strip->wOffset == strip->rOffset) {
    p = &strip->pixels[n * 3];
  } else {
    p = &strip->pixels[n * 4];
    w = (uint8_t)(c >> 24);
    p[strip->wOffset] = strip->brightness ? ((w * strip->brightness) >> 8) : w;
  }

  p[strip->rOffset] = r;
  p[strip->gOffset] = g;
  p[strip->bOffset] = b;
}

// Fill strip
void neopixel_fill(NeoPixel *strip, uint32_t c, uint16_t first, uint16_t count) {
  uint16_t end;
  uint16_t i;

  if (!strip || first >= strip->numLEDs) return;

  if (count == 0) {
    end = strip->numLEDs;
  } else {
    end = first + count;
    if (end > strip->numLEDs) end = strip->numLEDs;
  }

  for (i = first; i < end; i++) {
    neopixel_setPixelColor32(strip, i, c);
  }
}

// Set brightness
void neopixel_setBrightness(NeoPixel *strip, uint8_t b) {
  uint8_t newBrightness;
  uint8_t c;
  uint8_t *ptr;
  uint8_t oldBrightness;
  uint16_t scale;
  uint16_t i;

  if (!strip) return;

  newBrightness = b + 1;
  if (newBrightness != strip->brightness) {
    ptr = strip->pixels;
    oldBrightness = strip->brightness - 1;

    if (oldBrightness == 0) {
      scale = 0;
    } else if (b == 255) {
      scale = 65535 / oldBrightness;
    } else {
      scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    }

    for (i = 0; i < strip->numBytes; i++) {
      c = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    strip->brightness = newBrightness;
  }
}

// Clear strip
void neopixel_clear(NeoPixel *strip) {
  if (!strip || !strip->pixels) return;
  NEOPIXEL_MEMSET(strip->pixels, 0, strip->numBytes);
}

// Can show
bool neopixel_canShow(NeoPixel *strip) {
  if (!strip) return false;

#ifdef ARDUINO
  uint32_t now = micros();
  if (strip->endTime > now) {
    strip->endTime = now;
  }
  return (now - strip->endTime) >= 300L;
#else
  // For non-Arduino platforms, you may need to implement micros()
  return true;
#endif
}

// Get pixels
uint8_t* neopixel_getPixels(NeoPixel *strip) {
  if (!strip) return NULL;
  return strip->pixels;
}

// Get brightness
uint8_t neopixel_getBrightness(NeoPixel *strip) {
  if (!strip) return 0;
  return strip->brightness - 1;
}

// Get pin
int16_t neopixel_getPin(NeoPixel *strip) {
  if (!strip) return -1;
  return strip->pin;
}

// Get number of pixels
uint16_t neopixel_numPixels(NeoPixel *strip) {
  if (!strip) return 0;
  return strip->numLEDs;
}

// Get pixel color
uint32_t neopixel_getPixelColor(NeoPixel *strip, uint16_t n) {
  uint8_t *p;

  if (!strip || n >= strip->numLEDs) return 0;

  if (strip->wOffset == strip->rOffset) {
    p = &strip->pixels[n * 3];
    if (strip->brightness) {
      return (((uint32_t)(p[strip->rOffset] << 8) / strip->brightness) << 16) |
             (((uint32_t)(p[strip->gOffset] << 8) / strip->brightness) << 8) |
             ((uint32_t)(p[strip->bOffset] << 8) / strip->brightness);
    } else {
      return ((uint32_t)p[strip->rOffset] << 16) |
             ((uint32_t)p[strip->gOffset] << 8) |
             (uint32_t)p[strip->bOffset];
    }
  } else {
    p = &strip->pixels[n * 4];
    if (strip->brightness) {
      return (((uint32_t)(p[strip->wOffset] << 8) / strip->brightness) << 24) |
             (((uint32_t)(p[strip->rOffset] << 8) / strip->brightness) << 16) |
             (((uint32_t)(p[strip->gOffset] << 8) / strip->brightness) << 8) |
             ((uint32_t)(p[strip->bOffset] << 8) / strip->brightness);
    } else {
      return ((uint32_t)p[strip->wOffset] << 24) |
             ((uint32_t)p[strip->rOffset] << 16) |
             ((uint32_t)p[strip->gOffset] << 8) |
             (uint32_t)p[strip->bOffset];
    }
  }
}

// Rainbow
void neopixel_rainbow(NeoPixel *strip, uint16_t first_hue, int8_t reps,
                      uint8_t saturation, uint8_t brightness, bool gammify) {
  uint16_t i;
  uint16_t hue;
  uint32_t color;

  if (!strip) return;

  for (i = 0; i < strip->numLEDs; i++) {
    hue = first_hue + (i * reps * 65536) / strip->numLEDs;
    color = neopixel_ColorHSV(hue, saturation, brightness);
    if (gammify) color = neopixel_gamma32(color);
    neopixel_setPixelColor32(strip, i, color);
  }
}

// Static utility functions
uint8_t neopixel_sine8(uint8_t x) {
#ifdef ARDUINO
  return pgm_read_byte(&_NeoPixelSineTable[x]);
#else
  return _NeoPixelSineTable[x];
#endif
}

uint8_t neopixel_gamma8(uint8_t x) {
#ifdef ARDUINO
  return pgm_read_byte(&_NeoPixelGammaTable[x]);
#else
  return _NeoPixelGammaTable[x];
#endif
}

uint32_t neopixel_Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t neopixel_ColorRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t neopixel_ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {
  uint8_t r, g, b;
  uint32_t v1;
  uint16_t s1;
  uint8_t s2;

  hue = (hue * 1530L + 32768) / 65536;

  if (hue < 510) {
    b = 0;
    if (hue < 255) {
      r = 255;
      g = hue;
    } else {
      r = 510 - hue;
      g = 255;
    }
  } else if (hue < 1020) {
    r = 0;
    if (hue < 765) {
      g = 255;
      b = hue - 510;
    } else {
      g = 1020 - hue;
      b = 255;
    }
  } else if (hue < 1530) {
    g = 0;
    if (hue < 1275) {
      r = hue - 1020;
      b = 255;
    } else {
      r = 255;
      b = 1530 - hue;
    }
  } else {
    r = 255;
    g = b = 0;
  }

  v1 = 1 + val;
  s1 = 1 + sat;
  s2 = 255 - sat;

  return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
         (((((g * s1) >> 8) + s2) * v1) & 0xff00) |
         (((((b * s1) >> 8) + s2) * v1) >> 8);
}

uint32_t neopixel_gamma32(uint32_t x) {
  uint8_t *y;
  uint8_t i;

  y = (uint8_t *)&x;
  for (i = 0; i < 4; i++) {
    y[i] = neopixel_gamma8(y[i]);
  }
  return x;
}

neoPixelType neopixel_str2order(const char *v) {
  int8_t r, g, b, w;
  char c;
  uint8_t i;

  r = 0;
  g = 0;
  b = 0;
  w = -1;

  if (v) {
    for (i = 0; (c = v[i]) != '\0'; i++) {
      if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
      if (c == 'r') r = i;
      else if (c == 'g') g = i;
      else if (c == 'b') b = i;
      else if (c == 'w') w = i;
    }
    r &= 3;
  }
  if (w < 0) w = r;
  return (w << 6) | (r << 4) | ((g & 3) << 2) | (b & 3);
}

// Platform-specific show() implementation
// This is a large function that needs to include all platform-specific code
// For brevity, I'm including the key platforms. Full implementation would
// include all platforms from the original C++ code.

void neopixel_show(NeoPixel *strip) {
  if (!strip || !strip->pixels) return;

  // Wait for latch time
  while (!neopixel_canShow(strip)) {
    // Busy wait
  }

  // Call platform-specific implementation
  neopixel_show_platform(strip);

#ifdef ARDUINO
  strip->endTime = micros();
#elif defined(C51_COMPILER)
  // C51: You need to implement micros() or use timer-based approach
  strip->endTime = 0;
#else
  // For non-Arduino platforms, you may need to implement micros()
  strip->endTime = 0;
#endif
}

// Platform-specific show implementation
// This function contains all the platform-specific code from the original C++ implementation
static void neopixel_show_platform(NeoPixel *strip) {
  if (!strip || !strip->pixels) return;

  // First, check if a custom driver function is registered
  if (strip->driver_func != NULL) {
    neopixel_driver_params_t params;
    bool is800KHz = true;
#ifdef NEO_KHZ400
    is800KHz = strip->is800KHz;
#endif
    params.pin = strip->pin;
    params.pixels = strip->pixels;
    params.numBytes = strip->numBytes;
    params.is800KHz = is800KHz;
    params.userData = strip->driver_user_data;
    strip->driver_func(&params);
    return;
  }

  // Otherwise, use platform-specific implementations
#if defined(ARDUINO_ARCH_PSOC6)
  psoc6_show(strip->pin, strip->pixels, strip->numBytes,
#ifdef NEO_KHZ400
             strip->is800KHz
#else
             true
#endif
            );
  return;
#endif

#if defined(ESP8266) || defined(ESP32)
  espShow(strip->pin, strip->pixels, strip->numBytes,
#ifdef NEO_KHZ400
          strip->is800KHz ? 1 : 0
#else
          1
#endif
         );
  return;
#endif

#if defined(KENDRYTE_K210)
  k210Show(strip->pin, strip->pixels, strip->numBytes,
#ifdef NEO_KHZ400
           strip->is800KHz
#else
           true
#endif
          );
  return;
#endif

#if defined(ARDUINO_ARCH_RP2040)
  rp2040Show(strip, strip->pixels, strip->numBytes);
  return;
#endif

  // For other platforms (AVR, ARM, etc.), the implementation would go here
  // This is a simplified version - full implementation would include all
  // the platform-specific assembly code from the original C++ version

  // Note: The original C++ code has extensive platform-specific implementations
  // for AVR, ARM (Teensy, SAMD, etc.), STM32, and many other platforms.
  // For a complete port, all of these would need to be converted to C.

  // This is a placeholder that indicates the platform is not yet fully supported
  // in the C version. Users would need to add their platform-specific code here.

#ifndef ARDUINO
  // For non-Arduino platforms without specific implementations,
  // you may need to provide a generic implementation or error
#endif
}

// Register custom hardware driver function
void neopixel_register_driver(NeoPixel *strip, neopixel_driver_func_t driverFunc, void *userData) {
  if (!strip) return;
  strip->driver_func = driverFunc;
  strip->driver_user_data = userData;
}

#if defined(ARDUINO_ARCH_RP2040)
static bool rp2040claimPIO(NeoPixel *strip) {
  if (!strip) return false;

  strip->pio = NULL;
  strip->pio_sm = -1;
  strip->pio_program_offset = 0;

  // This would need to call the actual PIO claim function
  // Implementation depends on the rp2040_pio.h interface
  // For now, this is a placeholder
  return false;
}

static void rp2040releasePIO(NeoPixel *strip) {
  if (!strip || !strip->pio) return;
  // Release PIO resources
  strip->pio = NULL;
  strip->pio_sm = -1;
  strip->pio_program_offset = 0;
}

static void rp2040Show(NeoPixel *strip, uint8_t *pixels, uint32_t numBytes) {
  if (!strip || !pixels || !strip->pio || strip->pio_sm < 0) return;

  // This would use the PIO state machine to send data
  // Implementation depends on the rp2040_pio.h interface
}
#endif

