/**
 * @file AstraNP.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Class for controlling Neopixels on ASTRA's ESP32 Feathers
 *
 */
#pragma once

#if !__has_include("Adafruit_NeoPixel.h")
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  adafruit/Adafruit NeoPixel
#else

#include <Arduino.h>
#include "Adafruit_NeoPixel.h"

Adafruit_NeoPixel neopixel(1, 15);

/**
 * @brief Pre-baked status codes for the NeoPixel.
 *        These have associated colors and optionally blink rates.
 * 
 */
enum class AstraNPStatus {
    NP_OFF = 0,    // Turn NeoPixel off
    NP_DEFAULT,    // Default color
    NP_INIT,       // Feather initializing (running setup)
    NP_OK,         // Feather is running normally
    NP_WARN,       // Feather has warning
    NP_ERROR,      // Feather has error, should be looked into
    NP_FATAL,      // Feather has fatal error, needs immediate attention
    NP_PONG_USB,   // Feather is responding to a ping over USB
    NP_PONG_UART,  // Feather is responding to a ping over UART
    NP_PONG_CAN,   // Feather is responding to a ping over CAN
};

class AstraNeoPixel {
   private:
    int16_t neoPin;
    AstraNPStatus status;
    bool blink;
    uint32_t blink_delay;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

   public:
    AstraNeoPixel(int pNeoPin);
    void setStatus(int pStatus);
    int getStatus(void);
    void update(uint32_t time);  // Update the state of the physical NeoPixel based on status and millis()
};

void changeNeopixel(int red, int green, int blue) {
    neopixel.setPixelColor(0, neopixel.Color(red, green, blue));
    neopixel.show();
}

#endif  // __has_include()
