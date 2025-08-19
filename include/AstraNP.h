/**
 * @file AstraNP.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Class for controlling Neopixels on ASTRA's ESP32 Feathers
 *
 */
#pragma once

#include <Arduino.h>

#define MAX_LEN 5
#define DEFAULT_ON_TIME 300
#define DEFAULT_OFF_TIME 100
#define BET_CYCLE_LEN 2
#define LIMBO_TIME (DEFAULT_OFF_TIME * 8)


struct NPStatus {
    uint32_t color1;
    uint32_t color2;
    unsigned onTime;
    unsigned offTime;
    long duration;  // How long until this status code expires
    long startTime;
    long addTime;  // When this status code was added with addStatus()
};


class AstraNeoPixel {
   private:
    uint8_t npPin;             // Digital pin for NeoPixel DIN
    NPStatus status[MAX_LEN];  // Holds current status codes (added by addStatus())
    int statusCount;           // Number of current status codes
    int currentStatus;         // Currently displayed status code
    long limboStart;           // millis() value when limbo was entered (time between status codes)

   public:
    /**
     * @brief
     *
     * @param pNPPin Neopixel DIN pin (PIN_NEOPIXEL)
     */
    AstraNeoPixel(int pNPPin);

    /**
     * @brief Encounter an error? Add its status code, and specify how long it should show on the neopixel
     *
     * @param pStatus Status code (find in AstraNP.h)
     * @param pDuration seconds (NOT MILLISECONDS) before the status code expires
     */
    void addStatus(NPStatus pCode, long pDuration);

    // Update the state of the physical NeoPixel based on status and millis()
    // NOTE: this function is assumed to run frequently. If it's not, the Neopixel will NOT blink properly.
    // This function must be run atleast 20 Hz. Put it at the top of loop() or in an interrupt.
    void update();

    // For use in setup() to show the status of the setup process
    inline void writeColor(uint32_t color) {
        neopixelWrite(npPin, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
    }
};


//------------------------------------------------------------------------------------------------//
//  Pre-made Statuses

const NPStatus STATUS_NA = {0, 0, 0, 0, 0, 0, 0};
const NPStatus STATUS_IDLE = {0x00FF00, 0x00FF00, DEFAULT_ON_TIME, DEFAULT_ON_TIME, 0, 0, 0};
const NPStatus STATUS_BMP_NOCONN = {0xFF0000, 0x00FF00, DEFAULT_ON_TIME, DEFAULT_OFF_TIME, 0, 0, 0};
const NPStatus STATUS_BNO_NOCONN = {0xFF0000, 0x00E9FF, DEFAULT_ON_TIME, DEFAULT_OFF_TIME, 0, 0, 0};
const NPStatus STATUS_GPS_NOCONN = {0xFF0000, 0xFF0000, DEFAULT_ON_TIME, DEFAULT_OFF_TIME, 0, 0, 0};
const NPStatus STATUS_GPS_NOLOCK = {0xFF0000, 0xFF00FF, DEFAULT_ON_TIME, DEFAULT_OFF_TIME, 0, 0, 0};
const NPStatus STATUS_CAN_NOCONN = {0xF6FF00, 0x00FF00, DEFAULT_ON_TIME, DEFAULT_OFF_TIME, 0, 0, 0};


//------------------------------------------------------------------------------------------------//
//  Setup() Static Colors

const uint32_t COLOR_SETUP_START = 0x005F00;
const uint32_t COLOR_SETUP_COMMS = 0x5900FF;  // purble
const uint32_t COLOR_SETUP_SENSORS = 0xF6FF00;
const uint32_t COLOR_SETUP_MISC = 0xFF00EE;  // pimk
const uint32_t COLOR_SETUP_DONE = 0x00FFB6;  // sky blue sort of
