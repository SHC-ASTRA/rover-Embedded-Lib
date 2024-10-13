/**
 * @file AstraCAN.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @author Charles Marmann (cmm0077@uah.edu)
 * @brief Provides methods for interfacing with Rev SparkMax motor controllers over CAN
 *
 */
#pragma once

// Must have the CAN library corresponding to the current MCU
#if defined(CORE_TEENSY) && !__has_include("FlexCAN_T4.h")  // If on Teensy, must have FlexCAN_T4
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  https://github.com/tonton81/FlexCAN_T4

#elif defined(ESP32) && !__has_include("ESP32-TWAI-CAN.hpp")  // If on ESP32, must have ESP32-TWAI-CAN
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  handmade0octopus/ESP32-TWAI-CAN@^1.0.1


#else  // We have the required library.


#    if defined(CORE_TEENSY)

#        include <FlexCAN_T4.h>  // https://github.com/tonton81/FlexCAN_T4

// Core and FAERIE use CAN1, Arm uses CAN3
#        ifdef ARM
typedef FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#        else
typedef FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#        endif


#    elif defined(ESP32)

#        include <ESP32-TWAI-CAN.hpp>  // handmade0octopus/ESP32-TWAI-CAN

// Charles's stuff
#        include <stdint.h>
#        include <stdio.h>

#        include <cstring>

typedef TwaiCAN AstraFCAN;

#    endif


// Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (&buffer_data)[8]);


void identifyDevice(AstraFCAN &Can0, int can_id);


void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle);

void sendHeartbeat(AstraFCAN &Can0, int can_id);

void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value);

#endif  // __has_include()
