//WHAT THE HELL AM I DOING.h?
/**
 * @file AstraCAN.h
 * @author Charles Marmann (cmm0077@uah.edu)
 * @brief Provides methods for interfacing with Rev SparkMax motor controllers over CAN
 *
 */
#pragma once

// #if !__has_include("ESP32-TWAI-CAN.hpp")
// #    error Missing library! Please add the following line to lib_deps in platformio.ini:  https://github.com/tonton81/FlexCAN_T4

// #else

#include <ESP32-TWAI-CAN.hpp>  // https://github.com/tonton81/FlexCAN_T4

#include <stdio.h>
#include <stdint.h>

// Core and FAERIE use CAN1, Arm uses CAN3
#    ifdef ARM
typedef TwaiCAN AstraFCAN;
#    else
typedef TwaiCAN AstraFCAN;
#    endif


// Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (&buffer_data)[8]);


void identifyDevice(AstraFCAN &Can0, int can_id);


void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle);

void sendHeartbeat(AstraFCAN &Can0, int can_id);

void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value);
