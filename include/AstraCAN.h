/**
 * @file AstraCAN.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Provides methods for interfacing with Rev SparkMax motor controllers over CAN
 * @version 0.1.3
 * @date 2024-07-06
 *
 */
#pragma once

#if !__has_include("FlexCAN_T4.h")
#error Missing library! Please add the following line to lib_deps in platformio.ini:  https://github.com/tonton81/FlexCAN_T4

#else

#include <FlexCAN_T4.h>  // https://github.com/tonton81/FlexCAN_T4


// Core and FAERIE use CAN1, Arm uses CAN3
#ifdef ARM
typedef FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#else
typedef FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#endif


// Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (&buffer_data)[8]);


void identifyDevice(AstraFCAN &Can0, int can_id);


void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle);

void sendHeartbeat(AstraFCAN &Can0, int can_id);

void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value);

#endif  // __has_include
