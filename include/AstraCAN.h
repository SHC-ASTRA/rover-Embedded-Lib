/**
 * @file AstraCAN.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Provides methods for interfacing with Rev SparkMax motor controllers over CAN
 * @version 0.1
 * @date 2024-06-25
 *
 */
#pragma once


#include "ASTRA.h"

#if defined(CORE) || defined(ARM) || defined(FAERIE)

#include <FlexCAN_T4.h>


// Core and FAERIE use CAN1, Arm uses CAN3
#ifdef ARM
typedef FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#else
typedef FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> AstraFCAN;
#endif


// Convert float to little endian decimal representation
inline void Float2LEDec(float x, uint8_t (&buffer_data)[8]);


inline void identifyDevice(AstraFCAN &Can0, int can_id);


inline void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle);

inline void sendHeartbeat(AstraFCAN &Can0, int can_id);

inline void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value);

#endif
