/**
 * @file AstraREVCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA's utilities for interfacing with the REV Sparkmax over CAN
 *
 */
#pragma once

#include <Arduino.h>  // For fixed-size integer type definitions

#include "AstraCAN.h"
#include "AstraREVTypes.h"


//--------------------------------------------------------------------------//
//   Specific REV Commands                                                  //
//--------------------------------------------------------------------------//

void CAN_enumerate();


void CAN_sendDutyCycle(uint8_t deviceId, float dutyCycle);

void CAN_sendVelocity(uint8_t deviceId, float speed);

void CAN_sendSmartVelocity(uint8_t deviceId, float speed);

void CAN_sendPosition(uint8_t deviceId, float position);


void CAN_sendHeartbeat(uint8_t deviceId);

void CAN_identifySparkMax(uint8_t deviceId);


void CAN_setParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID, sparkMax_ParameterType type,
                      uint32_t value);

void CAN_reqParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID);


//-------------------------------------//
//  Backwards Compatibility Functions  //
//-------------------------------------//

inline void identifyDevice(TwaiCAN& Can0, int can_id) {
    CAN_identifySparkMax(can_id);
}

inline void sendDutyCycle(TwaiCAN& Can0, int can_id, float duty_cycle) {
    CAN_sendDutyCycle(can_id, duty_cycle);
}

inline void sendHeartbeat(TwaiCAN& Can0, int can_id) {
    CAN_sendHeartbeat(can_id);
}

inline void setParameter(TwaiCAN& Can0, int can_id, uint8_t paramID, uint32_t value) {
    Serial.println("ERROR: setParameter is deprecated. Please use CAN_setParameter.");
}


//--------------------------//
//  Basic Helper Functions  //
//--------------------------//

/**
 * @brief Convert float to little endian decimal representation
 *
 * @param[in] x
 * @param buffer_data 64-bit buffer corresponding the data frame of a CAN packet
 */
void Float2LEDec(float x, uint8_t (&buffer_data)[8]);


// Using target device REV ID and REV API ID
void CAN_sendPacket(uint8_t deviceId, int32_t apiId, uint8_t data[], uint8_t dataLen);

// Given direct values for the CAN packet
// In AstraREVCAN.cpp, this is defined differently depending on mcu.
void CAN_sendPacket(uint32_t messageID, uint8_t data[], uint8_t dataLen);

void printREVFrame(CanFrame frame);
