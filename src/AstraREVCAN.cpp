/**
 * @file AstraREVCAN.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA's utilities for interfacing with the REV Sparkmax over CAN
 *
 */

// Don't require a platformio project to download a MCU CAN library if not using CAN
// (AstraREVCAN.h will only be included if main.cpp includes it or we have the required MCU CAN
// library)
#if (defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")) || \
     (defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h"))
#    include "AstraREVCAN.h"
#endif


// Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (&buffer_data)[8]) {
    unsigned char b[8] = {0};
    memcpy(b, &x, 4);
    // int* buffer_data[4];
    for (int i = 0; i < 4; i++) {
        buffer_data[i] = b[i];
    }
    for (int i = 4; i < 8; i++) {
        buffer_data[i] = 0;
    }
}


//--------------------------------------------------------------------------//
//   ESP32                                                                  //
//--------------------------------------------------------------------------//

#if defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")


void CAN_sendSpeed(uint8_t deviceId, float speed, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    Float2LEDec(speed, frame);
    CAN_sendPacket(deviceId, 0x12, frame, 8, Can0);
}

void CAN_sendDutyCycle(uint8_t deviceId, float dutyCycle, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    Float2LEDec(dutyCycle, frame);
    CAN_sendPacket(deviceId, 0x02, frame, 8, Can0);
}


void CAN_sendHeartbeat(uint8_t deviceId, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(deviceId, 0xB2, frame, 8, Can0);
}

void CAN_identifySparkMax(uint8_t deviceId, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(deviceId, 0x76, frame, 0, Can0);
}

void CAN_setParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID,
                      sparkMax_ParameterType type, uint32_t value, AstraCAN& Can0) {
    uint8_t frame[8] = {0};  // First 32 bits of frame are value, next 8 bits are type
    Float2LEDec(static_cast<float>(value), frame);
    frame[4] = static_cast<uint8_t>(type);

    CAN_sendPacket(deviceId, static_cast<uint8_t>(parameterID) | 0x300, frame, 5, Can0);
}


// Given direct values for the CAN packet
void CAN_sendPacket(uint32_t messageID, uint8_t data[], uint8_t dataLen, AstraCAN& Can0) {
    CanFrame outMsg;
    outMsg.extd = 1;  // All REV CAN messages are extended
    outMsg.data_length_code = dataLen;
    outMsg.identifier = messageID;
    outMsg.data[0] = 0;  // Just in case... TODO: needed???
    for (uint8_t i = 0; i < dataLen; i++)
        outMsg.data[i] = data[i];
    Can0.writeFrame(outMsg);
    Serial.println("Send with ID ");
    Serial.println(outMsg.identifier, HEX);
}

// Using target device REV ID and REV API ID
void CAN_sendPacket(uint8_t deviceId, int32_t apiId, uint8_t data[], uint8_t dataLen,
                    AstraCAN& Can0) {
    uint32_t createdId = 0x2050000;
    // createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
    // createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
    createdId |= (apiId & 0x3FF) << 6;
    createdId |= (deviceId & 0x3F);

    CAN_sendPacket(createdId, data, dataLen, Can0);
}


//--------------------------------------------------------------------------//
//   Teensy                                                                 //
//--------------------------------------------------------------------------//

#elif defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h")


void identifyDevice(AstraCAN& Can0, int can_id) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 8;

    msg.identifier = 0x2051D80 + can_id;
    for (uint8_t i = 0; i < 8; i++)
        msg.data[i] = 0;
    msg.data[0] = can_id;
    Can0.writeFrame(msg);
}


void sendDutyCycle(AstraCAN& Can0, int can_id, float duty_cycle) {
    CanFrame msg = {0};
    msg.identifier = 0x2050080 + can_id;
    msg.extd = 1;
    msg.data_length_code = 8;

    Float2LEDec(duty_cycle, msg.data);

    Can0.writeFrame(msg);
}

void sendHeartbeat(AstraCAN& Can0, int can_id) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 8;

    msg.identifier = 0x2052C80;  // non-Rio heartbeat
    for (uint8_t i = 0; i < 8; i++)
        msg.data[i] = 0;
    msg.data[0] = pow(2, can_id);
    Can0.writeFrame(msg);
    // Serial.println(msg.id);
}

void setParameter(AstraCAN& Can0, int can_id, uint8_t paramID, uint32_t value) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 5;

    msg.identifier = 0x2050000;  // + can_id;
    msg.identifier |= ((uint8_t)can_id & 0x3F);
    msg.identifier |= ((paramID | 0x03) & 0x3FF) << 6;
    msg.data[3] = (uint8_t)value;
    // Float2LEDec(value, msg.data);
    msg.data[4] = 3;
    // msg.data[4] = paramID;         // Parameter ID
    Can0.writeFrame(msg);
}


#endif  // End microcontroller check
