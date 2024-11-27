/**
 * @file AstraREVCAN.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA's utilities for interfacing with the REV Sparkmax over CAN
 *
 */

#ifndef OLD_ASTRACAN_ENABLE

// Don't require a platformio project to download a MCU CAN library if not using CAN
// (AstraREVCAN.h will only be included if main.cpp includes it or we have the required MCU CAN
// library)
#if (defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")) || \
     (defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h"))
#    include "AstraREVCAN.h"
#endif


/**
 * @brief Convert float to little endian decimal representation
 * 
 * @param[in] x 
 * @param buffer_data 64-bit buffer corresponding the data frame of a CAN packet
 */
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
//   Specific REV Commands                                                  //
//--------------------------------------------------------------------------//

void CAN_enumerate(AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(0, 0x99, frame, 0, Can0);
    delay(80);  // Let devices finish enumeration; they will wait ID * 1ms before responding
}

void CAN_sendDutyCycle(uint8_t deviceId, float dutyCycle, AstraCAN& Can0) {
    // DO NOT USE THE NEW CAN FUNCTIONS FOR THIS. SEE COMMENT BELOW.
    // Ask David, Tristan, or Maddy. We don't fucking know why.
    CanFrame msg = {0};
    msg.identifier = 0x2050080 + deviceId;
    msg.extd = 1;
    msg.data_length_code = 8;

    Float2LEDec(dutyCycle, msg.data);

    Can0.writeFrame(msg);

    // WARNING: DO NOT USE THIS CODE. THE MOTORS WILL HAVE A SEIZURE.
    // uint8_t frame[8] = {0};
    // Float2LEDec(dutyCycle, frame);
    // CAN_sendPacket(deviceId, 0x02, frame, 8, Can0);
}

void CAN_sendVelocity(uint8_t deviceId, float speed, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    Float2LEDec(speed, frame);
    CAN_sendPacket(deviceId, 0x12, frame, 8, Can0);
}

void CAN_sendSmartVelocity(uint8_t deviceId, float speed, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    Float2LEDec(speed, frame);
    CAN_sendPacket(deviceId, 0x13, frame, 8, Can0);
}

void CAN_sendPosition(uint8_t deviceId, float position, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    Float2LEDec(position, frame);
    CAN_sendPacket(deviceId, 0x32, frame, 8, Can0);
}


void CAN_sendHeartbeat(uint8_t deviceId, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    frame[0] = pow(2, deviceId);
    CAN_sendPacket(0, 0xB2, frame, 8, Can0);  // Heartbeat is weird...
}

void CAN_identifySparkMax(uint8_t deviceId, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(deviceId, 0x76, frame, 0, Can0);
}


void CAN_setParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID,
                      sparkMax_ParameterType type, uint32_t value, AstraCAN& Can0) {
    uint8_t frame[8] = {0};  // First 32 bits of frame are value, next 8 bits are type
    frame[4] = static_cast<uint8_t>(type);

    // Load value into frame based on type
    if (type == sparkMax_ParameterType::kUint32 || type == sparkMax_ParameterType::kInt32) {
        frame[0] = value & 0xFF;
        frame[1] = (value >> 8) & 0xFF;
        frame[2] = (value >> 16) & 0xFF;
        frame[3] = (value >> 24) & 0xFF;
    } else if (type == sparkMax_ParameterType::kFloat32) {
        Float2LEDec(*reinterpret_cast<float*>(&value), frame);
    } else if (type == sparkMax_ParameterType::kBool) {
        frame[0] = value ? 1 : 0;
    }

    CAN_sendPacket(deviceId, static_cast<uint8_t>(parameterID) | 0x300, frame, 5, Can0);
}

void CAN_reqParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID, AstraCAN& Can0) {
    uint8_t frame[8] = {0};

    CAN_sendPacket(deviceId, static_cast<uint8_t>(parameterID) | 0x300, frame, 0, Can0);
}


//--------------------------------------------------------------------------//
//   Basic Helper functions                                                 //
//--------------------------------------------------------------------------//

// Using target device REV ID and REV API ID
void CAN_sendPacket(uint8_t deviceId, int32_t apiId, uint8_t data[], uint8_t dataLen,
                    AstraCAN& Can0) {
    uint32_t createdId = 0x2050000;
    // createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
    // createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
    createdId |= (apiId & 0x3FF) << 6;
    createdId |= (deviceId & 0x3F);

#ifdef DEBUG
    if (apiId != 0xB2) {  // Don't spam the serial monitor with heartbeats
        Serial.print("Sending to ");
        Serial.print(createdId, HEX);
        Serial.print(" - ");
        for (int i = 0; i < dataLen; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.print(" at ");
        Serial.print(millis());
        Serial.println();
    }
#endif

    CAN_sendPacket(createdId, data, dataLen, Can0);
}

void printREVFrame(CanFrame frame) {
    uint8_t deviceId = frame.identifier & 0x3F;
    uint32_t apiId = (frame.identifier >> 6) & 0x3FF;

    Serial.print(apiId, HEX);
    Serial.print(" from ");
    Serial.print(deviceId);
    // Message data:
    if (frame.data_length_code == 0)
        Serial.print(" - no data.");
    else {
        Serial.print(" - data: (");
        Serial.print(frame.data_length_code);
        Serial.print(" B) ");
        for (int i = 0; i < frame.data_length_code; i++) {
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
    }
    Serial.println();
}


//--------------------------------------------------------------------------//
//   Microcontroller-specific                                               //
//--------------------------------------------------------------------------//

#if defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")

//---------//
//  ESP32  //
//---------//

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
}


#elif defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h")

//----------//
//  Teensy  //
//----------//

// Given direct values for the CAN packet
void CAN_sendPacket(uint32_t messageID, uint8_t data[], uint8_t dataLen, AstraCAN& Can0) {
    CAN_message_t outMsg;
    outMsg.flags.extended = 1;  // All REV CAN messages are extended
    outMsg.len = dataLen;
    outMsg.id = messageID;
    outMsg.bug[0] = 0;  // Just in case... TODO: needed???
    for (uint8_t i = 0; i < dataLen; i++)
        outMsg.buf[i] = data[i];
    Can0.write(outMsg);
}


#endif  // End microcontroller check

#endif
