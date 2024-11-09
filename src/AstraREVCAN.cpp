/**
 * @file AstraREVCAN.cpp
 * @author David Sharpe (you@domain.com)
 * @brief ASTRA's utilities for interfacing with the REV Sparkmax over CAN
 *
 */

#include "AstraREVCAN.h"

#include <Arduino.h>

#include "AstraCAN.h"


/**
 * @brief Identify Sparkmax
 * 
 * @param storage 
 * @param Can0 
 */
void identify(CANStorage* storage, AstraCAN& Can0) {
    uint8_t frame[8] = {0};
    int32_t* status;
    HAL_WriteCANPacket(storage, frame, 0, 0x76, status, Can0);
}


/**
 * @brief Set parameter of sparkmax; returns true on success
 *
 * @param storage
 * @param parameterID
 * @param type
 * @param value
 * @param Can0
 * @return true
 * @return false
 */
void setParameterNew(CANStorage* storage, uint8_t parameterID, int32_t type, uint32_t value,
                  AstraCAN& Can0) {
    // mov     eax, [rbp+value]
    // mov     dword ptr [rbp+frame], eax
    // mov     eax, [rbp+type]
    // mov     byte ptr [rbp+frame+4], al
    // 0- value[0]
    // 1- value[1]
    // 2- value[2]
    // 3- value[3]
    // 4- type[0]
    // 5- n/a
    // 6- n/a
    // 7- n/a

    // First 32 bits of frame are value, next 8 bits are type
    uint8_t frame[8] = {0};
    Float2LEDec(static_cast<float>(value), frame);
    frame[4] = static_cast<uint8_t>(type);


    int32_t* status;
    HAL_WriteCANPacket(storage, frame, 5, parameterID | 0x03, status, Can0);
}


void HAL_WriteCANPacket(CANStorage* storage, uint8_t data[], uint8_t length, int32_t apiId,
                        int32_t* status, AstraCAN& Can0) {
    int32_t id = CreateCANId(storage, apiId);

    HAL_CAN_SendMessage(id, data, length, status, Can0);

    if (*status != 0) {
        return;
    }
}

void HAL_CAN_SendMessage(uint32_t messageID, uint8_t data[], uint8_t dataSize, int32_t* status,
                         AstraCAN& Can0) {
    CanFrame outMsg;
    outMsg.extd = 1;  // All REV CAN messages are extended
    outMsg.data_length_code = dataSize;
    outMsg.identifier = messageID;
    for (uint8_t i = 0; i < dataSize; i++)
        outMsg.data[i] = data[i];
    // Can0.writeFrame(outMsg);
}


int32_t CreateCANId(CANStorage* storage, int32_t apiId) {
    int32_t createdId = 0;
    createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
    createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
    createdId |= (apiId & 0x3FF) << 6;
    createdId |= (storage->deviceId & 0x3F);
    return createdId;
}
