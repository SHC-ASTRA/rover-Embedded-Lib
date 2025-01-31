/**
 * @file AstraVicCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Easily communicate over ASTRA Vehicle CAN
 *
 */
#pragma once

#include <Arduino.h>

#include "AstraCAN.h"


// Microcontroller VicCAN ID's based on submodule; use these instead of the raw numbers
enum class McuId : uint8_t {
    BROADCAST = 0,
    CORE,
    ARM,
    DIGIT,
    FAERIE,
    CITADEL
};

// Shorthand for the VicCAN ID used by the active MCU
#if defined(CORE)
#    define SUBMODULE_CAN_ID McuId::CORE
#elif defined(ARM)
#    define SUBMODULE_CAN_ID McuId::ARM
#elif defined(DIGIT)
#    define SUBMODULE_CAN_ID McuId::DIGIT
#elif defined(FAERIE)
#    define SUBMODULE_CAN_ID McuId::FAERIE
#elif defined(CITADEL)
#    define SUBMODULE_CAN_ID McuId::CITADEL
#else  // Default - no macro in platformio.ini; will only respond to broadcast messages
#    define SUBMODULE_CAN_ID McuId::BROADCAST
#endif

// Possible datatypes for a VicCAN frame; decides how to decode/encode data
enum class CanDataType : uint8_t {
    DT_NA = 0,
    DT_1i64,
    DT_2i32,
    DT_4i16,
    DT_8i8,
    DT_2f32
};

// Command IDs for standard VicCAN commands
typedef enum CanCmdId : uint8_t {
    // General misc
    CMD_TEST = 0,
    CMD_PING,
    CMD_B_LED,
    CMD_SENSOR_RECON,
    // Misc control
    // Submodule-specific (omitted)
    // Motor control
    // Data request
    CMD_DATA_IMU = 24  // i think?
};


//------------------------------------------------------------------------------------------------//
//  VicCanFrame  -  Contains specific data used to create a CanFrame for inter-MCU comms
//------------------------------------------------------------------------------------------------//

class VicCanFrame {
   public:
    // Within CAN ID
    McuId mcuId;           // 3 bits
    CanDataType dataType;  // 3 bits
    uint8_t cmdId;         // 5 bits
    // Built-in to CAN frame
    bool rtr;     // 1 bit
    uint8_t dlc;  // 4 bits
    uint8_t data[8];

    // Constructor
    VicCanFrame() {
        clear();
    }

    // Resets all data values; for use with static keyword
    void clear() {
        mcuId = McuId::BROADCAST;
        dataType = CanDataType::DT_NA;
        cmdId = 0;
        rtr = false;
        dlc = 0;
        for (int i = 0; i < 8; i++) {
            data[i] = 0;
        }
    }

    // Take a CAN ID and parse it into its components
    void parseCanId(uint32_t id) {
        mcuId = static_cast<McuId>(id & 0x7);
        dataType = static_cast<CanDataType>((id >> 3) & 0x7);
        cmdId = (id >> 6);
    }

    int createCanId() {
        return (static_cast<uint32_t>(mcuId) & 0x7) | ((static_cast<uint32_t>(dataType) & 0x7) << 3) |
               (static_cast<uint32_t>(cmdId) << 6);
    }

    int createCanId(CanDataType pDataType) {
        return (static_cast<uint32_t>(mcuId) & 0x7) | ((static_cast<uint32_t>(pDataType) & 0x7) << 3) |
               (static_cast<uint32_t>(cmdId) << 6);
    }

    // Take an entire CAN frame and parse it into its components
    void parseCanFrame(CanFrame& frame) {
        clear();
        parseCanId(frame.identifier);
        rtr = static_cast<bool>(frame.rtr);
        dlc = frame.data_length_code;
        for (int i = 0; i < dlc; i++) {
            data[i] = frame.data[i];
        }
    }

    inline bool isForMe() {
        return mcuId == SUBMODULE_CAN_ID || mcuId == McuId::BROADCAST;
    }
};


//------------------------------------------------------------------------------------------------//
//  VicCanController  -  Handles reading and responding to VicCanFrames
//------------------------------------------------------------------------------------------------//

class VicCanController {
    CanFrame inCanFrame;
    VicCanFrame inVicCanFrame;
    CanFrame outFrame;

   public:
    /**
     * @brief Extends readCanFrame() to check destination of CAN Frame
     * 
     * @param timeout for ESP32Can.readFrame()
     * @return true upon reading a CAN frame for this MCU;
     * @return false upon not finding a CAN frame or reading one for a different MCU
     */
    bool readCan(int timeout = 0) {
        if (!ESP32Can.readFrame(inCanFrame, (timeout < 0 ? 0 : timeout)))
            return false;  // No CAN frame received

        inVicCanFrame.parseCanFrame(inCanFrame);  // Load data from CanFrame into VicCanFrame

        if (!inVicCanFrame.isForMe())
            return false;  // Not for this MCU

        return true;
    }

    inline uint8_t getCmdId() {
        return inVicCanFrame.cmdId;
    }

    inline CanDataType getDataType() {
        return inVicCanFrame.dataType;
    }

    int parseData(std::vector<float>& outData, CanDataType dt) {
        outData.clear();

        /**/ if (dt == CanDataType::DT_NA)
            return 0;
        else if (dt == CanDataType::DT_1i64) {
            uint64_t udata =
               (static_cast<uint64_t>(inVicCanFrame.data[0]) << 56) |
               (static_cast<uint64_t>(inVicCanFrame.data[1]) << 48) |
               (static_cast<uint64_t>(inVicCanFrame.data[2]) << 40) |
               (static_cast<uint64_t>(inVicCanFrame.data[3]) << 32) |
               (static_cast<uint64_t>(inVicCanFrame.data[4]) << 24) |
               (static_cast<uint64_t>(inVicCanFrame.data[5]) << 16) |
               (static_cast<uint64_t>(inVicCanFrame.data[6]) << 8) |
               (static_cast<uint64_t>(inVicCanFrame.data[7]) << 0);
            outData.push_back(static_cast<float>(*reinterpret_cast<int64_t*>(&udata)));  // lil messy but will fix later
            return 1;
        }
        // else if (dt == CanDataType::DT_2i32)
    }

    void relayFromSerial(std::vector<String> args) {
        CanFrame outCanFrame;

        // Command layout: "can_relay, id, rtr, dlc, data"

        if (args.size() < 3) {
            Serial.println("Invalid command");
            return;
        }

        outCanFrame.identifier = args[1].toInt();
        outCanFrame.rtr = args[2].toInt();
        outCanFrame.data_length_code = args[3].toInt();
        for (int i = 0; i < outCanFrame.data_length_code; i++) {
            outCanFrame.data[i] = args[i + 4].toInt();
        }

        ESP32Can.writeFrame(outCanFrame);
    }

    void readyOutCanFrame(uint8_t dlc, CanDataType outDataType) {
        outFrame.identifier = inVicCanFrame.createCanId(outDataType);
        outFrame.rtr = false;
        outFrame.data_length_code = dlc;
    }

    void respond(int64_t data) {
        readyOutCanFrame(8, CanDataType::DT_1i64);
        outFrame.data[0] = (data >> 56) & 0xFF;
        outFrame.data[1] = (data >> 48) & 0xFF;
        outFrame.data[2] = (data >> 40) & 0xFF;
        outFrame.data[3] = (data >> 32) & 0xFF;
        outFrame.data[4] = (data >> 24) & 0xFF;
        outFrame.data[5] = (data >> 16) & 0xFF;
        outFrame.data[6] = (data >> 8) & 0xFF;
        outFrame.data[7] = data & 0xFF;
        ESP32Can.writeFrame(outFrame);
    }
} vicCAN;

inline bool dtIsInt(CanDataType dt) {
    return dt == CanDataType::DT_1i64 || dt == CanDataType::DT_2i32 || dt == CanDataType::DT_4i16 ||
           dt == CanDataType::DT_8i8;
}