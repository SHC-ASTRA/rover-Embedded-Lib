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

    /**
     * @brief Extends readCanFrame() to check destination of CAN Frame
     * 
     * @param timeout for ESP32Can.readFrame()
     * @return true upon reading a CAN frame for this MCU;
     * @return false upon not finding a CAN frame or reading one for a different MCU
     */
    bool readCan(int timeout = 0) {
        if (!ESP32Can.readFrame(inCanFrame, (timeout < 0 ? 0 : timeout)))  // No negative timeouts
            return false;  // No CAN frame received

        inVicCanFrame.parseCanFrame(inCanFrame);  // Load data from CanFrame into VicCanFrame

        if (!inVicCanFrame.isForMe())
            return false;  // Not for this MCU

        return true;
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

    void respond(uint8_t outDataType) {
        // Create a CAN packet with stuff from recv'ed one
        CanFrame outFrame;
        // outFrame.identifier = createId(mcuId, outDataType, cmdId);
    }
};
