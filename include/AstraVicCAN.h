/**
 * @file AstraVicCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Easily communicate over ASTRA Vehicle CAN
 *
 */
#pragma once

#include <Arduino.h>

#include "AstraCAN.h"


enum class McuId : uint8_t {
    MCU_BROADCAST = 0,
    MCU_CORE,
    MCU_ARM,
    MCU_DIGIT,
    MCU_FAERIE,
    MCU_CITADEL
};

#if defined(CORE)
#define SUBMODULE_CAN_ID McuId::MCU_CORE
#elif defined(ARM)
#define SUBMODULE_CAN_ID McuId::MCU_ARM
#elif defined(DIGIT)
#define SUBMODULE_CAN_ID McuId::MCU_DIGIT
#elif defined(FAERIE)
#define SUBMODULE_CAN_ID McuId::MCU_FAERIE
#elif defined(CITADEL)
#define SUBMODULE_CAN_ID McuId::MCU_CITADEL
#else
#define SUBMODULE_CAN_ID McuId::MCU_BROADCAST
#endif

enum class DataParseType : uint8_t {
    DT_NA = 0,
    DT_1i64,
    DT_2i32,
    DT_4i16,
    DT_8i8,
    DT_2f32
};

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

class VicCanFrame {
   public:
    AstraCAN* Can0;
    // Within CAN ID
    McuId mcuId;  // 3 bits
    DataParseType dataType;  // 3 bits
    uint8_t cmdId;  // 5 bits
    // Built-in to CAN frame
    bool rtr;  // 1 bit
    uint8_t dlc;  // 4 bits
    uint8_t data[8];

    // Constructor
    VicCanFrame() {
        clear();
    }

    VicCanFrame(AstraCAN* CanObj) {
        Can0 = CanObj;
        clear();
    }

    // Resets all data values for use with static keyword
    void clear() {
        mcuId = McuId::MCU_BROADCAST;
        dataType = DataParseType::DT_NA;
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
        dataType = static_cast<DataParseType>((id >> 3) & 0x7);
        cmdId = (id >> 6);
    }

    void parseCanFrame(CanFrame& frame) {
        clear();
        parseCanId(frame.identifier);

        rtr = static_cast<bool>(frame.rtr);

        dlc = frame.data_length_code;
        for (int i = 0; i < dlc; i++) {
            data[i] = frame.data[i];
        }
    }

    bool isForMe() {
        return mcuId == SUBMODULE_CAN_ID || mcuId == McuId::MCU_BROADCAST;
    }

    bool readCan(int timeout = 0) {  // Extends readCanFrame()
        static CanFrame rxFrame;
        if (!Can0->readFrame(rxFrame, (timeout < 0 ? 0 : timeout)))
            return false;  // No CAN frame received
        
        parseCanFrame(rxFrame);  // Load data from CanFrame into VicCanFrame

        if (!isForMe())
            return false;  // Not for this MCU
        
        return true;
    }

    void respond(uint8_t outDataType) {
        // Create a CAN packet with stuff from recv'ed one
        CanFrame outFrame;
        // outFrame.identifier = createId(mcuId, outDataType, cmdId);
    }

    bool isBuiltin() {
        if (cmdId == 1 || cmdId == 2 || cmdId == 3)
            return true;
        else
            return false;
    }

    void handleBuiltin() {
        if (!isBuiltin())
            return;
        
        if (cmdId == 1) {  // Ping
            // Respond with ping
        } else if (cmdId == 2) {  // Time
            // Respond with millis
        } else if (cmdId == 3) {  // LED ctrl
            // if data is 0, turn off
            // if data is 1, turn on
        }
    }
};
