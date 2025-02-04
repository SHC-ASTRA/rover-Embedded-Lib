/**
 * @file AstraVicCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Easily communicate over ASTRA Vehicle CAN
 *
 */
#pragma once

#include <Arduino.h>

#include "AstraCAN.h"
#include <vector>


// Microcontroller VicCAN ID's based on submodule; use these instead of the raw numbers
enum class McuId : uint8_t {
    MCU_BROADCAST = 0,
    MCU_CORE,
    MCU_ARM,
    MCU_DIGIT,
    MCU_FAERIE,
    MCU_CITADEL
};

// Shorthand for the VicCAN ID used by the active MCU
#if defined(CORE)
#    define SUBMODULE_CAN_ID McuId::MCU_CORE
#elif defined(ARM)
#    define SUBMODULE_CAN_ID McuId::MCU_ARM
#elif defined(DIGIT)
#    define SUBMODULE_CAN_ID McuId::MCU_DIGIT
#elif defined(FAERIE)
#    define SUBMODULE_CAN_ID McuId::MCU_FAERIE
#elif defined(CITADEL)
#    define SUBMODULE_CAN_ID McuId::MCU_CITADEL
#else  // Default - no macro in platformio.ini; will only respond to broadcast messages
#    define SUBMODULE_CAN_ID McuId::MCU_BROADCAST
#endif

// Possible datatypes for a VicCAN frame; decides how to decode/encode data
enum class CanDataType : uint8_t {
    DT_1f64 = 0,
    DT_2f32,
    DT_4i16,
    DT_8i8
};

// Command IDs for standard VicCAN commands
typedef enum CanCmdId : uint8_t {
    // General misc
    CMD_TEST = 0,
    CMD_PING,
    CMD_B_LED,
    CMD_SENSOR_RECON,
    // REV Motor control
    CMD_REV_STOP = 16,
    CMD_REV_IDENTIFY,
    CMD_REV_IDLE_MODE,
    CMD_REV_SET_DUTY,
    // Misc physical control
    CMD_LSS_TURNBY_DEG = 24,
    CMD_PWMSERVO_SET_DEG,
    CMD_DCMOTOR_SET_SPEED,
    // Submodule-specific (omitted)
    // Data request
    CMD_GNSS_LAT = 48,
    CMD_GNSS_LON,
    CMD_GNSS_SAT,
    CMD_DATA_TEMP,
    CMD_DATA_IMU
};


//------------------------------------------------------------------------------------------------//
//  VicCanFrame  -  Contains specific data used to create a CanFrame for inter-MCU comms
//------------------------------------------------------------------------------------------------//

class VicCanFrame {
   public:
    // Within CAN ID
    McuId mcuId;           // 3 bits
    CanDataType dataType;  // 2 bits
    uint8_t cmdId;         // 6 bits
    // Built-in to CAN frame
    bool rtr;         // 1 bit
    uint8_t dlc;      // 4 bits
    uint8_t data[8];  // 0..8 bytes

    // Constructor
    VicCanFrame() {
        clear();
    }

    // Resets all data values; for use with static keyword
    void clear() {
        mcuId = McuId::MCU_BROADCAST;
        dataType = CanDataType::DT_1f64;
        cmdId = 0;
        rtr = false;
        dlc = 0;
        for (int i = 0; i < 8; i++) {
            data[i] = 0;
        }
    }

    // Take a CAN ID and parse it into its components
    void parseCanId(uint32_t id) {
        mcuId = static_cast<McuId>((id >> 8) & 0x7);
        dataType = static_cast<CanDataType>((id >> 6) & 0x3);
        cmdId = id & 0x3F;
    }

    inline int createCanId() {
        return createCanId(dataType);
    }

    int createCanId(CanDataType pDataType) {
        return (static_cast<uint8_t>(mcuId) << 8) | (static_cast<uint8_t>(pDataType) << 6) | cmdId;
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
        return mcuId == SUBMODULE_CAN_ID || mcuId == McuId::MCU_BROADCAST;
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

    void parseData(std::vector<double>& outData) {
        outData.clear();

        // The idea behind this method of encoding data into a can frame is that there is no
        // type noercion--the data is stored bit-by-bit in data[8] as if it was one cohesive
        // 64-bit memory address. re-interpret_cast is used to accomplish this as C++ is
        // not going to mess with an unsigned int.

        /**/ if (inVicCanFrame.dlc == 0) {
            // No data to parse
        }
        else if (inVicCanFrame.dataType == CanDataType::DT_1f64) {
            uint64_t udata =
               (static_cast<uint64_t>(inVicCanFrame.data[0]) << 56) |
               (static_cast<uint64_t>(inVicCanFrame.data[1]) << 48) |
               (static_cast<uint64_t>(inVicCanFrame.data[2]) << 40) |
               (static_cast<uint64_t>(inVicCanFrame.data[3]) << 32) |
               (static_cast<uint64_t>(inVicCanFrame.data[4]) << 24) |
               (static_cast<uint64_t>(inVicCanFrame.data[5]) << 16) |
               (static_cast<uint64_t>(inVicCanFrame.data[6]) << 8) |
               (static_cast<uint64_t>(inVicCanFrame.data[7]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<double*>(&udata)));  // shut up, ik
        }
        else if (inVicCanFrame.dataType == CanDataType::DT_2f32) {
            uint32_t udata =
               (static_cast<uint32_t>(inVicCanFrame.data[0]) << 24) |
               (static_cast<uint32_t>(inVicCanFrame.data[1]) << 16) |
               (static_cast<uint32_t>(inVicCanFrame.data[2]) << 8) |
               (static_cast<uint32_t>(inVicCanFrame.data[3]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<float*>(&udata)));
            udata =
               (static_cast<uint32_t>(inVicCanFrame.data[4]) << 24) |
               (static_cast<uint32_t>(inVicCanFrame.data[5]) << 16) |
               (static_cast<uint32_t>(inVicCanFrame.data[6]) << 8) |
               (static_cast<uint32_t>(inVicCanFrame.data[7]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<float*>(&udata)));
        }
        else if (inVicCanFrame.dataType == CanDataType::DT_4i16) {
            uint16_t udata =
               (static_cast<uint16_t>(inVicCanFrame.data[0]) << 8) |
               (static_cast<uint16_t>(inVicCanFrame.data[1]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<int16_t*>(&udata)));
            udata =
               (static_cast<uint16_t>(inVicCanFrame.data[2]) << 8) |
               (static_cast<uint16_t>(inVicCanFrame.data[3]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<int16_t*>(&udata)));
            udata =
               (static_cast<uint16_t>(inVicCanFrame.data[4]) << 8) |
               (static_cast<uint16_t>(inVicCanFrame.data[5]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<int16_t*>(&udata)));
            udata =
               (static_cast<uint16_t>(inVicCanFrame.data[6]) << 8) |
               (static_cast<uint16_t>(inVicCanFrame.data[7]) << 0);
            outData.push_back(static_cast<double>(*reinterpret_cast<int16_t*>(&udata)));
        }
        else if (inVicCanFrame.dataType == CanDataType::DT_8i8) {
            for (int i = 0; i < 8; i++) {
                outData.push_back(static_cast<double>(inVicCanFrame.data[i]));
            }
        }
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


    // DT_1f64
    void encodeData(uint8_t canData[8], double data1) {
        uint64_t udata = *reinterpret_cast<uint64_t*>(&data1);
        canData[0] = (udata >> 56) & 0xFF;
        canData[1] = (udata >> 48) & 0xFF;
        canData[2] = (udata >> 40) & 0xFF;
        canData[3] = (udata >> 32) & 0xFF;
        canData[4] = (udata >> 24) & 0xFF;
        canData[5] = (udata >> 16) & 0xFF;
        canData[6] = (udata >> 8) & 0xFF;
        canData[7] = udata & 0xFF;
    }

    // DT_2f32
    void encodeData(uint8_t canData[8], float data1, float data2) {
        uint32_t udata = *reinterpret_cast<uint32_t*>(&data1);
        canData[0] = (udata >> 24) & 0xFF;
        canData[1] = (udata >> 16) & 0xFF;
        canData[2] = (udata >> 8) & 0xFF;
        canData[3] = udata & 0xFF;
        udata = *reinterpret_cast<uint32_t*>(&data2);
        canData[4] = (udata >> 24) & 0xFF;
        canData[5] = (udata >> 16) & 0xFF;
        canData[6] = (udata >> 8) & 0xFF;
        canData[7] = udata & 0xFF;
    }

    // DT_4i16
    void encodeData(uint8_t canData[8], int16_t data1, int16_t data2, int16_t data3, int16_t data4) {
        uint16_t udata = *reinterpret_cast<uint16_t*>(&data1);
        canData[0] = (udata >> 8) & 0xFF;
        canData[1] = udata & 0xFF;
        udata = *reinterpret_cast<uint16_t*>(&data2);
        canData[2] = (udata >> 8) & 0xFF;
        canData[3] = udata & 0xFF;
        udata = *reinterpret_cast<uint16_t*>(&data3);
        canData[4] = (udata >> 8) & 0xFF;
        canData[5] = udata & 0xFF;
        udata = *reinterpret_cast<uint16_t*>(&data4);
        canData[6] = (udata >> 8) & 0xFF;
        canData[7] = udata & 0xFF;
    }

    // DT_8i8
    void encodeData(uint8_t canData[8], int8_t data1, int8_t data2, int8_t data3, int8_t data4, int8_t data5,
                    int8_t data6, int8_t data7, int8_t data8) {
        canData[0] = *reinterpret_cast<uint8_t*>(&data1);
        canData[1] = *reinterpret_cast<uint8_t*>(&data2);
        canData[2] = *reinterpret_cast<uint8_t*>(&data3);
        canData[3] = *reinterpret_cast<uint8_t*>(&data4);
        canData[4] = *reinterpret_cast<uint8_t*>(&data5);
        canData[5] = *reinterpret_cast<uint8_t*>(&data6);
        canData[6] = *reinterpret_cast<uint8_t*>(&data7);
        canData[7] = *reinterpret_cast<uint8_t*>(&data8);
    }

    //------------//
    // Responding //
    //------------//

    void readyTxFrame(uint8_t dlc, CanDataType outDataType, uint8_t cmdId) {
        outFrame.identifier = inVicCanFrame.createCanId(outDataType);
        outFrame.rtr = false;
        outFrame.data_length_code = dlc;
    }


    void send(uint8_t cmdId, double data) {
        readyTxFrame(8, CanDataType::DT_1f64, cmdId);
        encodeData(outFrame.data, data);
        ESP32Can.writeFrame(outFrame);
    }

    void send(uint8_t cmdId, float data1, float data2) {
        readyTxFrame(8, CanDataType::DT_2f32, cmdId);
        encodeData(outFrame.data, data1, data2);
        ESP32Can.writeFrame(outFrame);
    }

    void send(uint8_t cmdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 = 0) {
        readyTxFrame(8, CanDataType::DT_4i16, cmdId);
        encodeData(outFrame.data, data1, data2, data3, data4);
        ESP32Can.writeFrame(outFrame);
    }

    void send(uint8_t cmdId, int8_t data1, int8_t data2, int8_t data3, int8_t data4, int8_t data5, int8_t data6 = 0,
              int8_t data7 = 0, int8_t data8 = 0) {
        readyTxFrame(8, CanDataType::DT_8i8, cmdId);
        encodeData(outFrame.data, data1, data2, data3, data4, data5, data6, data7, data8);
        ESP32Can.writeFrame(outFrame);
    }


    inline void respond(double data) {
        send(inVicCanFrame.cmdId, data);
    }

    inline void respond(float data1, float data2) {
        send(inVicCanFrame.cmdId, data1, data2);
    }

    inline void respond(int16_t data1, int16_t data2, int16_t data3, int16_t data4 = 0) {
        send(inVicCanFrame.cmdId, data1, data2, data3, data4);
    }

    inline void respond(int8_t data1, int8_t data2, int8_t data3, int8_t data4, int8_t data5,
                        int8_t data6 = 0, int8_t data7 = 0, int8_t data8 = 0) {
        send(inVicCanFrame.cmdId, data1, data2, data3, data4, data5, data6, data7, data8);
    }
} vicCAN;
