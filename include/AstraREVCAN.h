/**
 * @file AstraREVCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA's utilities for interfacing with the REV Sparkmax over CAN
 *
 */
#pragma once

#ifndef OLD_ASTRACAN_ENABLE

#include <Arduino.h>  // For fixed-size integer type definitions


// Must have the CAN library corresponding to the current MCU
#if defined(CORE_TEENSY) && !__has_include("FlexCAN_T4.h")  // If on Teensy, must have FlexCAN_T4
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  https://github.com/tonton81/FlexCAN_T4

#elif defined(ESP32) && \
    !__has_include("ESP32-TWAI-CAN.hpp")  // If on ESP32, must have ESP32-TWAI-CAN
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  handmade0octopus/ESP32-TWAI-CAN@^1.0.1

#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#    error "Raspberry Pi Pico is not supported"

#else  // We have the required library.


//--------------------------------------------------------------------------//
//   Microcontroller-specific                                               //
//--------------------------------------------------------------------------//

#    if defined(ESP32)

//---------//
//  ESP32  //
//---------//

#        include <ESP32-TWAI-CAN.hpp>  // handmade0octopus/ESP32-TWAI-CAN

typedef TwaiCAN AstraCAN;


#    elif defined(CORE_TEENSY)

//----------//
//  Teensy  //
//----------//

#        include <FlexCAN_T4.h>  // https://github.com/tonton81/FlexCAN_T4

// Core and FAERIE use CAN1, Arm uses CAN3
#        ifdef ARM
typedef FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AstraCAN;
#        else
typedef FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> AstraCAN;
#        endif


#    endif  // End microcontroller check


//--------------------------------------------------------------------------//
//   REV Enums                                                              //
//--------------------------------------------------------------------------//

enum class sparkMax_ParameterType : uint8_t {
    kInt32 = 0x0,
    kUint32 = 0x1,
    kFloat32 = 0x2,
    kBool = 0x3,
};

enum class sparkMax_ConfigParameter : int32_t {
    kCanID = 0x0,
    kInputMode = 0x1,
    kMotorType = 0x2,
    kCommAdvance = 0x3,
    kSensorType = 0x4,
    kCtrlType = 0x5,
    kIdleMode = 0x6,
    kInputDeadband = 0x7,
    kFeedbackSensorPID0 = 0x8,
    kFeedbackSensorPID1 = 0x9,
    kPolePairs = 0xA,
    kCurrentChop = 0xB,
    kCurrentChopCycles = 0xC,
    kP_0 = 0xD,
    kI_0 = 0xE,
    kD_0 = 0xF,
    kF_0 = 0x10,
    kIZone_0 = 0x11,
    kDFilter_0 = 0x12,
    kOutputMin_0 = 0x13,
    kOutputMax_0 = 0x14,
    kP_1 = 0x15,
    kI_1 = 0x16,
    kD_1 = 0x17,
    kF_1 = 0x18,
    kIZone_1 = 0x19,
    kDFilter_1 = 0x1A,
    kOutputMin_1 = 0x1B,
    kOutputMax_1 = 0x1C,
    kP_2 = 0x1D,
    kI_2 = 0x1E,
    kD_2 = 0x1F,
    kF_2 = 0x20,
    kIZone_2 = 0x21,
    kDFilter_2 = 0x22,
    kOutputMin_2 = 0x23,
    kOutputMax_2 = 0x24,
    kP_3 = 0x25,
    kI_3 = 0x26,
    kD_3 = 0x27,
    kF_3 = 0x28,
    kIZone_3 = 0x29,
    kDFilter_3 = 0x2A,
    kOutputMin_3 = 0x2B,
    kOutputMax_3 = 0x2C,
    kInverted = 0x2D,
    kOutputRatio = 0x2E,
    kSerialNumberLow = 0x2F,
    kSerialNumberMid = 0x30,
    kSerialNumberHigh = 0x31,
    kLimitSwitchFwdPolarity = 0x32,
    kLimitSwitchRevPolarity = 0x33,
    kHardLimitFwdEn = 0x34,
    kHardLimitRevEn = 0x35,
    kSoftLimitFwdEn = 0x36,
    kSoftLimitRevEn = 0x37,
    kRampRate = 0x38,
    kFollowerID = 0x39,
    kFollowerConfig = 0x3A,
    kSmartCurrentStallLimit = 0x3B,
    kSmartCurrentFreeLimit = 0x3C,
    kSmartCurrentConfig = 0x3D,
    kSmartCurrentReserved = 0x3E,
    kMotorKv = 0x3F,
    kMotorR = 0x40,
    kMotorL = 0x41,
    kMotorRsvd1 = 0x42,
    kMotorRsvd2 = 0x43,
    kMotorRsvd3 = 0x44,
    kEncoderCountsPerRev = 0x45,
    kEncoderAverageDepth = 0x46,
    kEncoderSampleDelta = 0x47,
    kEncoderInverted = 0x48,
    kEncoderRsvd1 = 0x49,
    kVoltageCompMode = 0x4A,
    kCompensatedNominalVoltage = 0x4B,
    kSmartMotionMaxVelocity_0 = 0x4C,
    kSmartMotionMaxAccel_0 = 0x4D,
    kSmartMotionMinVelOutput_0 = 0x4E,
    kSmartMotionAllowedClosedLoopError_0 = 0x4F,
    kSmartMotionAccelStrategy_0 = 0x50,
    kSmartMotionMaxVelocity_1 = 0x51,
    kSmartMotionMaxAccel_1 = 0x52,
    kSmartMotionMinVelOutput_1 = 0x53,
    kSmartMotionAllowedClosedLoopError_1 = 0x54,
    kSmartMotionAccelStrategy_1 = 0x55,
    kSmartMotionMaxVelocity_2 = 0x56,
    kSmartMotionMaxAccel_2 = 0x57,
    kSmartMotionMinVelOutput_2 = 0x58,
    kSmartMotionAllowedClosedLoopError_2 = 0x59,
    kSmartMotionAccelStrategy_2 = 0x5A,
    kSmartMotionMaxVelocity_3 = 0x5B,
    kSmartMotionMaxAccel_3 = 0x5C,
    kSmartMotionMinVelOutput_3 = 0x5D,
    kSmartMotionAllowedClosedLoopError_3 = 0x5E,
    kSmartMotionAccelStrategy_3 = 0x5F,
    kIMaxAccum_0 = 0x60,
    kSlot3Placeholder1_0 = 0x61,
    kSlot3Placeholder2_0 = 0x62,
    kSlot3Placeholder3_0 = 0x63,
    kIMaxAccum_1 = 0x64,
    kSlot3Placeholder1_1 = 0x65,
    kSlot3Placeholder2_1 = 0x66,
    kSlot3Placeholder3_1 = 0x67,
    kIMaxAccum_2 = 0x68,
    kSlot3Placeholder1_2 = 0x69,
    kSlot3Placeholder2_2 = 0x6A,
    kSlot3Placeholder3_2 = 0x6B,
    kIMaxAccum_3 = 0x6C,
    kSlot3Placeholder1_3 = 0x6D,
    kSlot3Placeholder2_3 = 0x6E,
    kSlot3Placeholder3_3 = 0x6F,
    kPositionConversionFactor = 0x70,
    kVelocityConversionFactor = 0x71,
    kClosedLoopRampRate = 0x72,
    kSoftLimitFwd = 0x73,
    kSoftLimitRev = 0x74,
    kSoftLimitRsvd0 = 0x75,
    kSoftLimitRsvd1 = 0x76,
    kAnalogRevPerVolt = 0x77,
    kAnalogRotationsPerVoltSec = 0x78,
    kAnalogAverageDepth = 0x79,
    kAnalogSensorMode = 0x7A,
    kAnalogInverted = 0x7B,
    kAnalogSampleDelta = 0x7C,
    kAnalogRsvd0 = 0x7D,
    kAnalogRsvd1 = 0x7E,
    kDataPortConfig = 0x7F,
    kAltEncoderCountsPerRev = 0x80,
    kAltEncoderAverageDepth = 0x81,
    kAltEncoderSampleDelta = 0x82,
    kAltEncoderInverted = 0x83,
    kAltEncodePositionFactor = 0x84,
    kAltEncoderVelocityFactor = 0x85,
    kAltEncoderRsvd0 = 0x86,
    kAltEncoderRsvd1 = 0x87,
    kHallSensorSampleRate = 0x88,
    kHallSensorAverageDepth = 0x89,
    kNumParameters = 0x8A,
    kDutyCyclePositionFactor = 0x8B,
    kDutyCycleVelocityFactor = 0x8C,
    kDutyCycleInverted = 0x8D,
    kDutyCycleSensorMode = 0x8E,
    kDutyCycleAverageDepth = 0x8F,
    kDutyCycleSampleDelta = 0x90,
    kDutyCycleOffsetv1p6p2 = 0x91,
    kDutyCycleRsvd0 = 0x92,
    kDutyCycleRsvd1 = 0x93,
    kDutyCycleRsvd2 = 0x94,
    kPositionPIDWrapEnable = 0x95,
    kPositionPIDMinInput = 0x96,
    kPositionPIDMaxInput = 0x97,
    kDutyCycleZeroCentered = 0x98,
    kDutyCyclePrescaler = 0x99,
    kDutyCycleOffset = 0x9A,
    kProductId = 0x9B,
    kDeviceMajorVersion = 0x9C,
    kDeviceMinorVersion = 0x9D,
    NumParameters = 0x9E,
};

enum class ParameterStatus {
    kOK = 0,
    kInvalidID = 1,
    kMismatchType = 2,
    kAccessMode = 3,
    kInvalid = 4,
    kNotImplementedDeprecated = 5,
};

enum class sparkMax_IdleMode { kCoast = 0, kBrake = 1 };

enum class sparkMax_faultID {
    kBrownout = 0,
    kOvercurrent = 1,
    kIWDTReset = 2,
    kMotorFault = 3,
    kSensorFault = 4,
    kStall = 5,
    kEEPROMCRC = 6,
    kCANTX = 7,
    kCANRX = 8,
    kHasReset = 9,
    kDRVFault = 10,
    kOtherFault = 11,
    kSoftLimitFwd = 12,
    kSoftLimitRev = 13,
    kHardLimitFwd = 14,
    kHardLimitRev = 15
};

enum class sparkMax_ctrlType {
    kDutyCycle = 0,
    kVelocity = 1,
    kVoltage = 2,
    kPosition = 3,
    kSmartMotion = 4,
    kCurrent = 5,
    kSmartVelocity = 6
};

struct motorStatus1 {
    float sensorVelocity;
    uint8_t motorTemperature;
    float busVoltage;
    float outputCurrent;
    uint64_t timestamp;
};

struct motorStatus2 {
    float sensorPosition;
    uint64_t timestamp;
};


//--------------------------------------------------------------------------//
//   Specific REV Commands                                                  //
//--------------------------------------------------------------------------//

void CAN_enumerate(AstraCAN& Can0);


void CAN_sendDutyCycle(uint8_t deviceId, float dutyCycle, AstraCAN& Can0);

void CAN_sendVelocity(uint8_t deviceId, float speed, AstraCAN& Can0);

void CAN_sendSmartVelocity(uint8_t deviceId, float speed, AstraCAN& Can0);

void CAN_sendPosition(uint8_t deviceId, float position, AstraCAN& Can0);


void CAN_sendHeartbeat(uint8_t deviceId, AstraCAN& Can0);

void CAN_identifySparkMax(uint8_t deviceId, AstraCAN& Can0);


void CAN_setParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID,
                      sparkMax_ParameterType type, uint32_t value, AstraCAN& Can0);

void CAN_reqParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID, AstraCAN& Can0);


//-------------------------------------//
//  Backwards Compatibility Functions  //
//-------------------------------------//

inline void identifyDevice(AstraCAN &Can0, int can_id) {
    CAN_identifySparkMax(can_id, Can0);
}

inline void sendDutyCycle(AstraCAN &Can0, int can_id, float duty_cycle) {
    CAN_sendDutyCycle(can_id, duty_cycle, Can0);
}

inline void sendHeartbeat(AstraCAN &Can0, int can_id) {
    CAN_sendHeartbeat(can_id, Can0);
}

inline void setParameter(AstraCAN &Can0, int can_id, uint8_t paramID, uint32_t value) {
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
void CAN_sendPacket(uint8_t deviceId, int32_t apiId, uint8_t data[], uint8_t dataLen,
                    AstraCAN& Can0);

// Given direct values for the CAN packet
// In AstraREVCAN.cpp, this is defined differently depending on mcu.
void CAN_sendPacket(uint32_t messageID, uint8_t data[], uint8_t dataLen, AstraCAN& Can0);

void printREVFrame(CanFrame frame);


#endif  // End library check

#endif
