/**
 * @file AstraREVCAN.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief CAN functions for the Sparkmax from REV's library
 *
 */
#pragma once

#include <Arduino.h>  // For fixed-size integer type definitions

#include "AstraCAN.h"


//--------------------------------------------------------------------------//
//   Other microcontrollers                                                 //
//--------------------------------------------------------------------------//

// Fallback to legacy Teensy CAN code if running on a Teensy
#if defined(CORE_TEENSY)
#    include "AstraCAN.h"

// Pico is not supported
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#    error "Raspberry Pi Pico is not supported"


#elif defined(ESP32) && \
    !__has_include("ESP32-TWAI-CAN.hpp")  // If on ESP32, must have ESP32-TWAI-CAN
#    error Missing library! Please add the following line to lib_deps in platformio.ini:  handmade0octopus/ESP32-TWAI-CAN@^1.0.1

#elif defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")  // We have the required library.

//--------------------------------------------------------------------------//
//   ESP32                                                                  //
//--------------------------------------------------------------------------//
#    include "ESP32-TWAI-CAN.hpp"                            // handmade0octopus/ESP32-TWAI-CAN

//--------------------------------------------------------------------------//
//   REV Enums                                                              //
//--------------------------------------------------------------------------//

enum class c_SparkMax_ParameterType : int32_t {
    c_SparkMax_kInt32 = 0x0,
    c_SparkMax_kUint32 = 0x1,
    c_SparkMax_kFloat32 = 0x2,
    c_SparkMax_kBool = 0x3,
};

enum class c_SparkMax_ConfigParameter : int32_t {
    c_SparkMax_kCanID = 0x0,
    c_SparkMax_kInputMode = 0x1,
    c_SparkMax_kMotorType = 0x2,
    c_SparkMax_kCommAdvance = 0x3,
    c_SparkMax_kSensorType = 0x4,
    c_SparkMax_kCtrlType = 0x5,
    c_SparkMax_kIdleMode = 0x6,
    c_SparkMax_kInputDeadband = 0x7,
    c_SparkMax_kFeedbackSensorPID0 = 0x8,
    c_SparkMax_kFeedbackSensorPID1 = 0x9,
    c_SparkMax_kPolePairs = 0xA,
    c_SparkMax_kCurrentChop = 0xB,
    c_SparkMax_kCurrentChopCycles = 0xC,
    c_SparkMax_kP_0 = 0xD,
    c_SparkMax_kI_0 = 0xE,
    c_SparkMax_kD_0 = 0xF,
    c_SparkMax_kF_0 = 0x10,
    c_SparkMax_kIZone_0 = 0x11,
    c_SparkMax_kDFilter_0 = 0x12,
    c_SparkMax_kOutputMin_0 = 0x13,
    c_SparkMax_kOutputMax_0 = 0x14,
    c_SparkMax_kP_1 = 0x15,
    c_SparkMax_kI_1 = 0x16,
    c_SparkMax_kD_1 = 0x17,
    c_SparkMax_kF_1 = 0x18,
    c_SparkMax_kIZone_1 = 0x19,
    c_SparkMax_kDFilter_1 = 0x1A,
    c_SparkMax_kOutputMin_1 = 0x1B,
    c_SparkMax_kOutputMax_1 = 0x1C,
    c_SparkMax_kP_2 = 0x1D,
    c_SparkMax_kI_2 = 0x1E,
    c_SparkMax_kD_2 = 0x1F,
    c_SparkMax_kF_2 = 0x20,
    c_SparkMax_kIZone_2 = 0x21,
    c_SparkMax_kDFilter_2 = 0x22,
    c_SparkMax_kOutputMin_2 = 0x23,
    c_SparkMax_kOutputMax_2 = 0x24,
    c_SparkMax_kP_3 = 0x25,
    c_SparkMax_kI_3 = 0x26,
    c_SparkMax_kD_3 = 0x27,
    c_SparkMax_kF_3 = 0x28,
    c_SparkMax_kIZone_3 = 0x29,
    c_SparkMax_kDFilter_3 = 0x2A,
    c_SparkMax_kOutputMin_3 = 0x2B,
    c_SparkMax_kOutputMax_3 = 0x2C,
    c_SparkMax_kInverted = 0x2D,
    c_SparkMax_kOutputRatio = 0x2E,
    c_SparkMax_kSerialNumberLow = 0x2F,
    c_SparkMax_kSerialNumberMid = 0x30,
    c_SparkMax_kSerialNumberHigh = 0x31,
    c_SparkMax_kLimitSwitchFwdPolarity = 0x32,
    c_SparkMax_kLimitSwitchRevPolarity = 0x33,
    c_SparkMax_kHardLimitFwdEn = 0x34,
    c_SparkMax_kHardLimitRevEn = 0x35,
    c_SparkMax_kSoftLimitFwdEn = 0x36,
    c_SparkMax_kSoftLimitRevEn = 0x37,
    c_SparkMax_kRampRate = 0x38,
    c_SparkMax_kFollowerID = 0x39,
    c_SparkMax_kFollowerConfig = 0x3A,
    c_SparkMax_kSmartCurrentStallLimit = 0x3B,
    c_SparkMax_kSmartCurrentFreeLimit = 0x3C,
    c_SparkMax_kSmartCurrentConfig = 0x3D,
    c_SparkMax_kSmartCurrentReserved = 0x3E,
    c_SparkMax_kMotorKv = 0x3F,
    c_SparkMax_kMotorR = 0x40,
    c_SparkMax_kMotorL = 0x41,
    c_SparkMax_kMotorRsvd1 = 0x42,
    c_SparkMax_kMotorRsvd2 = 0x43,
    c_SparkMax_kMotorRsvd3 = 0x44,
    c_SparkMax_kEncoderCountsPerRev = 0x45,
    c_SparkMax_kEncoderAverageDepth = 0x46,
    c_SparkMax_kEncoderSampleDelta = 0x47,
    c_SparkMax_kEncoderInverted = 0x48,
    c_SparkMax_kEncoderRsvd1 = 0x49,
    c_SparkMax_kVoltageCompMode = 0x4A,
    c_SparkMax_kCompensatedNominalVoltage = 0x4B,
    c_SparkMax_kSmartMotionMaxVelocity_0 = 0x4C,
    c_SparkMax_kSmartMotionMaxAccel_0 = 0x4D,
    c_SparkMax_kSmartMotionMinVelOutput_0 = 0x4E,
    c_SparkMax_kSmartMotionAllowedClosedLoopError_0 = 0x4F,
    c_SparkMax_kSmartMotionAccelStrategy_0 = 0x50,
    c_SparkMax_kSmartMotionMaxVelocity_1 = 0x51,
    c_SparkMax_kSmartMotionMaxAccel_1 = 0x52,
    c_SparkMax_kSmartMotionMinVelOutput_1 = 0x53,
    c_SparkMax_kSmartMotionAllowedClosedLoopError_1 = 0x54,
    c_SparkMax_kSmartMotionAccelStrategy_1 = 0x55,
    c_SparkMax_kSmartMotionMaxVelocity_2 = 0x56,
    c_SparkMax_kSmartMotionMaxAccel_2 = 0x57,
    c_SparkMax_kSmartMotionMinVelOutput_2 = 0x58,
    c_SparkMax_kSmartMotionAllowedClosedLoopError_2 = 0x59,
    c_SparkMax_kSmartMotionAccelStrategy_2 = 0x5A,
    c_SparkMax_kSmartMotionMaxVelocity_3 = 0x5B,
    c_SparkMax_kSmartMotionMaxAccel_3 = 0x5C,
    c_SparkMax_kSmartMotionMinVelOutput_3 = 0x5D,
    c_SparkMax_kSmartMotionAllowedClosedLoopError_3 = 0x5E,
    c_SparkMax_kSmartMotionAccelStrategy_3 = 0x5F,
    c_SparkMax_kIMaxAccum_0 = 0x60,
    c_SparkMax_kSlot3Placeholder1_0 = 0x61,
    c_SparkMax_kSlot3Placeholder2_0 = 0x62,
    c_SparkMax_kSlot3Placeholder3_0 = 0x63,
    c_SparkMax_kIMaxAccum_1 = 0x64,
    c_SparkMax_kSlot3Placeholder1_1 = 0x65,
    c_SparkMax_kSlot3Placeholder2_1 = 0x66,
    c_SparkMax_kSlot3Placeholder3_1 = 0x67,
    c_SparkMax_kIMaxAccum_2 = 0x68,
    c_SparkMax_kSlot3Placeholder1_2 = 0x69,
    c_SparkMax_kSlot3Placeholder2_2 = 0x6A,
    c_SparkMax_kSlot3Placeholder3_2 = 0x6B,
    c_SparkMax_kIMaxAccum_3 = 0x6C,
    c_SparkMax_kSlot3Placeholder1_3 = 0x6D,
    c_SparkMax_kSlot3Placeholder2_3 = 0x6E,
    c_SparkMax_kSlot3Placeholder3_3 = 0x6F,
    c_SparkMax_kPositionConversionFactor = 0x70,
    c_SparkMax_kVelocityConversionFactor = 0x71,
    c_SparkMax_kClosedLoopRampRate = 0x72,
    c_SparkMax_kSoftLimitFwd = 0x73,
    c_SparkMax_kSoftLimitRev = 0x74,
    c_SparkMax_kSoftLimitRsvd0 = 0x75,
    c_SparkMax_kSoftLimitRsvd1 = 0x76,
    c_SparkMax_kAnalogRevPerVolt = 0x77,
    c_SparkMax_kAnalogRotationsPerVoltSec = 0x78,
    c_SparkMax_kAnalogAverageDepth = 0x79,
    c_SparkMax_kAnalogSensorMode = 0x7A,
    c_SparkMax_kAnalogInverted = 0x7B,
    c_SparkMax_kAnalogSampleDelta = 0x7C,
    c_SparkMax_kAnalogRsvd0 = 0x7D,
    c_SparkMax_kAnalogRsvd1 = 0x7E,
    c_SparkMax_kDataPortConfig = 0x7F,
    c_SparkMax_kAltEncoderCountsPerRev = 0x80,
    c_SparkMax_kAltEncoderAverageDepth = 0x81,
    c_SparkMax_kAltEncoderSampleDelta = 0x82,
    c_SparkMax_kAltEncoderInverted = 0x83,
    c_SparkMax_kAltEncodePositionFactor = 0x84,
    c_SparkMax_kAltEncoderVelocityFactor = 0x85,
    c_SparkMax_kAltEncoderRsvd0 = 0x86,
    c_SparkMax_kAltEncoderRsvd1 = 0x87,
    c_SparkMax_kHallSensorSampleRate = 0x88,
    c_SparkMax_kHallSensorAverageDepth = 0x89,
    c_SparkMax_kNumParameters = 0x8A,
    c_SparkMax_kDutyCyclePositionFactor = 0x8B,
    c_SparkMax_kDutyCycleVelocityFactor = 0x8C,
    c_SparkMax_kDutyCycleInverted = 0x8D,
    c_SparkMax_kDutyCycleSensorMode = 0x8E,
    c_SparkMax_kDutyCycleAverageDepth = 0x8F,
    c_SparkMax_kDutyCycleSampleDelta = 0x90,
    c_SparkMax_kDutyCycleOffsetv1p6p2 = 0x91,
    c_SparkMax_kDutyCycleRsvd0 = 0x92,
    c_SparkMax_kDutyCycleRsvd1 = 0x93,
    c_SparkMax_kDutyCycleRsvd2 = 0x94,
    c_SparkMax_kPositionPIDWrapEnable = 0x95,
    c_SparkMax_kPositionPIDMinInput = 0x96,
    c_SparkMax_kPositionPIDMaxInput = 0x97,
    c_SparkMax_kDutyCycleZeroCentered = 0x98,
    c_SparkMax_kDutyCyclePrescaler = 0x99,
    c_SparkMax_kDutyCycleOffset = 0x9A,
    c_SparkMax_kProductId = 0x9B,
    c_SparkMax_kDeviceMajorVersion = 0x9C,
    c_SparkMax_kDeviceMinorVersion = 0x9D,
    c_SparkMax_NumParameters = 0x9E,
};

enum class IdleMode { kCoast = 0, kBrake = 1 };

enum class FaultID {
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

enum class HAL_CANDeviceType : int32_t {
    /// Broadcast.
    HAL_CAN_Dev_kBroadcast = 0,
    /// Robot controller.
    HAL_CAN_Dev_kRobotController = 1,
    /// Motor controller.
    HAL_CAN_Dev_kMotorController = 2,
    /// Relay controller.
    HAL_CAN_Dev_kRelayController = 3,
    /// Gyro sensor.
    HAL_CAN_Dev_kGyroSensor = 4,
    /// Accelerometer.
    HAL_CAN_Dev_kAccelerometer = 5,
    /// Ultrasonic sensor.
    HAL_CAN_Dev_kUltrasonicSensor = 6,
    /// Gear tooth sensor.
    HAL_CAN_Dev_kGearToothSensor = 7,
    /// Power distribution.
    HAL_CAN_Dev_kPowerDistribution = 8,
    /// Pneumatics.
    HAL_CAN_Dev_kPneumatics = 9,
    /// Miscellaneous.
    HAL_CAN_Dev_kMiscellaneous = 10,
    /// IO breakout.
    HAL_CAN_Dev_kIOBreakout = 11,
    /// Firmware update.
    HAL_CAN_Dev_kFirmwareUpdate = 31
};

enum class HAL_CANManufacturer : int32_t {
    /// Broadcast.
    HAL_CAN_Man_kBroadcast = 0,
    /// National Instruments.
    HAL_CAN_Man_kNI = 1,
    /// Luminary Micro.
    HAL_CAN_Man_kLM = 2,
    /// DEKA Research and Development Corp.
    HAL_CAN_Man_kDEKA = 3,
    /// Cross the Road Electronics.
    HAL_CAN_Man_kCTRE = 4,
    /// REV robotics.
    HAL_CAN_Man_kREV = 5,
    /// Grapple.
    HAL_CAN_Man_kGrapple = 6,
    /// MindSensors.
    HAL_CAN_Man_kMS = 7,
    /// Team use.
    HAL_CAN_Man_kTeamUse = 8,
    /// Kauai Labs.
    HAL_CAN_Man_kKauaiLabs = 9,
    /// Copperforge.
    HAL_CAN_Man_kCopperforge = 10,
    /// Playing With Fusion.
    HAL_CAN_Man_kPWF = 11,
    /// Studica.
    HAL_CAN_Man_kStudica = 12,
    /// TheThriftyBot.
    HAL_CAN_Man_kTheThriftyBot = 13,
    /// Redux Robotics.
    HAL_CAN_Man_kReduxRobotics = 14,
    /// AndyMark.
    HAL_CAN_Man_kAndyMark = 15,
    /// Vivid-Hosting.
    HAL_CAN_Man_kVividHosting = 16
};

/**
 * @brief Device manufacturer, type, and ID
 *
 */
struct CANStorage {
    HAL_CANManufacturer manufacturer;
    HAL_CANDeviceType deviceType;
    uint8_t deviceId;
    // wpi::mutex mapMutex;
    // wpi::SmallDenseMap<int32_t, int32_t> periodicSends;
    // wpi::SmallDenseMap<int32_t, Receives> receives;
};

//--------------------------------------------------------------------------//
//   REV Functions                                                          //
//--------------------------------------------------------------------------//

void identify(CANStorage* storage, AstraCAN& Can0);

void setParameterNew(CANStorage* storage, uint8_t parameterID, int32_t type, uint32_t value,
                     AstraCAN& Can0);

void HAL_WriteCANPacket(CANStorage* storage, uint8_t data[], uint8_t length, int32_t apiId,
                        int32_t* status, AstraCAN& Can0);

void HAL_CAN_SendMessage(uint32_t messageID, uint8_t data[], uint8_t dataSize, int32_t* status,
                         AstraCAN& Can0);

/**
 * @brief Creates a CAN frame ID using information about the frame's target and purpose.
 *
 * @param storage A CANStorage ptr containing information about the target device including
 * manufacturer, device type, and device ID
 * @param apiId 32-bit int relating to the purpose of the CAN frame, i.e. the command (gets
 * truncated to 10 bits)
 * @return 32-bit int ID for the CAN frame
 */
int32_t CreateCANId(CANStorage* storage, int32_t apiId);

#endif  // End ESP32 CAN code
