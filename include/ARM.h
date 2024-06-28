/**
 * @file ARM.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Arm
 * @version 0.1
 * @date 2024-06-27
 *
 */
#pragma once


//---------------------//
// PlatformIO lib_deps //
//---------------------//

// ftrias/TeensyThreads
// https://github.com/tonton81/FlexCAN_T4
// adafruit/Adafruit Unified Sensor
// adafruit/Adafruit BMP3XX Library
// adafruit/Adafruit BNO055
// adafruit/Adafruit Unified Sensor
// sparkfun/SparkFun u-blox GNSS Arduino Library
// jonas-merkle/AS5047P
// https://github.com/Lynxmotion/LSS_Library_Arduino


//------//
// LIBS //
//------//

#define ARM

#include <Servo.h>  // Axis 0

#include "AstraArm.h"
#include "AstraCAN.h"
#include "AstraMotors.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"


//------//
// PINS //
//------//

#define PIN_AS5047P_1_CS 10
#define PIN_AS5047P_2_CS 37
#define PIN_AS5047P_3_CS 36
#define PIN_AXIS_0_PWM 19


//-----------//
// Constants //
//-----------//

#define SERIAL_BAUD 115200

#define COMMS_UART Serial3
#define COMMS_UART_BAUD 115200

#define LSS_BAUD (LSS_DefaultBaud)
#define LSS_SERIAL (Serial7)

#define AS5047P_CUSTOM_SPI_BUS_SPEED 10'000'000
