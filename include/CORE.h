/**
 * @file CORE.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Core
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

#define CORE

#include "AstraCAN.h"
#include "AstraMotors.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"


//------//
// PINS //
//------//

#define PIN_LED_STRIP 10

// 20 used for something with the IMU?


//-----------//
// Constants //
//-----------//

#define SERIAL_BAUD 115200
