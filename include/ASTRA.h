/**
 * @file ASTRA.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief
 * @version 0.2.1
 * @date 2024-07-04
 *
 */
#pragma once


//---------------------//
// PlatformIO lib_deps //
//---------------------//

// * Misc
// https://github.com/ftrias/TeensyThreads
// https://github.com/Lynxmotion/LSS_Library_Arduino
// adafruit/Adafruit Unified Sensor
// adafruit/Adafruit SHT31 Library
// * ARM
// jonas-merkle/AS5047P
// https://github.com/Lynxmotion/LSS_Library_Arduino
// * REVMOTOR
// https://github.com/tonton81/FlexCAN_T4
// * SENSOR
// adafruit/Adafruit Unified Sensor
// adafruit/Adafruit BMP3XX Library
// adafruit/Adafruit BNO055
// adafruit/Adafruit Unified Sensor
// sparkfun/SparkFun u-blox GNSS Arduino Library


//----------//
// Includes //
//----------//

#if defined(ASTRA)


#if defined(CORE)
#include "project/CORE.h"

#elif defined(ARM)
#include "project/ARM.h"

#elif defined(WRIST)
#include "project/WRIST.h"

#elif defined(FAERIE)
#include "project/FAERIE.h"

#elif defined(CITADEL)
#include "project/CITADEL.h"

#else
#warning "Please uncomment the relevant `#define` statement in `/include/AstraSELECTOR.h`."

#endif  // defined(CORE)


#include <Arduino.h>

#include "AstraArm.h"
#include "AstraCAN.h"
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraSensors.h"


#else  // platformio.ini not set up with build options
#warning "Please modify platformio.ini to add ASTRA build options"

#endif  // defined(ASTRA)
