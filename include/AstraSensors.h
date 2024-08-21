/**
 * @file AstraSensors.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Provides functions for using Astra's sensors
 * @version 0.1.3
 * @date 2024-07-06
 *
 */
#pragma once

// This is beatiful I love this so much
#if !__has_include(                                                                           \
    "AS5047P.h") ||                                                                           \
    !__has_include("Adafruit_BNO055.h") ||                                                    \
                   !__has_include("Adafruit_Sensor.h") ||                                     \
                                  !__has_include("SparkFun_u-blox_GNSS_Arduino_Library.h") || \
                                                 !__has_include("Adafruit_BMP3XX.h")

#    if !__has_include("AS5047P.h")
#        error Missing library! Please add the following line to lib_deps in platformio.ini:  jonas-merkle/AS5047P
#    endif
#    if !__has_include("Adafruit_BNO055.h")
#        error Missing library! Please add the following line to lib_deps in platformio.ini:  adafruit/Adafruit BNO055
#    endif
#    if !__has_include("Adafruit_Sensor.h")
#        error Missing library! Please add the following line to lib_deps in platformio.ini:  adafruit/Adafruit Unified Sensor
#    endif
#    if !__has_include("SparkFun_u-blox_GNSS_Arduino_Library.h")
#        error Missing library! Please add the following line to lib_deps in platformio.ini:  sparkfun/SparkFun u-blox GNSS Arduino Library
#    endif
#    if !__has_include("Adafruit_BMP3XX.h")
#        error Missing library! Please add the following line to lib_deps in platformio.ini:  adafruit/Adafruit BMP3XX Library
#    endif

#else

#    include <AS5047P.h>          // jonas-merkle/AS5047P
#    include <Adafruit_BNO055.h>  // adafruit/Adafruit BNO055
#    include <Adafruit_Sensor.h>  // adafruit/Adafruit Unified Sensor
#    include <EEPROM.h>
#    include <SPI.h>
#    include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // sparkfun/SparkFun u-blox GNSS Arduino Library
#    include <Wire.h>
#    include <utility/imumaths.h>  // where does come from?

#    include <cmath>
#    include <cstdlib>
#    include <queue>

#    include "Adafruit_BMP3XX.h"  // adafruit/Adafruit BMP3XX Library


#    define SEALEVELPRESSURE_HPA (1013.25)


/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(Adafruit_BNO055 &bno);

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(Adafruit_BNO055 &bno);

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(Adafruit_BNO055 &bno);

bool isCalibrated(Adafruit_BNO055 &bno);

void loadCalibration(Adafruit_BNO055 &bno);

void calibrateBNO(Adafruit_BNO055 &bno);

void pullBNOData(Adafruit_BNO055 &bno, float (&bno_data)[7]);

void initializeBMP(Adafruit_BMP3XX &bmp);

void pullBMPData(Adafruit_BMP3XX &bmp, float (&bmp_data)[3]);

float getBNOOrient(Adafruit_BNO055 &bno);

void getPosition(SFE_UBLOX_GNSS &myGNSS, double (&gps_data)[3]);

String getUTC(SFE_UBLOX_GNSS &myGNSS);

#endif  // __has_include
