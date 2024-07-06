/**
 * @file AstraSensors.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Provides functions for using Astra's sensors
 * @version 0.1.2
 * @date 2024-07-04
 *
 */
#pragma once

#include <AS5047P.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include <cmath>
#include <cstdlib>
#include <queue>

#include "Adafruit_BMP3XX.h"


#define SEALEVELPRESSURE_HPA (1013.25)


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

void getPosition(SFE_UBLOX_GNSS &myGNSS, float (&gps_data)[3]);

String getUTC(SFE_UBLOX_GNSS &myGNSS);
