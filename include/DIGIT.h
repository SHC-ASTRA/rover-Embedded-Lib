/**
 * @file DIGIT.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Digit
 * @version 0.1
 * @date 2024-06-27
 *
 */
#pragma once


//---------------------//
// PlatformIO lib_deps //
//---------------------//

// https://github.com/Lynxmotion/LSS_Library_Arduino

//------//
// LIBS //
//------//

#define DIGIT

#include <Servo.h>  // Mandible     // This name fits perfectly but I hate it soo much


//------//
// PINS //
//------//

#define PIN_LASER 15
#define PIN_AS5047P_CS 17


//-----------//
// Constants //
//-----------//

#define SERIAL_BAUD 115200

#define COMMS_UART Serial1
#define COMMS_UART_BAUD 115200
