/**
 * @file DIGIT.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Digit
 *
 */
#pragma once


#if defined(ESP32)

#    warning "ESP32 board for Digit is WIP!"


#elif defined(CORE_TEENSY)  // URC 2024 setup

//------//
// Pins //
//------//

// DC on/off control for laser on end effector
#    define PIN_LASER 8
// PWM control for REV motor that opens/closes end effector
#    define PIN_EF_MOTOR 19


//-----------//
// Constants //
//-----------//

#    define COMMS_UART Serial1


#endif
