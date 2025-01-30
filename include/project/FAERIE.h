/**
 * @file FAERIE.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Pinouts etc used on FAERIE embedded
 *
 */
#pragma once


#if defined(ESP32)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025)
//------------------------------------------------------------------------------------------------//

#    define COMMS_UART Serial1
#    define COMMS_UART_NUM 4
#   define PIN_CAN_RX 7
#   define PIN_CAN_TX 8
#   define PIN_LASERS 27
#   define PIN_SPARKMAX_PWM 33
#   define PIN_5V_ADC 34
#   define PIN_BATT_ADC 36
#   define PIN_12V_ADC 39


#elif defined(CORE_TEENSY)

//------------------------------------------------------------------------------------------------//
//   Teensy 4.x (URC 2024)
//------------------------------------------------------------------------------------------------//

//------//
// Pins //
//------//

// Connects to Gate on nmos controlling laser GND
// Set HIGH to turn on laser, LOW to turn off
#    define PIN_LASER_NMOS 15
// Connects to PWM on servo on SCABBARD
// (Not actually used)
// Use with Servo library
#    define PIN_SERVO_PWM 1


//-----------//
// Constants //
//-----------//

#    define COMMS_UART Serial1
#    define COMMS_UART_NUM 4


// Servo PWM min mcs
#    define SERVO_MIN 500
// Servo PWM max mcs
#    define SERVO_MAX 2500


#endif
