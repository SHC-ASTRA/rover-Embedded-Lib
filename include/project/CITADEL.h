/**
 * @file CITADEL.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Pins etc used on CITADEL embedded
 *
 */
#pragma once


#if defined(ESP32)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025)
//------------------------------------------------------------------------------------------------//

#ifndef MOTORMCU

#warning "Pins not added for main MCU yet..."

#else

#   define PIN_STEP_1_DIR 14
#   define PIN_STEP_1_STEP 32
#   define PIN_STEP_2_DIR 15
#   define PIN_STEP_2_STEP 33
#   define PIN_STEP_3_STEP 12
#   define PIN_STEP_3_DIR 27

#   define PIN_STEP_4_STEP 22
#   define PIN_STEP_4_DIR 20

#   define PIN_M0 5
#   define PIN_M1 19

#endif


#elif defined(CORE_TEENSY)

//------------------------------------------------------------------------------------------------//
//   Teensy 4.x (URC 2024)
//------------------------------------------------------------------------------------------------//

//------//
// Pins //
//------//

#    define PIN_SERVO_1_PWM 19
#    define PIN_SERVO_2_PWM 22
#    define PIN_SERVO_3_PWM 23
#    define PIN_FAN_1_NMOS 33
#    define PIN_FAN_2_NMOS 31
#    define PIN_FAN_3_NMOS 32
#    define PIN_PUMP_1_NMOS 38
#    define PIN_PUMP_2_NMOS 39
#    define PIN_PUMP_3_NMOS 40
#    define PIN_PUMP_4_NMOS 41


//-----------//
// Constants //
//-----------//

// ID of Lynx Servo, on Bio Arm
#    define LSS_ID (3)
#    define LSS_SERIAL Serial2

// UART used for comms with Raspberry Pi
// (Not currently used)
// USB Serial used instead
#    define COMMS_UART Serial1
// UART used to control PTZ camera
// (Not currently used)
// No PTZ currently, and only used if IP control can't be used instead
#    define PTZ_UART Serial7


#endif
