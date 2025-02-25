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

// Main MCU

#define PIN_FAN_1 14
#define PIN_FAN_2 32
#define PIN_FAN_3 15
#define PIN_LYNX_RX 33
#define PIN_LYNX_TX 27
#define PIN_VIBMOTOR 13

#define PIN_PWMSERVO_1 5
#define PIN_PWMSERVO_2 19
#define PIN_PWMSERVO_3 21

#define PIN_CRX 22
#define PIN_CTX 20

// These are purposefully flipped! The schematic is from the perspective of the Raspi.
#define PIN_PI_TX 26
#define PIN_PI_RX 25

#define PIN_ADC_VBATT 34
#define PIN_ADC_12V 39
#define PIN_ADC_5V 36

#else

// Motor MCU

#   define PIN_STEP_1_DIR 14
#   define PIN_STEP_1_STEP 32
#   define PIN_STEP_2_DIR 15
#   define PIN_STEP_2_STEP 33
#   define PIN_STEP_3_DIR 12
#   define PIN_STEP_3_STEP 27

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
