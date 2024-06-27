/**
 * @file CITADEL.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Pins etc used on CITADEL embedded
 * @version 0.1
 * @date 2024-06-25
 * 
 */
#pragma once

#include <Arduino.h>
#include <LSS.h>


//------//
// PINS //
//------//

#define PIN_SERVO_1_PWM 19
#define PIN_SERVO_2_PWM 22
#define PIN_SERVO_3_PWM 23
#define PIN_FAN_1_NMOS 33
#define PIN_FAN_2_NMOS 31
#define PIN_FAN_3_NMOS 32
#define PIN_PUMP_1_NMOS 38
#define PIN_PUMP_2_NMOS 39
#define PIN_PUMP_3_NMOS 40
#define PIN_PUMP_4_NMOS 41

//-----------//
// Constants //
//-----------//

// ID of Lynx Servo, on Bio Arm
#define LSS_ID (3)
#define LSS_BAUD (LSS_DefaultBaud)
#define LSS_SERIAL (Serial2)

// UART used for comms with Raspberry Pi
// (Not currently used)
// USB Serial used instead
#define COMMS_UART Serial1
// UART used to control PTZ camera
// (Not currently used)
// No PTZ currently, and only used if IP control can't be used instead
#define PTZ_UART Serial7