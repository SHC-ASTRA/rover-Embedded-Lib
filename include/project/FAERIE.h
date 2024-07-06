/**
 * @file FAERIE.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Pinouts etc used on FAERIE embedded
 * @version 0.2
 * @date 2024-07-04
 *
 */
#pragma once


//------//
// PINS //
//------//

// Connects to Gate on nmos controlling laser GND
// Set HIGH to turn on laser, LOW to turn off
#define PIN_LASER_NMOS 15
// Connects to PWM on servo on SCABBARD
// (Not actually used)
// Use with Servo library
#define PIN_SERVO_PWM 1


//-----------//
// Constants //
//-----------//

#define COMMS_UART Serial4
#define COMMS_UART_NUM 4

// USB Serial baud, current Astra standard is 115200 afaik
#define SERIAL_BAUD 115200
// UART baud, for comms with Socket board
#define COMMS_UART_BAUD 115200

// Servo PWM min mcs
#define SERVO_MIN 500
// Servo PWM max mcs
#define SERVO_MAX 2500
