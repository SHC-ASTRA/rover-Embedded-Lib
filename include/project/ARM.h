/**
 * @file ARM.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Arm
 * @version 0.2
 * @date 2024-07-04
 *
 */
#pragma once


//------//
// PINS //
//------//

#define PIN_AS5047P_1_CS 10
#define PIN_AS5047P_2_CS 37
#define PIN_AS5047P_3_CS 36
#define PIN_AXIS_0_PWM 19


//-----------//
// Constants //
//-----------//

#define SERIAL_BAUD 115200

#define COMMS_UART Serial3
#define COMMS_UART_BAUD 115200

#define LSS_BAUD (LSS_DefaultBaud)
#define LSS_SERIAL (Serial7)

#define AS5047P_CUSTOM_SPI_BUS_SPEED 10'000'000
