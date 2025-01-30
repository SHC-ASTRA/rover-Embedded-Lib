/**
 * @file ARM.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Arm
 *
 */
#pragma once


#if defined(ESP32)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025)
//------------------------------------------------------------------------------------------------//

#    warning "ESP32 board for Arm is WIP!"

//-----------//
// Both MCUs //
//-----------//

#    define COMMS_UART Serial1  // UART between Main-Motor
#    define MOTOR_AMOUNT 3


#    if !defined(MOTORMCU)

//----------//
// Main MCU //
//----------//

#    else

//-----------//
// Motor MCU //
//-----------//

#   define MOTOR_ID_A1 1
#   define MOTOR_ID_A1 2
#   define MOTOR_ID_A1 3

#   warning "Motor IDs not set"

Serial1.print("Wrong Motor IDs used, they were added to resolve error, but not correctly selected");

#    endif


#elif defined(CORE_TEENSY)

//------------------------------------------------------------------------------------------------//
//   Teensy 4.x (URC 2024)
//------------------------------------------------------------------------------------------------//

//------//
// Pins //
//------//

#    define PIN_AS5047P_1_CS 10
#    define PIN_AS5047P_2_CS 37
#    define PIN_AS5047P_3_CS 36
#    define PIN_AXIS_0_PWM 19


//-----------//
// Constants //
//-----------//

#    define COMMS_UART Serial3

#    define LSS_SERIAL Serial7

#    define AS5047P_CUSTOM_SPI_BUS_SPEED 10'000'000


#endif
