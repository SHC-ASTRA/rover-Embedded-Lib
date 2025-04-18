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

//-----------//
// Both MCUs //
//-----------//

#    define COMMS_UART Serial1  // UART between Main-Motor



#    if !defined(MOTORMCU)

//----------//
// Main MCU //
//----------//

#        define ENCODER_AXIS0_PIN 33
#        define ENCODER_AXIS1_PIN 26
#        define ENCODER_AXIS2_PIN 25
#        define ENCODER_AXIS3_PIN 4

#        define CAN_TX 13
#        define CAN_RX 12

#        define LYNX_TX 22
#        define LYNX_RX 20

#        define MOTOR_AMOUNT 4

#        define AX0_CS 15

#        define TURNRIGHT 1
#        define TURNLEFT 0

#    else

//-----------//
// Motor MCU //
//-----------//

#        define MOTOR_ID_A1 1
#        define MOTOR_ID_A2 2
#        define MOTOR_ID_A3 3

#        define CAN_RX 12
#        define CAN_TX 13

#        define MOTOR_AMOUNT 3

#        warning "Motor IDs not set"

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
