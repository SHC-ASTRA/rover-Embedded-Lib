/**
 * @file TESTBED.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Testbed pins and constants and stuff
 *
 */
#pragma once


//-----------//
// Both MCUs //
//-----------//

#define COMMS_UART Serial1  // UART between Main-Motor

#define CAN_TX 13
#define CAN_RX 12


#if !defined(MOTORMCU)

//----------//
// Main MCU //
//----------//

// Voltage Dividers
#    define PIN_VDIV_5V 33
#    define PIN_VDIV_BATT 15
#    define PIN_VDIV_12V 32
#    define PIN_VDIV_3V3 14

// I2C
#    define I2C_SCL 20
#    define I2C_SDA 22


#else

//-----------//
// Motor MCU //
//-----------//

#    define MOTOR_ID_FL 2  // REV motor ID for front left wheel
#    define MOTOR_ID_FR 1  // REV motor ID for front right wheel
#    define MOTOR_ID_BL 4  // REV motor ID for back left wheel
#    define MOTOR_ID_BR 3  // REV motor ID for back right wheel

#   define WHEEL_CIRCUMFERENCE 0.6168  // Wheel's circumference in meters

#endif
