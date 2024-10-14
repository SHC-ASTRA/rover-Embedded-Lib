/**
 * @file TESTBED.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Testbed pins and constants and stuff
 *
 */
#pragma once


//------//
// PINS //
//------//


// Both MCUs

#define COMMS_UART Serial1  // UART between Main-Motor

#define CAN_TX 13
#define CAN_RX 12


#if !defined(MOTORMCU)  // Main MCU  by default

// UART defined for both by COMMS_UART above

// CAN defined for both above

// Voltage Dividers
#define PIN_VDIV_5V 33
#define PIN_VDIV_BATT 15
#define PIN_VDIV_12V 32
#define PIN_VDIV_3V3 14

// I2C
#define I2C_SCL 20
#define I2C_SDA 22


#else  // Motor Controller MCU  otherwise

// There are no Motor MCU specific pins on Testbed

#endif


//-----------//
// Constants //
//-----------//
