/**
 * @file CORE.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Core
 *
 */
#pragma once


#if defined(ESP32)

#    warning "ESP32 board for Core is WIP! Did you mean to use testbed?"


#elif defined(CORE_TEENSY)  // URC 2024 setup

//------//
// Pins //
//------//

#    define PIN_LED_STRIP 10

// 20 used for something with the IMU?


//-----------//
// Constants //
//-----------//

#endif
