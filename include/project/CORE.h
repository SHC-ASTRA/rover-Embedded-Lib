/**
 * @file CORE.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Core
 *
 */
#pragma once

#if defined(ESP32)
#   warning "Core PCB has not been made yet; CORE.h is probably wrong."
#elif defined(CORE_TEENSY)
#  warning "Assuming testbed board used for Core at URC 2024"
#endif


//------//
// PINS //
//------//

#if defined(CORE_TEENSY)  // URC 2024 board

#   define PIN_LED_STRIP 10

// 20 used for something with the IMU?

#elif defined(ESP32) // New Core PCB

// Not made yet

#endif


//-----------//
// Constants //
//-----------//

