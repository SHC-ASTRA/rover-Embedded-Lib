/**
 * @file ASTRA.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 * @version 0.1
 * @date 2024-06-28
 * 
 */
#pragma once


#if __has_include("AstraSELECTOR.h")

#include "AstraSELECTOR.h"

#if defined(CORE)
#include "CORE.h"

#elif defined(ARM)
#include "ARM.h"

#elif defined(WRIST)
#include "WRIST.h"

#elif defined(FAERIE)
#include "FAERIE.h"

#elif defined(CITADEL)
#include "CITADEL.h"

#else
#warning "Please uncomment the relevant `#define` statement in `/include/AstraSELECTOR.h`."

#endif

#else  // Project does not have selector header.

#error "Please create `/include/AstraSELECTOR.h` and uncomment the relevant `#define` statement."


#endif  // __has_include("AstraSELECTOR.h")
