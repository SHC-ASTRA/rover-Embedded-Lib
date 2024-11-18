# rover-Embedded-Lib

Standardizing ASTRA's embedded code.

## Table of Contents

 1. [Overview](#overview)
 2. [Adding to an exiting PlatformIO project](#adding-to-an-existing-platformio-project)
 3. [Starting a new PlatformIO project](#adding-to-an-existing-platformio-project)
 4. [Naming conventions](#naming-conventions)
 5. [File List](#files)
 6. [Theory](#theory)
 7. [Updating this Repository](#updating-this-repository)
 8. [Responsible People](#responsible-people)

## Overview

This library provides classes, structs, functions, pin number macros, and
team-wide constants, just to name a few. Here are a few examples:

* `parseInput(const String, std::vector<String>&)`: takes input from USB or UART and separates it into the vector, using commas as delimiters. Very useful for dealing with commands and data input from other mcu's.
* `CAN_sendDutyCycle(uint8_t, AstraCAN&)`: Formats and sends a CAN packet to a REV motor controller with a duty cycle. All communication with REV motor controllers should reside in `AstraREVCAN.{h,cpp}`.
* `isCalibrated(Adafruit_BNO055&)`: Attempts to check whether the BNO has been calibrated in EEPROM.
* `COMMS_UART`: Macro for the UART interface used by each mcu.
* `SERIAL_BAUD`: Standard baudrate for USB Serial used by all of ASTRA's mcu's.

## Adding to an existing PlatformIO project

 1. Add the following line to `lib_deps` in your `/platformio.ini`:

**https://github.com/SHC-ASTRA/rover-Embedded-Lib**

 2. Add the include statement for your project. Ex: `#include "project/CITADEL.h"`
 3. Make sure to grab the correct dependencies for `/platformio.ini` from the project headers.

## Starting a new PlatformIO project

 1. Copy the example file from `/.pio/libdeps/[board]/rover-Embedded-Lib/examples/Template/`
 2. Grab the correct project header from `include/project/` or create one
 from `TEMPLATE.h`

## Naming conventions

### In documentation

* **Library files** - Depending on context, either the files generally contained in the library,
or the main functional C++ files containing functions and classes.
* **Project header** - A header file that corresponds to one PlatformIO project, residing in `include/project/`.

### File names

* **Library files** - Camel case with the first letter of all words, including the first, capitalized. Ex: `AstraArm.cpp`
* **Project headers** - All caps. Ex: `CITADEL.h`

## Files

### Classes

* `AstraArm.h/.cpp` - Arm
* `AstraMotors.h/.cpp` - REV motor
* `AstraNP.h/.cpp` - Status indicator using NeoPixel

### Library

* `library.json` - PlatformIO stuff
* `README.md` - this file. Documentation and GitHub front page.

### Misc

* `AstraREVCAN.h/.cpp` - ASTRA's implementation of CAN communication with REV motors
* `AstraSensors.h/.cpp` - functions for sensors
* `AstraMisc.h/.cpp` - functions, consts, etc. useful to all ASTRA projects.

### Project Headers

* One per project.
* All caps file name, aside from `.h`
* Found in `include/project/`
* Includes pinouts (all pinouts important for code, optionally ones not needed for code)
* Includes constants relating to hardware
* DOES NOT include constants only relevant to the code
* The goal is too put less work on the embedded programmer, abstracting away electronics
* The embedded programmer SHOULD NOT have to edit their project's library. When in doubt, it goes in `main.cpp`.

## Theory

* `main.cpp` includes all the library headers it needs
* Only if a library header is included by `main.cpp` will it throw `#include` errors. If the header isn't
included in `main.cpp`, then no include errors, but if it is, then you will get required include errors.
* `.cpp` files enable themselves when all of its required external libraries are found. External libraries
are 90% of the reason the `.cpp` files need the option to be disabled.

## Updating this Repository

When making any update to this repository, before updating `main`, make sure to increment the version number in `library.json`, line 3.
Without making this change, projects using this library will not update automatically. Instead, they will have to delete the library
locally and let PlatformIO re-download it. (Not sure if this is true.......)

When making commits, it would be amazing if you use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/#summary).

### Creating a new project header

 1. Copy `TEMPLATE.h` in `rover-Embedded-Lib/include/project/`.
 2. Choose a project macro and add it to `ASTRA.h`.

### Creating a new header file

 1. Place the header file in `include/` and its implementation `.cpp` in `src/`.
 2. Choose a library macro to enable its files.
 3. Place the library macro in this file for documentation.

## Responsible People

### Author

Name: David Sharpe

Email: <ds0196@uah.edu>

### Maintainer

Name: David Sharpe

Email: <ds0196@uah.edu>
