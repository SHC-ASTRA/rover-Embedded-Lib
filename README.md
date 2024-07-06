# rover-Embedded-Lib

ASTRA's libraries for embedded code to interface with motors, sensors, etc, and provide pinouts, misc
constants, functions

## Implementing into existing PlatformIO project

 1. Add the following line to `lib_deps` in your `/platformio.ini`:
 [https://github.com/SHC-ASTRA/rover-Embedded-Lib](https://github.com/SHC-ASTRA/rover-Embedded-Lib)
 2. Add include statement for your project. Ex: `#include "project/CITADEL.h"`
 3. Make sure to grab the correct lines for `/platformio.ini` from the project headers.

## Starting a new PlatformIO project

 1. [To be written]
 2. [Copy example files from `/.pio/libdeps/[board]/rover-Embedded-Lib/examples/Template/`, copy template file and fill with stuff you need]

## Naming conventions

### In documentation

- **Library files** - Depending on context, either the files generally contained in the library,
or the main functional C++ files containing functions and classes.
- **Project header** - A header file that corresponds to one PlatformIO project.

### File names

- **Library files** - Camel case with the first letter of all words, including the first, capitalized. Ex: `AstraArm.cpp`
- **Project headers** - All caps. Ex: `CITADEL.h`

## Files

### Classes

- `AstraArm.h/.cpp` - Arm
- `AstraMotors.h/.cpp` - REV motor

### Library

- `library.json` - PlatformIO stuff
- `README.md` - this file. Documentation and GitHub front page.

### Misc

- `AstraCAN.h/.cpp` - ASTRA's implementation of CAN communication with REV motors
- `AstraSensors.h/.cpp` - functions for sensors
- `AstraMisc.h/.cpp` - functions, consts, etc. useful to all ASTRA projects.

### Project Headers

- One per project.
- All caps file name, aside from `.h`
- Found in `include/project/`
- Includes pinouts (all pinouts important for code, optionally ones not needed for code)
- Includes constants relating to hardware
- DOES NOT include constants only relevant to the code
- The goal is too put less work on the embedded programmer, abstracting away electronics
- The embedded programmer SHOULD NOT have to edit their project's library. When in doubt, it goes in `main.cpp`.

## Theory

- `main.cpp` includes all the library headers it needs
- Only if a library header is included by `main.cpp` will it throw `#include` errors. If the header isn't
included in `main.cpp`, then no include errors, but if it is, then you will get required include errors.
- `.cpp` files enable themselves when all of its required external libraries are found. External libraries
are 90% of the reason the `.cpp` files need the option to be disabled.

## Updating this Repository

When making any update to this repository, before updating `main`, make sure to increment the version number in `library.json`, line 3.
Without making this change, projects using this library will not update automatically. Instead, they will have to delete the library
locally and let PlatformIO re-download it.

### Creating a new project header

 1. Copy `TEMPLATE.h` in `rover-Embedded-Lib/include/project/`.
 2. Choose a project macro and add it to `ASTRA.h`.

### Creating a new header file

 1. Place the header file in `include/` and its implementation `.cpp` in `src/`.
 2. Choose a library macro to enable its files.
 3. Place the library macro in this file for documentation.
