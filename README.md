# rover-Embedded-Lib

ASTRA's libraries for embedded code to interface with motors, sensors, etc, and provide pinouts, misc
constants, functions

## Implementing into existing PlatformIO project

 1. Add the following line to `lib_deps` in your `/platformio.ini`:
 [https://github.com/SHC-ASTRA/rover-Embedded-Lib](https://github.com/SHC-ASTRA/rover-Embedded-Lib)
 2. Copy `/.pio/libdeps/[board]/rover-Embedded-Lib/examples/Template/AstraSELECTOR.h` to `/include/`.
 3. Uncomment the relevant project macro in the file you just copied.
 4. Replace the `#include` statements for hardware and Astra libraries in `Main.cpp` with `#include "ASTRA.h"`.
 5. Open the relevant file in `/.pio/libdeps/[board]/rover-Embedded-Lib/project/` and copy the libraries to `/platformio.ini`.

## Starting a new PlatformIO project

 1. [To be written]
 2. [Copy example files from `/.pio/libdeps/teensy40/rover-Embedded-Lib/examples/Template/`, copy template file and fill with stuff you need]

## Naming conventions

### In documentation

- **Library files** - Depending on context, either the files generally contained in the library,
or the main functional C++ files containing functions and classes.
- **Project header** - A header file that corresponds specifically to one or more PlatformIO projects.
- **Project macro** - A macro specific to a single PlatformIO project. Ex: `#define ARM`

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
- Includes pinouts (all pinouts important for code, optionally ones not needed for code)
- Includes constants relating to hardware
- DOES NOT include constants only relevant to the code
- DOES NOT include header files only relevant to the code (i.e., `AS5047P.h` goes in project header, `cmath` goes in main.cpp)
- The goal is too put less work on the embedded programmer, abstracting away electronics
- The embedded programmer SHOULD NOT have to edit their project's library. When in doubt, it goes in `main.cpp`.

## Theory

- `main.cpp` includes `ASTRA.h`
- `ASTRA.h` checks if `AstraSELECTOR.h` appears in the include path.
- `AstraSELECTOR.h` appears in the project directory, outside of the library, so that the project macro stays selected
even if the library is updated or removed/re-installed. The goal is to require *zero* edits to library files.
- `ASTRA.h` includes `AstraSELECTOR.h` and contains all error handling, as to not convolute other library files.
- The main goal of `ASTRA.h` is to have one header that all library files can reference to obtain the project macro
and enable/disable themselves (as to not pollute the project with unneeded PlatformIO libraries).
- `ASTRA.h` uses the macro from `AstraSELECTOR.h` to include a project header, removing the burden from `main.cpp`
and making it so that the only configuration required by the user is modifying `AstraSELECTOR.h` instead of finding
and including the right project header from the library.
- A bonus of the project headers including library files instead of `main.cpp` is to abstract away the library's
structure, so the embedded programmer working on their project only has to worry about the functionally of the library,
not the implementation.
