# rover-Embedded-Lib

ASTRA's libraries for embedded code to interface with motors, sensors, etc, and provide pinouts, misc
constants, functions

## Files

### Classes

- `AstraArm.h/cpp` - Arm

- `AstraMotors.h/cpp` - REV motor

### Misc

- `AstraCAN.h` - ASTRA's implementation of CAN communication with REV motors

- `AstraSensors.h` - functions for sensors

- `AstraMisc.h` - functions, consts, etc. useful to all ASTRA projects.

### Project Headers

- One per project.
- All caps file name, aside from `.h`
- Includes pinouts (all pinouts important for code, optionally ones not needed for code)
- Includes constants relating to hardware
- Includes project macro for activating relavant library `.cpp` files
- DOES NOT include constants only relevant to the code
- DOES NOT include header files only relevant to the code (i.e., `AS5047P.h` goes in project header, `cmath` goes in main.cpp)
- The goal is too put less work on the embedded programmer, abstracting away electronics
- The embedded programmer SHOULD NOT have to edit their project's library. In doubt, it goes in `main.cpp`.

### Library

- `library.json` - PlatformIO stuff

- `README.ms` - this file. Documentation and GitHub front page.
