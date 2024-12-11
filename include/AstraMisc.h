/**
 * @file AstraMisc.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Misc functions and definitions used in Astra embedded
 *
 */
#pragma once

#include <Arduino.h>

#include <vector>

// Baud rate for comms with LSS servos
#define LSS_BAUD LSS_DefaultBaud


// Baud rate for USB comms to a computer
#define SERIAL_BAUD 115200
// Baud rate for inter-microcontroller comms over UART
#define COMMS_UART_BAUD 115200

// All String messages for Astra embedded should use this delimiter
#define CMD_DELIM ','


// Standard struct to consolidate timer variables
// Example Usage:
//
// Timer ledBlink;
// ledBlink.interval = 1000;
// ...
// if (millis() - ledBlink.lastMillis >= ledBlink.interval) {
//     ledBlink.lastMillis = millis();
//     ledBlink.state = !ledBlink.state;
//     digitalWrite(LED_BUILTIN, ledBlink.state);
// }
struct Timer {
    unsigned long lastMillis = 0;
    unsigned long interval = 0;  // e.g. (millis() - lastMillis >= interval)
    int state = 0;
};


// TODO: Maybe loopHeartbeats() can go here?
// It could take AstraMotors*[] to address having multiple motors.
// Using a for loop to iterate through the motors

// void loopHeartbeatsNew(AstraMotors* motors[], const int numMotors);


/**
 * @brief 
 * 
 * @param args vector<String> args given by parseInput()
 * @param numArgs Number of arguments desired, not including first argument (e.g., "ctrl" or "ping")
 * @return true if the correct number of arguments have been provided
 */
inline bool checkArgs(const std::vector<String>& args, const size_t numArgs) {
    return (args.size() - 1 == numArgs);
}


/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param args vector<String> to hold separated Strings
 * @author David Sharpe
 */
void parseInput(const String input, std::vector<String>& args) {
    // Modified from
    // https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/9

    // Index of previously found delim
    int lastIndex = -1;
    // Index of currently found delim
    int index = -1;
    // lastIndex=index, so lastIndex starts at -1, and with lastIndex+1, first search begins at 0

    // if empty input for some reason, don't do anything
    if (input.length() == 0)
        return;

    // Protection against infinite loop
    unsigned count = 0;
    while (count++,
           count < 200 /*arbitrary limit on number of delims because while(true) is scary*/) {
        lastIndex = index;
        // using lastIndex+1 instead of input = input.substring to reduce memory impact
        index = input.indexOf(CMD_DELIM, lastIndex + 1);
        if (index == -1) {  // No instance of delim found in input
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex + 1));
            // Exit the loop when there are no more delims
            break;
        } else {  // delim found
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex + 1, index));
        }
    }

    // output is via vector<String>& args
}

/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param args vector<String> to hold separated Strings
 * @param delim char which separates parts of input
 * @author David Sharpe, for ASTRA
 * @deprecated Use function without delim parameter
 */
void parseInput(const String input, std::vector<String>& args, const char delim) {
    // Modified from
    // https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/9

    // Index of previously found delim
    int lastIndex = -1;
    // Index of currently found delim
    int index = -1;
    // lastIndex=index, so lastIndex starts at -1, and with lastIndex+1, first search begins at 0

    // if empty input for some reason, don't do anything
    if (input.length() == 0)
        return;

    // Protection against infinite loop
    unsigned count = 0;
    while (count++,
           count < 200 /*arbitrary limit on number of delims because while(true) is scary*/) {
        lastIndex = index;
        // using lastIndex+1 instead of input = input.substring to reduce memory impact
        index = input.indexOf(CMD_DELIM, lastIndex + 1);
        if (index == -1) {  // No instance of delim found in input
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex + 1));
            // Exit the loop when there are no more delims
            break;
        } else {  // delim found
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex + 1, index));
        }
    }

    // output is via vector<String>& args
}


#ifdef ARDUINO_RASPBERRY_PI_PICO
unsigned hwPinToGPIO(unsigned hwPin) {
    switch (hwPin) {
        case 1:
            return 0;
            break;
        case 2:
            return 1;
            break;
        case 4:
            return 2;
            break;
        case 5:
            return 3;
            break;
        case 6:
            return 4;
            break;
        case 7:
            return 5;
            break;
        case 9:
            return 6;
            break;
        case 10:
            return 7;
            break;
        case 11:
            return 8;
            break;
        case 12:
            return 9;
            break;
        case 14:
            return 10;
            break;
        case 15:
            return 11;
            break;
        case 16:
            return 12;
            break;
        case 17:
            return 13;
            break;
        case 19:
            return 14;
            break;
        case 20:
            return 15;
            break;
        case 21:
            return 16;
            break;
        case 22:
            return 17;
            break;
        case 24:
            return 18;
            break;
        case 25:
            return 19;
            break;
        case 26:
            return 20;
            break;
        case 27:
            return 21;
            break;
        case 29:
            return 22;
            break;
        case 31:
            return 26;
            break;
        case 32:
            return 27;
            break;
        case 34:
            return 28;
            break;
        default:
            return 0;
            break;
    }
}
#endif
