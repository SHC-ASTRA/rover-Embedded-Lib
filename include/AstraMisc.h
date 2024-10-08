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


// TODO: Maybe loopHeartbeats() can go here?
// It could take AstraMotors*[] to address having multiple motors.
// Using a for loop to iterate through the motors

// void loopHeartbeatsNew(AstraMotors* motors[], const int numMotors);


/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param args vector<String> to hold separated Strings
 * @param delim char which separates parts of input
 * @author David Sharpe, for ASTRA
 */
void parseInput(const String input, std::vector<String>& args, const char delim);
