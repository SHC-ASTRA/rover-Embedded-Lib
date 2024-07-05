/**
 * @file AstraMisc.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Misc functions and definitions used in Astra embedded
 * @version 0.1.2
 * @date 2024-07-04
 *
 */
#pragma once

#include "ASTRA.h"

#include <vector>


// TODO: Maybe loopHeartbeats() can go here?
// It could take AstraMotors*[] to address having multiple motors.
// Using a for loop to iterate through the motors


/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param args vector<String> to hold separated Strings
 * @param delim char which separates parts of input
 * @author David Sharpe, for ASTRA
 */
void parseInput(const String input, std::vector<String>& args, const char delim);
