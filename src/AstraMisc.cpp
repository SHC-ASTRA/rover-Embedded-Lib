/**
 * @file AstraMisc.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Misc functions used in Astra embedded
 * @version 0.1.1
 * @date 2024-07-04
 * 
 */

#include <Arduino.h>

#include <vector>

/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param args vector<String> to hold separated Strings
 * @param delim char which separates parts of input
 * @author David Sharpe, for ASTRA
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
        index = input.indexOf(delim, lastIndex + 1);
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
