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

typedef unsigned long ASTRA_TIME_T;


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
    ASTRA_TIME_T lastMillis = 0;
    ASTRA_TIME_T interval = 0;  // e.g. (millis() - lastMillis >= interval)
    int state = 0;
};


// Clamps x between out_min and out_max using the expected input min and max
// Used for controller input
double map_d(double x, double in_min, double in_max, double out_min, double out_max) {
    const double run = in_max - in_min;
    if (run == 0)
    {
	    return 0;  // in_min == in_max, error
    }
    const double rise = out_max - out_min;
    const double delta = x - in_min;
    return (delta * rise) / run + out_min;
}


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

/**
 * @brief Converts ADC reading to voltage based on a voltage divider
 * 
 * @param reading ADC reading
 * @param r1 Resistance of R1 in the voltage divider (in kOhms)
 * @param r2 Resistance of R2 in the voltage divider (in kOhms)
 * @return float Voltage calculated from the ADC reading
 */
float convertADC(uint16_t reading, const float r1, const float r2) {
    if (reading == 0)
        return 0;  // Avoid divide by zero

    // Max Vs that the voltage divider is designed to read
    const float maxSource = (3.3 * (r1 * 1000 + r2 * 1000)) / (r2 * 1000);

    // ADC range is 0-4095 (12-bit precision)
    return (static_cast<float>(reading) / 4095.0) * maxSource;  // Clamp reading [0-maxSource]
}


// Whether or not to print on every stopwatch action
#define STOPWATCH_PRINT
// Serial(*) to use for stopwatch printouts
#define STOPWATCH_SERIAL Serial

class Stopwatch_t {
private:
    ASTRA_TIME_T start_time = 0;
    ASTRA_TIME_T stop_time = 0;
    std::vector<ASTRA_TIME_T> lap_times;

public:
    void printMicros(ASTRA_TIME_T stamp) const {
        if (stamp < 2000) {  // < 2 ms
            STOPWATCH_SERIAL.print(stamp);
            STOPWATCH_SERIAL.print(" μs");
        } else if (stamp < 2 * 1000 * 1000) {  // < 2 s
            STOPWATCH_SERIAL.print(stamp / 1000.0);
            STOPWATCH_SERIAL.print(" ms");
        } else if (stamp < 2 * 1000 * 1000 * 60) {  // < 2 min
            STOPWATCH_SERIAL.print(stamp / (1000.0 * 1000.0));
            STOPWATCH_SERIAL.print(" s");
        } else {  // >= 2 min
            STOPWATCH_SERIAL.print(stamp / (1000.0 * 1000.0 * 60.0));
            STOPWATCH_SERIAL.print(" min");
        }
    }

    /**
     * @brief Resets all values and starts stopwatch at current micros()
     *
     * @return ASTRA_TIME_T start_time = micros()
     */
    ASTRA_TIME_T start() {
        start_time = micros();
        stop_time = 0;
        lap_times.clear();

#ifdef STOPWATCH_PRINT
        STOPWATCH_SERIAL.print("Stopwatch started at ");
        printMicros(start_time);
        STOPWATCH_SERIAL.println();
#endif

        return start_time;
    }

    /**
     * @brief Records current micros() as a lap time
     *
     * @return ASTRA_TIME_T lap_time = micros()
     */
    ASTRA_TIME_T lap() {
        ASTRA_TIME_T lap_time = micros();
        lap_times.push_back(lap_time);

#ifdef STOPWATCH_PRINT
        STOPWATCH_SERIAL.print("Stopwatch lapped (");
        STOPWATCH_SERIAL.print(lap_times.size()-1);
        STOPWATCH_SERIAL.print(") at ");
        printMicros(lap_time);
        STOPWATCH_SERIAL.println();
#endif

        return lap_time;
    }

    /**
     * @brief Records the current micros() as the stop time and prints summary
     *
     * @return ASTRA_TIME_T elapsed_time = micros() - start_time
     */
    ASTRA_TIME_T stop() {
        stop_time = micros();

#ifdef STOPWATCH_PRINT
        STOPWATCH_SERIAL.print("Stopwatch stopped.");

        printSummary();
#endif

        return stop_time - start_time;
    }

    /**
     * @brief Prints stopwatch summary to STOPWATCH_SERIAL
     * 
     */
    void printSummary() const {
        STOPWATCH_SERIAL.println("Stopwatch status:");
        STOPWATCH_SERIAL.print("Started at ");
        printMicros(start_time);
        STOPWATCH_SERIAL.println();

        if (lap_times.size() > 0) {
            for (const auto& i : lap_times) {
                STOPWATCH_SERIAL.print("Lap ");
                STOPWATCH_SERIAL.print(i + 1);
                STOPWATCH_SERIAL.print(" at ");
                printMicros(lap_times[i]);
                STOPWATCH_SERIAL.print(": ");
                if (i == 0)
                    printMicros(lap_times[i] - start_time);
                else
                    printMicros(lap_times[i] - lap_times[i - 1]);
                STOPWATCH_SERIAL.println();
            }
        }

        STOPWATCH_SERIAL.print("Stopped at ");
        printMicros(stop_time);
        if (lap_times.size() > 0) {
            STOPWATCH_SERIAL.print(": ");
            printMicros(stop_time - lap_times.back());
            STOPWATCH_SERIAL.println();
        } else {
            STOPWATCH_SERIAL.println();
        }

        STOPWATCH_SERIAL.print("Elapsed time: ");
        printMicros(stop_time - start_time);
        STOPWATCH_SERIAL.println();
    }

    ASTRA_TIME_T getStartTime() const {
        return start_time;
    }

    ASTRA_TIME_T getStopTime() const {
        return stop_time;
    }

    ASTRA_TIME_T getElapsedTime() const {
        return stop_time - start_time;
    }
} stopwatch;
