/**
 * @file Template.cpp
 * @author your name (you@domain.com)
 * @brief description
 * @version 0.1.2
 * @date 2024-06-27
 *
 */

#include "ASTRA.h"


// Comment out to disable LED blinking
#define BLINK 1

uint32_t lastBlink = 0;


void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        digitalToggle(LED_BUILTIN);
    }
#endif


    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable


        /**/ if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle")
                digitalToggle(LED_BUILTIN);
            else
                digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}
