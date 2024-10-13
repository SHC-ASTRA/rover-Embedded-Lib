//WHAT THE HELL AM I DOING?
/**
 * @file AstraCAN2.cpp
 * @author Charles Marmann (cmm0077@uah.edu)
 * @brief 
 * 
 */

// #define ENABLE_ASTRACAN2_h

#if __has_include("ESP32-TWAI-CAN.hpp") && defined(ENABLE_ASTRACAN2_H)

#include "AstraCAN2.h"



// Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (&buffer_data)[8]) {
    unsigned char b[8] = {0};
    memcpy(b, &x, 4);
    // int* buffer_data[4];
    for (int i = 0; i < 4; i++) {
        buffer_data[i] = b[i];
    }
    for (int i = 4; i < 8; i++) {
        buffer_data[i] = 0;
    }
}


void identifyDevice(AstraFCAN &Can0, int can_id) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 8;

    msg.identifier = 0x2051D80 + can_id;
    for (uint8_t i = 0; i < 8; i++)
        msg.data[i] = 0;
    msg.data[0] = can_id;
    Can0.writeFrame(msg);
}


void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle) {

    // Code mostly taken from example
    CanFrame msg = { 0 };
	msg.identifier = 0x2050080 + can_id;
	msg.extd = 1;
	msg.data_length_code = 8;

    Float2LEDec(duty_cycle, msg.data);

    Can0.writeFrame(msg);
}

void sendHeartbeat(AstraFCAN &Can0, int can_id) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 8;

    msg.identifier = 0x2052C80;  // non-Rio heartbeat
    for (uint8_t i = 0; i < 8; i++)
        msg.data[i] = 0;
    msg.data[0] = pow(2, can_id);
    Can0.writeFrame(msg);
    // Serial.println(msg.id);
}

void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 5;

    msg.identifier = 0x205C000 + can_id;
    Float2LEDec(value, msg.data);  // Set the parameter to this
    msg.data[4] = paramID;         // Parameter ID
    Can0.writeFrame(msg);
}

#endif // __has_include("FlexCAN_T4.h")
