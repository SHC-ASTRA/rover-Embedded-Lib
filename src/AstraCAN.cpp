/**
 * @file AstraCAN.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief 
 * 
 */

#if __has_include("FlexCAN_T4.h")

#include "AstraCAN.h"


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
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2051D80 + can_id;
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = 0;
    msg.buf[0] = can_id;
    Can0.write(msg);
}


void sendDutyCycle(AstraFCAN &Can0, int can_id, float duty_cycle) {
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2050080 + can_id;
    Float2LEDec(duty_cycle, msg.buf);
    Can0.write(msg);
}

void sendHeartbeat(AstraFCAN &Can0, int can_id) {
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2052C80;  // non-Rio heartbeat
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = 0;
    msg.buf[0] = pow(2, can_id);
    Can0.write(msg);
    // Serial.println(msg.id);
}

void setParameter(AstraFCAN &Can0, int can_id, uint8_t paramID, uint32_t value) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.len = 5;

    msg.id = 0x205C000 + can_id;
    Float2LEDec(value, msg.buf);  // Set the parameter to this
    msg.buf[4] = paramID;         // Parameter ID
    Can0.write(msg);
}

#endif // __has_include("FlexCAN_T4.h")
