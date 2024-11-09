/**
 * @file AstraCAN.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @author Charles Marmann (cmm0077@uah.edu)
 * @author David Sharpe (ds0196@uah.edu)
 * @brief
 *
 */

// #define ENABLE

#if __has_include("ESP32-TWAI-CAN.hpp")

#   include "AstraCAN.h"


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

void identifyDevice(AstraCAN& Can0, int can_id) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 8;

    msg.identifier = 0x2051D80 + can_id;
    for (uint8_t i = 0; i < 8; i++)
        msg.data[i] = 0;
    msg.data[0] = can_id;
    Can0.writeFrame(msg);
}


void sendDutyCycle(AstraCAN& Can0, int can_id, float duty_cycle) {
    CanFrame msg = {0};
    msg.identifier = 0x2050080 + can_id;
    msg.extd = 1;
    msg.data_length_code = 8;

    Float2LEDec(duty_cycle, msg.data);

    Can0.writeFrame(msg);
}

void sendHeartbeat(AstraCAN& Can0, int can_id) {
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

void setParameter(AstraCAN& Can0, int can_id, uint8_t paramID, uint32_t value) {
    CanFrame msg;
    msg.extd = 1;
    msg.data_length_code = 5;

    msg.identifier = 0x2050000; // + can_id;
    msg.identifier |= ((uint8_t)can_id & 0x3F);
    msg.identifier |= ((paramID | 0x03) & 0x3FF) << 6;
    msg.data[3] = (uint8_t)value;
    // Float2LEDec(value, msg.data);
    msg.data[4] = 3;
    // msg.data[4] = paramID;         // Parameter ID
    Can0.writeFrame(msg);
}


#elif __has_include("FlexCAN_T4.h")

#    include "AstraCAN.h"

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


void identifyDevice(AstraCAN &Can0, int can_id) {
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2051D80 + can_id;
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = 0;
    msg.buf[0] = can_id;
    Can0.write(msg);
}


void sendDutyCycle(AstraCAN &Can0, int can_id, float duty_cycle) {
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2050080 + can_id;
    Float2LEDec(duty_cycle, msg.buf);
    Can0.write(msg);
}

void sendHeartbeat(AstraCAN &Can0, int can_id) {
    CAN_message_t msg;
    msg.flags.extended = 1;

    msg.id = 0x2052C80;  // non-Rio heartbeat
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = 0;
    msg.buf[0] = pow(2, can_id);
    Can0.write(msg);
    // Serial.println(msg.id);
}

void setParameter(AstraCAN &Can0, int can_id, uint8_t paramID, uint32_t value) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.len = 5;

    msg.id = 0x205C000 + can_id;
    Float2LEDec(value, msg.buf);  // Set the parameter to this
    msg.buf[4] = paramID;         // Parameter ID
    Can0.write(msg);
}

#endif  // __has_include("FlexCAN_T4.h")
