/**
 * @file AstraMotors.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Implements class for controlling Rev Sparkmax motors
 *
 */

#if __has_include("ESP32-TWAI-CAN.hpp") || __has_include("FlexCAN_T4.h")

#    include "AstraMotors.h"


double map(double x, double in_min, double in_max, double out_min, double out_max) {
    const double run = in_max - in_min;
    if (run == 0)
    {
	    return 0;  // in_min == in_max, error
    }
    const double rise = out_max - out_min;
    const double delta = x - in_min;
    return (delta * rise) / run + out_min;
}


AstraMotors::AstraMotors(AstraCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv,
                         int setMaxSpeed, float setMaxDuty) {
    canObject = setCanObject;
    motorID = setMotorID;
    controlMode = setCtrlMode;  // 0-Speed 1-Duty Cycle
    inverted = inv;

    currentMotorSpeed = 0;
    targetMotorSpeed = 0;
    maxSpeed = setMaxSpeed;

    currentDutyCycle = 0;
    targetDutyCycle = 0;
    dutyCycleAccel = 0.05;
    maxDuty = setMaxDuty;
}

float AstraMotors::convertControllerValue(float stickValue) {
    float output = stickValue;

    if (!inverted) {
        output *= -1;
    }

    if (controlMode == 0)  // speed Control mode
    {
#    ifdef ARM
        return 0;  // speed control not implemented
#    else
        output = map(output, -1, 1, (-1 * maxSpeed), maxSpeed);
#    endif
    } else {  // duty cycle control mode
        output = map(output, -1, 1, (-1 * maxDuty), maxDuty);
    }

    return output;
}


void AstraMotors::setSpeed(float val) {  // controller input value
    if (abs(val) <= 0.02) {
        targetMotorSpeed = 0;
    } else {
        targetMotorSpeed = convertControllerValue(val);
    }
}

void AstraMotors::setDuty(float val) {  // controller input value
#    ifdef ARM
    currentDutyCycle = val;
    targetDutyCycle = val;
#    else
    if (abs(val) <= 0.02) {
        targetDutyCycle = 0;
    } else {
        targetDutyCycle = convertControllerValue(val);
    }
#    endif
}


void AstraMotors::sendDuty(float val) {
    setDuty(val);
    currentDutyCycle = targetDutyCycle;
    sendDuty();
}

void AstraMotors::accelerate() {
    UpdateForAcceleration();
#ifdef DEBUG
    Serial.print("Accelerating to ");
    Serial.println(currentDutyCycle);
#endif
    if (controlMode == 1)  // Duty cycle mode
        sendDuty();
}


void AstraMotors::UpdateForAcceleration() {
#    ifdef ARM
    currentDutyCycle = targetDutyCycle;
#    else

    if (targetDutyCycle == 0) {
        currentDutyCycle = 0;
        return;
    }
    else if (targetMotorSpeed == 0) {
        currentMotorSpeed = 0;
    }

    if(controlMode == CTRL_DUTYCYCLE) {
        const float threshold = 0.1;
        const float current = currentDutyCycle;
        const float target = targetDutyCycle;

        if (abs(target - current) <= threshold) {  // if within threshold, just set it, don't gradually accelerate
            currentDutyCycle = targetDutyCycle;
        } else if (current < target - threshold) {  // increment if below set
            currentDutyCycle += dutyCycleAccel;
        } else if (current > target + threshold) {  // decrement if above set
            currentDutyCycle -= dutyCycleAccel;
        } else {
            if ((current > 0 && target < 0) ||
                (current < 0 && target > 0))  // if sticks in opposite direction, quick stop
            {
                currentDutyCycle = 0;
                targetDutyCycle = 0;
            }
            currentDutyCycle = 0;
        }
    }
#    endif
}

void AstraMotors::parseStatus1(uint8_t frameIn[]) {
    // (RPM) Motor velocity is the first 32-bits, little endian IEEE 754 float
    uint32_t motorVel = (frameIn[3] << 24) | (frameIn[2] << 16) | (frameIn[1] << 8) | frameIn[0];
    status1.sensorVelocity = *reinterpret_cast<float*>(&motorVel);

    // (*C) Motor temperature is a dedicated byte in the data frame (4).
    status1.motorTemperature = frameIn[4];

    // (V) Motor voltage is 12-bit and comes from [5] and [6]
    uint16_t voltageMSB = ((static_cast<uint16_t>(frameIn[6]) & 0xF) << 8);  // Lower 4 bits from [6]
    uint16_t voltageLSB = frameIn[5];  // Full byte from [5]
    status1.busVoltage = static_cast<float>(voltageMSB | voltageLSB) / 128.0;  // No re-interpret

    // (A) Motor current is 12-bit and comes from [6] and [7]
    uint16_t currentMSB = (static_cast<uint16_t>(frameIn[7]) << 4);  // Full byte from [7]
    uint16_t currentLSB = ((static_cast<uint16_t>(frameIn[6]) & 0xF0) >> 4);  // Upper 4 bits from [6]
    status1.outputCurrent = static_cast<float>(currentMSB | currentLSB) / 32.0;  // No re-interpret

    // (ms) Timestamp
    status1.timestamp = millis();
}

void AstraMotors::parseStatus2(uint8_t frameIn[]) {
    // (rotations) Motor position is the first 32-bits, little endian IEEE 754 float
    uint32_t motorPos = (frameIn[3] << 24) | (frameIn[2] << 16) | (frameIn[1] << 8) | frameIn[0];
    status2.sensorPosition = *reinterpret_cast<float*>(&motorPos);

    // Ignore second half of frame

    // (ms) Timestamp
    status2.timestamp = millis();
}

#endif  // __has_include("FlexCAN_T4.h")
