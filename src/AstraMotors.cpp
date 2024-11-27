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


AstraMotors::AstraMotors(AstraCAN* setCanObject, int setMotorID, sparkMax_ctrlType setCtrlMode, bool inv,
                         int setMaxSpeed, float setMaxDuty) {
    canObject = setCanObject;
    motorID = setMotorID;
    controlMode = setCtrlMode;  // 0-Speed 1-Duty Cycle
    inverted = inv;

    currentMotorSpeed = 0;
    targetMotorSpeed = 0;
    speedAccel = 5;
    maxSpeed = setMaxSpeed;

    currentDutyCycle = 0;
    targetDutyCycle = 0;
    dutyCycleAccel = 0.05;
    maxDuty = setMaxDuty;

#   ifdef TESTBED
    gearBox = 64;  // default to 64:1 for Testbed
#   else
    gearBox = 100;  // default to 100:1 for Core
#   endif
    rotatingToPos = false;
}

float AstraMotors::convertControllerValue(float stickValue) {
    float output = stickValue;

    if (!inverted) {
        output *= -1;
    }

    if (controlMode == sparkMax_ctrlType::kVelocity)  // speed Control mode
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
    if (controlMode == sparkMax_ctrlType::kDutyCycle)  // Duty cycle mode
        sendDuty();
    else if (controlMode == sparkMax_ctrlType::kVelocity || controlMode == sparkMax_ctrlType::kSmartVelocity)  // Speed mode
        sendSpeed();
}


void AstraMotors::UpdateForAcceleration() {
#    ifdef ARM
    currentDutyCycle = targetDutyCycle;
#    else

    if (controlMode == sparkMax_ctrlType::kDutyCycle && targetDutyCycle == 0) {
        currentDutyCycle = 0;
        return;
    }
    else if ((controlMode == sparkMax_ctrlType::kVelocity || controlMode == sparkMax_ctrlType::kSmartVelocity) && targetMotorSpeed == 0) {
        currentMotorSpeed = 0;
        return;
    }

    if (rotatingToPos) {
        if (millis() - status2.timestamp > 100
            || (direction() == 1 && status2.sensorPosition > targetPos - 1)
            || (direction() == -1 && status2.sensorPosition < targetPos + 1))
        {
            stop();
#ifdef DEBUG
            Serial.println("Stopping rotation.");
#endif
        }
        return;
    }

    if (controlMode == sparkMax_ctrlType::kDutyCycle) {
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
    } else if (controlMode == sparkMax_ctrlType::kSmartVelocity) {
        currentMotorSpeed = targetMotorSpeed;
    } else {
        const float threshold = 5;
        const float current = currentMotorSpeed;
        const float target = targetMotorSpeed;

        if (abs(target - current) <= threshold) {  // if within threshold, just set it, don't gradually accelerate
            currentMotorSpeed = targetMotorSpeed;
        } else if (current < target - threshold) {  // increment if below set
            currentMotorSpeed += speedAccel;
        } else if (current > target + threshold) {  // decrement if above set
            currentMotorSpeed -= speedAccel;
        } else {
            if ((current > 0 && target < 0) ||
                (current < 0 && target > 0))  // if sticks in opposite direction, quick stop
            {
                currentMotorSpeed = 0;
                targetMotorSpeed = 0;
            }
            currentMotorSpeed = 0;
        }
    }
#    endif
}

void AstraMotors::parseStatus0(uint8_t frameIn[]) {
    // (A) Applied output is 16-bit and comes from [0] and [1]
    uint16_t outputMSB = (frameIn[1] << 8);  // Full byte from [1]
    uint16_t outputLSB = frameIn[0];  // Full byte from [0]
    status0.appliedOutput = static_cast<float>(outputMSB | outputLSB) / 32767.0;  // No re-interpret

    // (bits) Faults are 16-bit and come from [2] and [3]
    status0.faults = (frameIn[3] << 8) | frameIn[2];

    // (bits) Sticky faults are 16-bit and come from [4] and [5]
    status0.stickyFaults = (frameIn[5] << 8) | frameIn[4];

    // (bits) Lock is a dedicated bit in the data frame (7)
    status0.lock = (frameIn[6] >> 2) & 0x3;

    // (bit) Inverted is a dedicated bit in the data frame (7)
    status0.isInverted = (frameIn[6] >> 1) & 0x1;

    // (ms) Timestamp
    status0.timestamp = millis();
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

void AstraMotors::turnByDeg(float deg) {
    rotatingToPos = true;
    targetPos = status2.sensorPosition + ((deg / 360.0) * gearBox);

#ifdef DEBUG
    Serial.print("Turning to pos: ");
    Serial.println(targetPos);
#endif

    if (controlMode == sparkMax_ctrlType::kDutyCycle) {
        const float dutyCycle = 0.15;  // Arbitrary for now
        if (deg < 0)
            sendDuty(dutyCycle);
        else
            sendDuty(-1 * dutyCycle);
    } else if (controlMode == sparkMax_ctrlType::kVelocity) {
        const float speed = 100;  // Arbitrary for now
        if (deg < 0)
            setSpeed(speed);
        else
            setSpeed(-1 * speed);
    }
}

#endif  // __has_include("FlexCAN_T4.h")
