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

#endif  // __has_include("FlexCAN_T4.h")
