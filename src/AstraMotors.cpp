/**
 * @file AstraMotors.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Implements class for controlling Rev Sparkmax motors
 *
 */

#if __has_include("ESP32-TWAI-CAN.hpp") || __has_include("FlexCAN_T4.h")

#    include "AstraMotors.h"


long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AstraMotors::AstraMotors(AstraFCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv,
                         int setMaxSpeed, float setMaxDuty) {
    canObject = setCanObject;

    controlMode = setCtrlMode;  // 0-Speed 1-Duty Cycle

    currentDutyCycle = 0;
    setDutyCycle = 0;

    currentMotorSpeed = 0;
    setMotorSpeed = 0;

    maxSpeed = setMaxSpeed;
    maxDuty = setMaxDuty;

    dutyCycleAccel = 0.05;

    motorID = setMotorID;

    inverted = inv;
}

// Add the funky graph
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


int AstraMotors::getControlMode() const {
    return controlMode;
}


void AstraMotors::setSpeed(float val) {  // controller input value
    if (abs(val) <= 0.02) {
        setMotorSpeed = 0;
    } else {
        setMotorSpeed = convertControllerValue(val);
    }
}

int AstraMotors::getSpeed() const {
    return currentMotorSpeed;
}

int AstraMotors::getID() const {
    return motorID;
}


void AstraMotors::setDuty(float val) {  // controller input value
#    ifdef ARM
    currentDutyCycle = val;
    setDutyCycle = val;
#    else
    if (abs(val) <= 0.02) {
        setDutyCycle = 0;
    } else {
        setDutyCycle = convertControllerValue(val);
    }
#    endif
}

float AstraMotors::getDuty() const {
    return currentDutyCycle;
}
float AstraMotors::getSetDuty() const {
    return setDutyCycle;
}

void AstraMotors::identify() {
    identifyDevice(*canObject, motorID);
}

void AstraMotors::setBrake(bool enable) {
    setParameter(*canObject, motorID, 6, (int)enable);
}

void AstraMotors::sendDuty() {
    sendDutyCycle(*canObject, motorID, currentDutyCycle);
}

void AstraMotors::sendDuty(float val) {
    setDuty(val);
    currentDutyCycle = setDutyCycle;
    sendDuty();
}


void AstraMotors::UpdateForAcceleration() {
#    ifdef ARM
    currentDutyCycle = setDutyCycle;
#    else
    float dCThreshold = 0.1;
    float cD = currentDutyCycle;
    float sD = setDutyCycle;

    // if(controlMode == 1){
    if (setDutyCycle != 0) {
        if ((cD <= sD + 0.1) &&
            (cD >=
             sD - 0.1)) {  // if within 0.1 of desired. Just set it, don't gradually accelerate
            currentDutyCycle = setDutyCycle;
        } else if (cD < sD - dCThreshold) {  // increment if below set
            currentDutyCycle += dutyCycleAccel;
        } else if (cD > sD + dCThreshold) {  // decrement if above set
            currentDutyCycle -= dutyCycleAccel;
        } else {
            if ((cD > 0 && sD < 0) ||
                (cD < 0 && sD > 0))  // if sticks in opposite direction, quick stop
            {
                currentDutyCycle = 0;
                setDutyCycle = 0;
            }
            currentDutyCycle = 0;
        }
    } else {  // if set 0
        currentDutyCycle = 0;
    }
    //}
#    endif
}

#endif  // __has_include("FlexCAN_T4.h")
