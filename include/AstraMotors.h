/**
 * @file AstraMotors.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Outlines class for controlling Rev Sparkmax motors
 *
 */
#pragma once

#include <cmath>

#include "AstraREVCAN.h"

enum motorCtrlMode {
    CTRL_SPEED = 0,
    CTRL_DUTYCYCLE = 1
};

// Clamps x between out_min and out_max using the expected input min and max
// Used for controller input
long map(long x, long in_min, long in_max, long out_min, long out_max);

class AstraMotors {
    AstraCAN* canObject;
    int motorID;  // REV motor ID
    int controlMode;  // 0- Speed  1-Duty Cycle
    bool inverted;  // Inverts the speed of the motor, this should be true for right wheels

    int currentMotorSpeed;  // Current speed of the motor
    int targetMotorSpeed;   // What the speed of the motor should be
    int maxSpeed;

    float currentDutyCycle;
    float targetDutyCycle;
    float dutyCycleAccel;
    float maxDuty;

   public:

    motorStatus1 status1;  // Keep public for now for testing
    
    /**
     * @brief Default constructor for a REV motor controller
     * 
     * @param setCanObject Can0 or TwaiCan
     * @param setMotorID REV motor ID for this motor
     * @param setCtrlMode Either CTRL_SPEED or CTRL_DUTYCYCLE for controlling via RPM or percent speed
     * @param inv Whether or not to invert the motor's direction (right wheels)
     * @param setMaxSpeed Max RPM for speed control mode
     * @param setMaxDuty Max percent speed for duty cycle control mode [-1, 1]
     */
    AstraMotors(AstraCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed = 100, float setMaxDuty = 1.0);

    /**
     * @brief Clamps a joystick value between max and min speed or duty
     * 
     * @param stickvalue Input value from the controller joystick between -1 and 1
     * @return float Clamped value able to be sent to the motors
     */
    float convertControllerValue(float stickvalue);
    
    //---------------------------------------------//
    //  Getters
    //---------------------------------------------//
    
    inline int getControlMode() const {
        return controlMode;
    }
    // Get the current speed
    inline int getSpeed() const {
        return currentMotorSpeed;
    }
    inline float getDuty() const {
        return currentDutyCycle;
    }
    inline float getSetDuty() const {
        return targetDutyCycle;
    }
    // Get the motor's set CAN ID
    inline int getID() const {
        return motorID;
    }

    //---------------------------------------------//
    //  Setters
    //---------------------------------------------//

    // Set the targetMotorSpeed variable
    void setSpeed(float val);
    void setDuty(float val);

    // Update the current speed to try and match targetMotorSpeed
    void UpdateForAcceleration();

    //---------------------------------------------//
    //  Controlling physical motor
    //---------------------------------------------//
    
    // Send the identify command to the motor
    inline void identify() {
        CAN_identifySparkMax(motorID, *canObject);
    }
    // Enable either brake (true) or coast (false) idle mode
    inline void setBrake(bool enable) {
        CAN_setParameter(motorID, sparkMax_ConfigParameter::kIdleMode, sparkMax_ParameterType::kUint32, static_cast<uint32_t>(enable), *canObject);
    }
    // Send the currently tracked duty cycle to the motor
    inline void sendDuty() {
        CAN_sendDutyCycle(motorID, currentDutyCycle, *canObject);
    }
    
    void sendDuty(float val);    // Send this duty cycle to the motor (Bypasses acceleration)
    void accelerate();           // Run UpdateForAcceleration() and sendDuty()
};
