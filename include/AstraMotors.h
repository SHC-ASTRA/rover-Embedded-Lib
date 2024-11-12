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
    
    int getControlMode() const;
    int getSpeed() const;  // Get the current speed
    float getDuty() const;
    float getSetDuty() const;
    int getID() const;  // Get the motor's set CAN ID

    void setSpeed(float val);  // Set the targetMotorSpeed variable
    void setDuty(float val);
    
    void identify();             // Send the identify command to the motor
    void setBrake(bool enable);  // Enable either brake (true) or coast (false) idle mode
    void sendDuty();             // Send the currently tracked duty cycle to the motor
    void sendDuty(float val);    // Send this duty cycle to the motor (Bypasses acceleration)
    void accelerate();           // Run UpdateForAcceleration() and sendDuty()
    
    void UpdateForAcceleration();  // Update the current speed to try and match targetMotorSpeed
};
