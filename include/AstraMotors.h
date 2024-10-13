/**
 * @file AstraMotors.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Outlines class for controlling Rev Sparkmax motors
 *
 */
#pragma once

#include <cmath>

#include "AstraCAN.h"


long map(long x, long in_min, long in_max, long out_min, long out_max);

class AstraMotors {
    int controlMode;  // 0- Speed  1-Duty Cycle

    int currentMotorSpeed;  // Current speed of the motor
    int setMotorSpeed;      // What the speed of the motor should be

    float currentDutyCycle;
    float setDutyCycle;
    float dutyCycleAccel;

    int motorID;

    int maxSpeed;
    float maxDuty;

    bool inverted;  // Inverts the speed of the motor, this should be -1 for motor 3 and 4

    AstraFCAN canObject;

   public:
    // AstraMotors();                                      // Startup function
    // AstraMotors(bool inv);                              // Creates an object with inverted = -1
    AstraMotors(AstraFCAN setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty);

    float convertControllerValue(float stickvalue);  // Converts the joystick value -1 < 0 < 1 to 1700 < 1500 < 1300
    // void setMotorMultiplier(float val);                 // Set the speedMultiplier variable
    
    int getControlMode() const;
    int getSpeed() const;      // Get the current speed
    float getDuty() const;
    float getSetDuty() const;
    int getID() const;         // Get the motor's set CAN ID

    void setSpeed(float val);  // Set the setMotorSpeed variable
    void setDuty(float val);
    void setBrake(bool enable);   // Enable/disable the brake mode
    

    void UpdateForAcceleration();  // Update the current speed to try and match the setMotorSpeed
                                   // variable
};
