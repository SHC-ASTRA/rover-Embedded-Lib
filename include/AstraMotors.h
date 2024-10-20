/**
 * @file AstraMotors.h
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Outlines class for controlling Rev Sparkmax motors
 *
 */
#pragma once

#include <cmath>

#include "AstraCAN.h"

// Clamps x between out_min and out_max using the expected input min and max
// Used for controller input
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

    AstraCAN* canObject;

   public:
    // AstraMotors();                                      // Startup function
    // AstraMotors(bool inv);                              // Creates an object with inverted = -1
    
    /**
     * @brief Default constructor for a REV motor controller
     * 
     * @param setCanObject Can0
     * @param setMotorID 
     * @param setCtrlMode 
     * @param inv 
     * @param setMaxSpeed 
     * @param setMaxDuty 
     */
    AstraMotors(AstraCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty);

    /**
     * @brief Clamps a joystick value between max and min speed or duty
     * 
     * @param stickvalue Input value from the controller joystick between -1 and 1
     * @return float Clamped value able to be sent to the motors
     */
    float convertControllerValue(float stickvalue);
    // void setMotorMultiplier(float val);                 // Set the speedMultiplier variable
    
    int getControlMode() const;
    int getSpeed() const;      // Get the current speed
    float getDuty() const;
    float getSetDuty() const;
    int getID() const;         // Get the motor's set CAN ID

    void setSpeed(float val);  // Set the setMotorSpeed variable
    void setDuty(float val);
    
    void identify();  // Send the identify command to the motor
    void setBrake(bool enable);   // Enable/disable the brake mode
    void sendDuty();  // Send the currently tracked duty cycle to the motor
    void sendDuty(float val);  // Send this duty cycle to the motor (Bypasses acceleration)
    void accelerate();  // Run UpdateForAcceleration() and sendDuty()
    

    void UpdateForAcceleration();  // Update the current speed to try and match the setMotorSpeed
                                   // variable
};
