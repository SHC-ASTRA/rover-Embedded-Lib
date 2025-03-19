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


class AstraMotors {
    int motorID;  // REV motor ID
    sparkMax_ctrlType controlMode;  // 0 - Duty cycle  1 - Speed
    bool inverted;  // Inverts the speed of the motor, this should be true for right wheels

    int currentMotorSpeed;  // Current speed of the motor
    int targetMotorSpeed;   // What the speed of the motor should be
    float speedAccel;
    int maxSpeed;

    float currentDutyCycle;
    float targetDutyCycle;
    float dutyCycleAccel;
    float maxDuty;

    int gearBox;  // Gearbox ratio attached to motor; e.g. for 64:1, use 64

    bool rotatingToPos;
    float targetPos;

   public:

    motorStatus0 status0;
    motorStatus1 status1;  // Keep public for now for testing
    motorStatus2 status2;
    
    /**
     * @brief Default constructor for a REV motor controller
     * 
     * @param setMotorID REV motor ID for this motor
     * @param setCtrlMode Either CTRL_SPEED or CTRL_DUTYCYCLE for controlling via RPM or percent speed
     * @param inv Whether or not to invert the motor's direction (right wheels)
     * @param setMaxSpeed Max RPM for speed control mode
     * @param setMaxDuty Max percent speed for duty cycle control mode [-1, 1]
     */
    AstraMotors(int setMotorID, sparkMax_ctrlType setCtrlMode = sparkMax_ctrlType::kDutyCycle, bool inv = false, int setMaxSpeed = 100, float setMaxDuty = 1.0);

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
    
    inline sparkMax_ctrlType getControlMode() const {
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

    inline int getGearBox() const {
        return gearBox;
    }

    //---------------------------------------------//
    //  Setters
    //---------------------------------------------//

    // Gearbox ratio attached to motor; e.g. for 64:1, use 64
    inline void setGearBox(int ratio) {
        gearBox = ratio;
    }

    // Set the targetMotorSpeed variable
    void setSpeed(float val);
    void setDuty(float val);
    inline void setCurrent(float val) {  // No acceleration for current control
        if (controlMode != sparkMax_ctrlType::kCurrent)
            return;
        CAN_sendControl(motorID, sparkMax_ctrlType::kCurrent, val);
    }

    // Update the current speed to try and match targetMotorSpeed
    void UpdateForAcceleration();

    void parseStatus0(uint8_t frameIn[]);
    void parseStatus1(uint8_t frameIn[]);
    void parseStatus2(uint8_t frameIn[]);

    //---------------------------------------------//
    //  Controlling physical motor
    //---------------------------------------------//
    
    // Send the identify command to the motor
    inline void identify() {
        CAN_identifySparkMax(motorID);
    }
    // Enable either brake (true) or coast (false) idle mode
    inline void setBrake(bool enable) {
        CAN_setParameter(motorID, sparkMax_ConfigParameter::kIdleMode, sparkMax_ParameterType::kUint32, static_cast<uint32_t>(enable));
    }
    // Send the currently tracked duty cycle to the motor
    inline void sendDuty() {
        CAN_sendControl(motorID, sparkMax_ctrlType::kDutyCycle, currentDutyCycle);
    }

    inline void sendSpeed() {
        CAN_sendControl(motorID, sparkMax_ctrlType::kVelocity, currentMotorSpeed);
    }
    
    void sendDuty(float val);    // Send this duty cycle to the motor (bypasses acceleration)
    void sendSpeed(float val);    // Send this speed to the motor (bypasses acceleration)
    void accelerate();           // Run UpdateForAcceleration() and sendDuty()

    void turnByDeg(float deg);     // Turn the motor by deg degrees
    void turnToDeg(float deg);     // Turn the motor to deg degrees
    
    // Stop motor; does not activate brake mode
    inline void stop() {
        rotatingToPos = false;
        sendDuty(0);
    }

    inline int direction() {
        if (controlMode == sparkMax_ctrlType::kDutyCycle) {
            if (currentDutyCycle == 0)
                return 0;
            else
                return (currentDutyCycle > 0) ? 1 : -1;
        } else {
            if (currentMotorSpeed == 0)
                return 0;
            else
                return (currentMotorSpeed > 0) ? 1 : -1;
        }
    }

    inline bool isRotToPos() {
        return rotatingToPos;
    }
};
