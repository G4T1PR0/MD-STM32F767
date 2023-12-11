/*
 * MotorController.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_

#include <Devices/Devices.hpp>
#include <Lib/pid.hpp>

class MotorController {
   public:
    MotorController(A3921* A3921, currentSensor* currentSensor, Encoder* encoder);
    MotorController(A3921* A3921, currentSensor* currentSensor, steerAngleSensor* steerAngleSensor);
    void init(void);
    void update(void);

    void setMode(int mode);

    void setDuty(float duty);

    void setCurrent(float current);

    void setVelocity(float velocity);

    void setAngle(float angle);

   private:
    A3921* _driver;
    currentSensor* _current;
    Encoder* _encoder;
    steerAngleSensor* _steerAngle;

    PID _current_pid;
    PID _velocity_pid;
    PID _angle_pid;

    bool _isSteer;
    int _mode;

    float _motorInputDuty;
    float _pidTargetCurrent;

    float _targetDuty;
    float _targetCurrent;
    float _targetVelocity;
    float _targetAngle;
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_ */
