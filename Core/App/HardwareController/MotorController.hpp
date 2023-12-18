/*
 * MotorController.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_

#include <Devices/Devices.hpp>
#include <Devices/Driver/Interface/baseCurrentSensor.hpp>
#include <Devices/Driver/Interface/baseEncoder.hpp>
#include <Devices/Driver/Interface/baseMotorDriver.hpp>
#include <Devices/Driver/Interface/baseSteerAngleSensor.hpp>
#include <Lib/pid.hpp>

class MotorController {
   public:
    MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseEncoder* encoder);
    MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseSteerAngleSensor* steerAngleSensor);
    void init(void);
    void update(void);

    void setMode(int mode);

    void setDuty(float duty);

    void setCurrent(float current);

    void setVelocity(float velocity);

    void setAngle(float angle);

   private:
    baseMotorDriver* _driver;
    baseCurrentSensor* _current;
    baseEncoder* _encoder;
    baseSteerAngleSensor* _steerAngle;

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
