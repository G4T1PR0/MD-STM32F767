/*
 * MotorController.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <HardwareController/MotorController.hpp>

MotorController::MotorController(A3921* A3921, currentSensor* currentSensor, Encoder* encoder) {
    _driver = A3921;
    _current = currentSensor;
    _encoder = encoder;
    _isSteer = false;
}

MotorController::MotorController(A3921* A3921, currentSensor* currentSensor, steerAngleSensor* steerAngleSensor) {
    _driver = A3921;
    _current = currentSensor;
    _steerAngle = steerAngleSensor;
    _isSteer = true;
}

void MotorController::init(void) {
    _mode = 0;
}

void MotorController::update(void) {
    switch (_mode) {
        case 0:  // Motor OFF
            _driver->setDuty(0);
            break;

        case 1:  // Motor PWM Control
            _motorInputDuty = _targetDuty;
            _driver->setDuty(_motorInputDuty);
            break;

        case 2:  // Motor Current Control
            _pidTargetCurrent = _targetCurrent;
            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _current->getCurrent());
            _driver->setDuty(_motorInputDuty);
            break;

        case 3:  // Motor Velocity Control
            _pidTargetCurrent = _velocity_pid.update(_targetVelocity, _encoder->getCount());
            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _current->getCurrent());
            _driver->setDuty(_motorInputDuty);

            break;

        case 4:  // Motor Angle Control
            _pidTargetCurrent = _angle_pid.update(_targetAngle, _steerAngle->getAngle());
            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _current->getCurrent());
            _driver->setDuty(_motorInputDuty);
            break;
    }
}

void MotorController::setMode(int mode) {
    if (mode < 0 || mode > 4) {
        _mode = 0;
        return;
    }
    if (_isSteer) {
        if (mode == 3) {
            _mode = 0;
        }
    } else {
        _mode = mode;
    }
}

void MotorController::setDuty(float duty) {
    _targetDuty = duty;
}

void MotorController::setCurrent(float current) {
    _targetCurrent = current;
}

void MotorController::setVelocity(float velocity) {
    _targetVelocity = velocity;
}

void MotorController::setAngle(float angle) {
    _targetAngle = angle;
}
