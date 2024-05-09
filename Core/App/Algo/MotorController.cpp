/*
 * MotorController.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <Algo/MotorController.hpp>

MotorController::MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseEncoder* encoder) {
    _driver = driver;
    _current = currentSensor;
    _encoder = encoder;
    _isSteer = false;
}

MotorController::MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseSteerAngleSensor* steerAngleSensor) {
    _driver = driver;
    _current = currentSensor;
    _steerAngle = steerAngleSensor;
    _isSteer = true;
}

void MotorController::init(void) {
    _mode = 0;
    _current_pid.setPID(2, 0, 0);
    _velocity_pid.setPID(0.01, 0, 0);
}

void MotorController::update(MotorController* instance) {
    instance->_update();
}

void inline MotorController::_update(void) {
    _observedCurrent = _current->getCurrent();
    if (_isSteer) {
        _observedAngle = _steerAngle->getAngle();
    } else {
        if (_isMotorConnectionReversed) {
            _observedVelocity = -_encoder->getCount();
        } else {
            _observedVelocity = _encoder->getCount();
        }
    }
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

            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _observedCurrent);
            if (_pidTargetCurrent > 0) {
                if (_motorInputDuty < 0)
                    _motorInputDuty = 0;
            } else if (_pidTargetCurrent < 0) {
                if (_motorInputDuty > 0)
                    _motorInputDuty = 0;
            } else if (_pidTargetCurrent == 0) {
                _motorInputDuty = 0;
            }
            // if (_pidTargetCurrent < 0.001 && _pidTargetCurrent > -0.001) {
            //     _motorInputDuty = 0;
            // }

            _driver->setDuty(_motorInputDuty);
            break;

        case 3:  // Motor Velocity Control
            _pidTargetCurrent = _velocity_pid.update(_targetVelocity, _observedVelocity);
            if (_targetVelocity > 0) {
                if (_pidTargetCurrent < 0)
                    _pidTargetCurrent = 0;
            } else if (_targetVelocity < 0) {
                if (_pidTargetCurrent > 0)
                    _pidTargetCurrent = 0;
            } else if (_targetVelocity == 0) {
                _pidTargetCurrent = 0;
            }

            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _observedCurrent);
            if (_pidTargetCurrent > 0) {
                if (_motorInputDuty < 0)
                    _motorInputDuty = 0;
            } else if (_pidTargetCurrent < 0) {
                if (_motorInputDuty > 0)
                    _motorInputDuty = 0;
            } else if (_pidTargetCurrent == 0) {
                _motorInputDuty = 0;
            }
            // if (_pidTargetCurrent < 0.001 && _pidTargetCurrent > -0.001) {
            //     _motorInputDuty = 0;
            // }

            _driver->setDuty(_motorInputDuty);
            break;

        case 4:  // Motor Angle Control
            _pidTargetCurrent = _angle_pid.update(_targetAngle, _observedAngle);
            _motorInputDuty = _current_pid.update(_pidTargetCurrent, _observedCurrent);
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
        } else {
            _mode = mode;
        }
    } else if (mode == 4) {
        _mode = 0;
    } else {
        _mode = mode;
    }
}

int MotorController::getMode() {
    return _mode;
}

void MotorController::setDuty(float duty) {
    if (_isMotorDirectionReversed) {
        _targetDuty = -duty;
    } else {
        _targetDuty = duty;
    }
}

float MotorController::getDuty() {
    if (_isMotorDirectionReversed) {
        return -_motorInputDuty;
    } else {
        return _motorInputDuty;
    }
}

void MotorController::setCurrent(float current) {
    if (_isMotorDirectionReversed) {
        _targetCurrent = -current;
    } else {
        _targetCurrent = current;
    }
}

float MotorController::getCurrent() {
    if (_isMotorDirectionReversed) {
        return -_observedCurrent;
    } else {
        return _observedCurrent;
    }
}

float MotorController::getTargetCurrent() {
    if (_isMotorDirectionReversed) {
        return -_pidTargetCurrent;
    } else {
        return _pidTargetCurrent;
    }
}

void MotorController::setVelocity(float velocity) {
    if (_isMotorDirectionReversed) {
        _targetVelocity = -velocity;
    } else {
        _targetVelocity = velocity;
    }
}

float MotorController::getVelocity() {
    if (_isMotorDirectionReversed) {
        return -_observedVelocity;
    } else {
        return _observedVelocity;
    }
}

float MotorController::getTargetVelocity() {
    if (_isMotorDirectionReversed) {
        return -_targetVelocity;
    } else {
        return _targetVelocity;
    }
}

void MotorController::setAngle(float angle) {
    _targetAngle = angle;
}

float MotorController::getAngle() {
    return _observedAngle;
}

float MotorController::getTargetAngle() {
    return _targetAngle;
}

void MotorController::setMotorConnectionReversed(bool isReversed) {
    _isMotorConnectionReversed = isReversed;
}

void MotorController::setMotorDirection(bool d) {
    _isMotorDirectionReversed = d;
}