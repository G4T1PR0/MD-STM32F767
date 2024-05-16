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
    _driver->setBrakeEnabled(true);
}

void MotorController::init(void) {
    _mode = 0;
    _current_pid.setPID(0, 0, 0);
    _velocity_pid.setPID(0, 0, 0);
    _angle_pid.setPID(0, 0, 0);
    _currentLimit = 10;
    _maxiumCurrentLimit = 10;
}

void MotorController::update(MotorController* instance) {
    instance->_update();
}

void inline MotorController::_update(void) {
    _observedCurrent = _current->getCurrent();

    if (_observedCurrent > _maxiumCurrentLimit) {
        this->OC = true;
    } else if (_observedCurrent < -_maxiumCurrentLimit) {
        this->OC = true;
    } else {
        this->OC = false;
    }

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

            if (_observedCurrent > _currentLimit) {
                _motorInputDuty = 0;
            } else if (_observedCurrent < -_currentLimit) {
                _motorInputDuty = 0;
            }

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

            if (_observedCurrent > _currentLimit) {
                _motorInputDuty = 0;
            } else if (_observedCurrent < -_currentLimit) {
                _motorInputDuty = 0;
            }

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

            if (_observedCurrent > _currentLimit) {
                _motorInputDuty = 0;
            } else if (_observedCurrent < -_currentLimit) {
                _motorInputDuty = 0;
            }

            _driver->setDuty(_motorInputDuty);
            break;

        case 4:  // Motor Angle Control
            _pidTargetCurrent = _angle_pid.update(_targetAngle, _observedAngle);
            // _motorInputDuty = _current_pid.update(_pidTargetCurrent, _observedCurrent);

            // if (_pidTargetCurrent > 0) {
            //     if (_motorInputDuty < 0)
            //         _motorInputDuty = 0;
            // } else if (_pidTargetCurrent < 0) {
            //     if (_motorInputDuty > 0)
            //         _motorInputDuty = 0;
            // } else if (_pidTargetCurrent == 0) {
            //     _motorInputDuty = 0;
            //}

            _motorInputDuty = _pidTargetCurrent;

            if (_motorInputDuty > 0.5) {
                _motorInputDuty = 0.5;
            } else if (_motorInputDuty < -0.5) {
                _motorInputDuty = -0.5;
            }

            if (_observedCurrent > _currentLimit) {
                _motorInputDuty = 0;
            } else if (_observedCurrent < -_currentLimit) {
                _motorInputDuty = 0;
            }

            _driver->setDuty(_motorInputDuty);
            break;

        case 10:  // beep
            _beep_cnt++;

            if (_beep_cnt >= _beep_time * 20) {
                _beep_cnt = 0;
                if (_isMotorShutDown) {
                    _mode = 100;
                } else {
                    _mode = 0;
                }
            } else {
                if (_beep_cnt % (20 / _beep_freq) == 0) {
                    _beep_flag = !_beep_flag;
                }

                if (_beep_flag) {
                    _driver->setDuty(0.1);
                } else {
                    _driver->setDuty(-0.1);
                }
            }
            break;

        case 100:  // Motor Shutdown
            _driver->setDuty(0);
            break;
    }
}

void MotorController::setMode(int mode) {
    if (_mode == 100) {
        if (mode == 10) {
            _mode = 10;
        }
    } else {
        if ((mode < 0 || mode > 4) && (mode != 10 && mode != 100)) {
            _mode = 0;
            return;
        }

        if (_isSteer) {
            if (mode == 3 || mode == 10) {
                _mode = 0;
            } else {
                _mode = mode;
            }
        } else if (mode == 4) {
            _mode = 0;
        } else {
            if (mode == 100) {
                _isMotorShutDown = true;
            }
            _mode = mode;
        }
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

void MotorController::setCurrentPID(float p, float i, float d) {
    _current_pid.setPID(p, i, d);
}

void MotorController::setCurrent(float current) {
    if (_isMotorDirectionReversed) {
        _targetCurrent = -current;
    } else {
        _targetCurrent = current;
    }
}

void MotorController::setCurrentLimit(float limit) {
    _currentLimit = limit;
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

void MotorController::setAnglePID(float p, float i, float d) {
    _angle_pid.setPID(p, i, d);
}

void MotorController::setAngle(float angle) {
    if (angle >= 50) {
        angle = 50;
    } else if (angle <= -50) {
        angle = -50;
    }
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

void MotorController::setBeepTime(int time) {
    if (time < 0) {
        _beep_time = 0;
    } else {
        _beep_time = time;
    }
}

void MotorController::setBeepFreqKhz(int freq) {
    if (freq <= 0) {
        _beep_freq = 1;
    } else if (freq >= 20) {
        _beep_freq = 20;
    } else {
        _beep_freq = freq;
    }
}