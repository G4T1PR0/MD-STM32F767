/*
 * A3921.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/A3921.hpp>

A3921::A3921(MAL* mcu, MAL::P_PWM pwm, MAL::P_GPIO phase, MAL::P_GPIO sr) {
    _mcu = mcu;
    _pwm = pwm;
    _phase = phase;
    _sr = sr;
}

void A3921::init() {
    _previousDuty = 0;
}

void A3921::setDuty(float duty) {
    if (_previousDuty == duty) {
    } else {
        _previousDuty = duty;
        if (duty < 0) {
            _mcu->gpioSetValue(_phase, 0);
        } else if (duty > 0) {
            _mcu->gpioSetValue(_phase, 1);
        }

        if (duty < 0) {
            duty = -duty;
        }

        if (duty > 1) {
            duty = 1;
        } else if (duty < 0) {
            duty = 0;
        }

        _mcu->pwmSetDuty(_pwm, duty);
        _mcu->gpioSetValue(_sr, _brakeEnabled);
        // _mcu->gpioSetValue(_sr, 1);
    }
}

void A3921::setBrakeEnabled(bool enabled) {
    _brakeEnabled = enabled;
}