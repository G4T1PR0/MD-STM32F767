/*
 * A3921.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/A3921.hpp>

A3921::A3921(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::Peripheral_PWM pwm, baseMcuAbstractionLayer::Peripheral_GPIO phase, baseMcuAbstractionLayer::Peripheral_GPIO sr) {
    _mcu = mcu;
    _pwm = pwm;
    _phase = phase;
    _sr = sr;
}

void A3921::init() {
    _previousPower = 0;
    _mcu->gpioSetValue(_sr, 1);
}

void A3921::setPower(float power) {
    if (_previousPower == power) {
    } else {
        _previousPower = power;
        if (power < 0) {
            _mcu->pwmSetValue(_pwm, 0 - power);
            _mcu->gpioSetValue(_phase, 0);
        } else if (power > 0) {
            _mcu->pwmSetValue(_pwm, power);
            _mcu->gpioSetValue(_phase, 1);
        } else {
            _mcu->pwmSetValue(_pwm, 0);
            _mcu->gpioSetValue(_phase, 0);
        }
        _mcu->gpioSetValue(_sr, 1);
    }
}