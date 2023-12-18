/*
 * steerAngleSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/steerAngleSensor.hpp>

steerAngleSensor::steerAngleSensor(MAL* mcu, MAL::Peripheral_ADC p) {
    _mcu = mcu;
    _p = p;
}

void steerAngleSensor::init() {
}

void steerAngleSensor::update() {
}
float steerAngleSensor::getAngle() {
    return _mcu->adcGetValue(_p) * _raw2angle;
}