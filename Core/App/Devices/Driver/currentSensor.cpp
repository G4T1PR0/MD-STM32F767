/*
 * currentSensor.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/currentSensor.hpp>

currentSensor::currentSensor(MAL* mcu, MAL::Peripheral_ADC p) {
    _mcu = mcu;
    _p = p;
}

void currentSensor::init() {
}
void currentSensor::update() {
}

float currentSensor::getCurrent() {
    _filter.push((_mcu->adcGetValue(_p) * _raw2voltage - 1.65) / _voltage2current);
    return _filter.get();
    // return (_mcu->adcGetValue(_p) * _raw2voltage - 1.65) / _voltage2current;
}
