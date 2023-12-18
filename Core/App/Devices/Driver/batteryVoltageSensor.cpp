/*
 * batteryVoltageSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/batteryVoltageSensor.hpp>

batteryVoltageSensor::batteryVoltageSensor(MAL* mcu, MAL::Peripheral_ADC p) {
    _mcu = mcu;
    _p = p;
}

void batteryVoltageSensor::init() {
}

void batteryVoltageSensor::update() {
}

float batteryVoltageSensor::getVoltage() {
    return _mcu->adcGetValue(_p) * _raw2voltage * _voltage2batt;
}
