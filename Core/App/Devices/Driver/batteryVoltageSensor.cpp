/*
 * batteryVoltageSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/batteryVoltageSensor.hpp>

#define LP_FILTER(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))

batteryVoltageSensor::batteryVoltageSensor(MAL* mcu, MAL::P_ADC p) {
    _mcu = mcu;
    _p = p;
}

void batteryVoltageSensor::init() {
}

void batteryVoltageSensor::update() {
}

float batteryVoltageSensor::getVoltage() {
    LP_FILTER(_temp_filter_value, (float)_mcu->adcGetValue(_p), 0.5f);
    return _mcu->adcGetValue(_p) * _raw2voltage * _voltage2batt;
}
