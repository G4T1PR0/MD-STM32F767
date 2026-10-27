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
    _temp_filter_value = _mcu->adcGetValue(_p);
}

// Called every 1ms: time constant about 10ms
void batteryVoltageSensor::update() {
    LP_FILTER(_temp_filter_value, (float)_mcu->adcGetValue(_p), 0.1f);
}

float batteryVoltageSensor::getVoltage() {
    return _temp_filter_value * _raw2voltage * _voltage2batt;
}
