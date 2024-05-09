/*
 * currentSensor.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/currentSensor.hpp>

currentSensor::currentSensor(MAL* mcu, MAL::P_ADC p) {
    _mcu = mcu;
    _p = p;
}

void currentSensor::init() {
}
void currentSensor::update() {
}

#define ADC_BUFFER_SIZE 50
float currentSensor::getCurrent() {
    _filter.push(_mcu->adcGetValue(_p));
    return (_filter.get() * _raw2voltage - 1.65) / _voltage2current;
    // return (_mcu->adcGetValue(_p) * _raw2voltage - 1.65) / _voltage2current;
    // uint16_t buffer[ADC_BUFFER_SIZE];
    // _mcu->adcGetBufferValue(_p, buffer, ADC_BUFFER_SIZE);
    // unsigned long long sum = 0;
    // for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    //     sum += buffer[i];
    // }

    // float avg = sum / ADC_BUFFER_SIZE;

    // return (avg * _raw2voltage - 1.65) / _voltage2current;
}
