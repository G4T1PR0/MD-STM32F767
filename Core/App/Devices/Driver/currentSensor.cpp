/*
 * currentSensor.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/currentSensor.hpp>

#define LP_FILTER(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))
#define OFFSET_CNT 2000

currentSensor::currentSensor(MAL* mcu, MAL::P_ADC p) {
    _mcu = mcu;
    _p = p;
}

void currentSensor::init() {
    uint32_t sum = 0;
    for (int i = 0; i < OFFSET_CNT; i++) {
        sum += _mcu->adcGetValue(_p);
    }
    _offset = sum / OFFSET_CNT;
}

void currentSensor::update() {
}

#define ADC_BUFFER_SIZE 50
float currentSensor::getCurrent() {
    LP_FILTER(_temp_filter_value, (float)_mcu->adcGetValue(_p), 0.05f);
    return ((_temp_filter_value - _offset) * _raw2voltage) / _voltage2current;

    // _filter.push(_mcu->adcGetValue(_p));
    // return (_filter.get() * _raw2voltage - 1.65) / _voltage2current;

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
