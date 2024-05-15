/*
 * steerAngleSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/steerAngleSensor.hpp>

#define LP_FILTER(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))

steerAngleSensor::steerAngleSensor(MAL* mcu, MAL::P_ADC p) {
    _mcu = mcu;
    _p = p;
}

void steerAngleSensor::init() {
}

void steerAngleSensor::update() {
}
float steerAngleSensor::getAngle() {
    // LP_FILTER(_temp_filter_value, (float)_mcu->adcGetValue(_p), 1.0f);
    return (2000 - _mcu->adcGetValue(_p)) * _raw2angle;
    // return 45 - (((float)_mcu->adcGetValue(_p) - 654) * _raw2angle);
    //  return (float)_mcu->adcGetValue(_p);
}

float steerAngleSensor::getRawAngle() {
    return (float)_mcu->adcGetValue(_p);
}