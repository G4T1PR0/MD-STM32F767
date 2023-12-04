/*
 * currentSensor.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/currentSensor.hpp>

float currentSensor::getCurrent() {
    return (_mcu->getAdcValue(_p) * _raw2voltage - 1.65) / _voltage2current;
}
