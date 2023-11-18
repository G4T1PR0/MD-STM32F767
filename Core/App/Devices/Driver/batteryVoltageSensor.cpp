/*
 * batteryVoltageSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/batteryVoltageSensor.hpp>

float batteryVoltageSensor::getVoltage() {
    return _adc->get(_p) * _raw2voltage * _voltage2batt;
}
