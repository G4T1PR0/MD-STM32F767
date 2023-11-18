/*
 * baseAdcDriver.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/baseAdcDriver.hpp>

baseAdcDriver::baseAdcDriver(stmAdc* adc, Peripheral p) {
    _adc = adc;
    _p = p;
}

uint16_t baseAdcDriver::getRawValue() {
    return _adc->get(_p);
}

float baseAdcDriver::getVoltage() {
    return _adc->get(_p) * _raw2voltage;
}
