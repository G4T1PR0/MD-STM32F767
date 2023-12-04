/*
 * baseAdcDriver.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/baseAdcDriver.hpp>

baseAdcDriver::baseAdcDriver(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::Peripheral_ADC p) {
    _mcu = mcu;
    _p = p;
}

uint16_t baseAdcDriver::getRawValue() {
    return _mcu->getAdcValue(_p);
}

float baseAdcDriver::getVoltage() {
    return _mcu->getAdcValue(_p) * _raw2voltage;
}
