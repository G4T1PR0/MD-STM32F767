/*
 * steerAngleSensor.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/steerAngleSensor.hpp>

float steerAngleSensor::getAngle() {
    return _mcu->getAdcValue(_p) * _raw2angle;
}