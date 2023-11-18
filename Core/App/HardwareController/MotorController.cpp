/*
 * MotorController.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <HardwareController/MotorController.hpp>

MotorController::MotorController(A3921* A3921, currentSensor* currentSensor, Encoder* encoder) {
    _driver = A3921;
    _current = currentSensor;
    _encoder = encoder;
}

MotorController::MotorController(A3921* A3921, currentSensor* currentSensor, steerAngleSensor* steerAngleSensor) {
    _driver = A3921;
    _current = currentSensor;
    _steerAngle = steerAngleSensor;
}

void MotorController::init(void) {
}
