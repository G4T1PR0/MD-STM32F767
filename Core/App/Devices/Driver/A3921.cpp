/*
 * A3921.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/A3921.hpp>

A3921::A3921(stmTimerPwm* pwm, Peripheral p, GPIO_TypeDef* GPIO_PHASE_Port, uint16_t GPIO_PHASE_Pin, GPIO_TypeDef* GPIO_SR_Port, uint16_t GPIO_SR_Pin) {
    _pwm = pwm;
    _p = p;
    _GPIO_PHASE_Port = GPIO_PHASE_Port;
    _GPIO_PHASE_Pin = GPIO_PHASE_Pin;
    _GPIO_SR_Port = GPIO_SR_Port;
    _GPIO_SR_Pin = GPIO_SR_Pin;
}

void A3921::init() {
    _previousPower = 0;
    HAL_GPIO_WritePin(_GPIO_SR_Port, _GPIO_SR_Pin, GPIO_PIN_SET);
}

void A3921::setPower(float power) {
    if (_previousPower == power) {
    } else {
        _previousPower = power;
        if (power < 0) {
            _pwm->setPWM(_p, 0 - power);
            HAL_GPIO_WritePin(_GPIO_PHASE_Port, _GPIO_PHASE_Pin, GPIO_PIN_RESET);
        } else if (power > 0) {
            _pwm->setPWM(_p, power);
            HAL_GPIO_WritePin(_GPIO_PHASE_Port, _GPIO_PHASE_Pin, GPIO_PIN_SET);
        } else {
            _pwm->setPWM(_p, 0);
            HAL_GPIO_WritePin(_GPIO_PHASE_Port, _GPIO_PHASE_Pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(_GPIO_SR_Port, _GPIO_SR_Pin, GPIO_PIN_SET);
    }
}