/*
 * HardwareController.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <HardwareController/HardwareController.hpp>

MotorController* HardwareController::FL_Motor;
MotorController* HardwareController::FR_Motor;
MotorController* HardwareController::ST_Motor;
MotorController* HardwareController::RL_Motor;
MotorController* HardwareController::RR_Motor;

HardwareController::HardwareController(Devices* devices) {
    _devices = devices;
    FL_Motor = new MotorController(_devices->fl_driver, _devices->fl_current, _devices->fl_encoder);
    FR_Motor = new MotorController(_devices->fr_driver, _devices->fr_current, _devices->fr_encoder);
    ST_Motor = new MotorController(_devices->st_driver, _devices->st_current, _devices->steer_angle);
    RL_Motor = new MotorController(_devices->rl_driver, _devices->rl_current, _devices->rl_encoder);
    RR_Motor = new MotorController(_devices->rr_driver, _devices->rr_current, _devices->rr_encoder);
}

void HardwareController::init(void) {
    FL_Motor->init();
    FR_Motor->init();
    ST_Motor->init();
    RL_Motor->init();
    RR_Motor->init();

    _devices->mcu->timerInterruptSetCallback(MAL::Peripheral_TimerInterrupt::T100us, &this->update100us);
}

void HardwareController::update(void) {
}

void HardwareController::update100us(void) {
    MotorController::update(FL_Motor);
    MotorController::update(FR_Motor);
    MotorController::update(ST_Motor);
    MotorController::update(RL_Motor);
    MotorController::update(RR_Motor);
}