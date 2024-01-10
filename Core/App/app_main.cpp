/*
 * app_main.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <app_main.h>
#include <Devices/Devices.hpp>
#include <HardwareController/MotorController.hpp>

Devices devices;
MotorController FL_Motor(devices.fl_driver, devices.fl_current, devices.fl_encoder);
MotorController FR_Motor(devices.fr_driver, devices.fr_current, devices.fr_encoder);
MotorController ST_Motor(devices.st_driver, devices.st_current, devices.steer_angle);
MotorController RL_Motor(devices.rl_driver, devices.rl_current, devices.rl_encoder);
MotorController RR_Motor(devices.rr_driver, devices.rr_current, devices.rr_encoder);

void app_interrupt_100us();

void app_init() {
    devices.init();

    FL_Motor.init();
    FR_Motor.init();
    ST_Motor.init();
    RL_Motor.init();
    RR_Motor.init();

    devices.mcu->interruptSetCallback(MAL::Peripheral_Interrupt::T100us, &app_interrupt_100us);
}
void app_main() {
    app_init();
    while (1) {
        FL_Motor.setMode(1);
        FL_Motor.setDuty(0);
    }
}

void app_interrupt_100us() {
    MotorController::update(&FL_Motor);
    MotorController::update(&FR_Motor);
    MotorController::update(&ST_Motor);
    MotorController::update(&RL_Motor);
    MotorController::update(&RR_Motor);
}