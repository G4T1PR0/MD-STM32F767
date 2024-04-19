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

unsigned int led_cnt = 0;
unsigned int led_index = 0;

unsigned int debug_cnt = 0;

unsigned int motor_debug_cnt = 0;

void app_init() {
    devices.init();

    FL_Motor.init();
    FR_Motor.init();
    ST_Motor.init();
    RL_Motor.init();
    RR_Motor.init();

    devices.mcu->interruptSetCallback(MAL::Peripheral_Interrupt::T100us, &app_interrupt_100us);
}

MAL::Peripheral_GPIO led[]{
    MAL::Peripheral_GPIO::LED_1,
    MAL::Peripheral_GPIO::LED_2,
    MAL::Peripheral_GPIO::LED_3,
    MAL::Peripheral_GPIO::LED_4,
    MAL::Peripheral_GPIO::LED_5,
    MAL::Peripheral_GPIO::LED_6,
    MAL::Peripheral_GPIO::LED_7,
    MAL::Peripheral_GPIO::LED_8,
    MAL::Peripheral_GPIO::LED_9,
    MAL::Peripheral_GPIO::LED_10,
};

void app_main() {
    app_init();

    for (unsigned int i = 0; i < 10; i++) {
        devices.mcu->gpioSetValue(led[i], 1);
    }

    unsigned int led_mode = 0;

    unsigned int motor_mode = 0;

    devices.mcu->pwmSetFrequency(MAL::Peripheral_PWM::FR_PWM, 40000);
    while (1) {
        float d = FR_Motor.getDuty();
        if (d < 0) {
            d = -d;
        }

        float dd = 0;
        for (unsigned int i = 9; 0 < i; i--) {
            dd += 0.1;
            if (dd > d) {
                devices.mcu->gpioSetValue(led[i], 1);
            } else {
                devices.mcu->gpioSetValue(led[i], 0);
            }
        }

        FR_Motor.setMode(2);

        switch (motor_mode) {
            case 0:

                FR_Motor.setCurrent(-0.1);
                // FR_Motor.setDuty(0.2);
                if (motor_debug_cnt > 10 * 1000) {
                    motor_debug_cnt = 0;
                    motor_mode = 1;
                }
                break;

            case 1:
                FR_Motor.setCurrent(-0.6);
                // FR_Motor.setCurrent(-0.1);
                //  FR_Motor.setDuty(-0.2);
                if (motor_debug_cnt > 10 * 1000) {
                    motor_debug_cnt = 0;
                    motor_mode = 0;
                }
                break;

            default:
                break;
        }

        // led_cnt++;
        // if (led_cnt > 6000) {
        //     led_cnt = 0;
        //     if (led_mode == 0) {
        //         led_index++;
        //         if (led_index >= 10) {
        //             led_index = 9;
        //             led_mode = 1;
        //         }
        //     } else {
        //         led_index--;
        //         if (led_index <= 0) {
        //             led_index = 0;
        //             led_mode = 0;
        //         }
        //     }
        // }

        // devices.mcu->gpioSetValue(led[led_index], 0);

        // for (unsigned int i = 0; i < 10; i++) {
        //     if (devices.mcu->gpioGetValue(led[i]) == 0 && i != led_index) {
        //         devices.mcu->gpioSetValue(led[i], 1);
        //     }
        // }

        // if (debug_cnt > 100 * 10) {
        //     debug_cnt = 0;
        //     printf("duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", FR_Motor.getDuty(), FR_Motor.getTargetCurrent(), FR_Motor.getCurrent(), FR_Motor.getTargetVelocity(), FR_Motor.getVelocity());
        // }
    }
}

unsigned int update1ms_cnt = 0;

void app_interrupt_100us() {
    update1ms_cnt++;
    if (update1ms_cnt > 10) {
        update1ms_cnt = 0;
        devices.update1ms();
    }
    // MotorController::update(&FL_Motor);
    MotorController::update(&FR_Motor);
    // MotorController::update(&ST_Motor);
    // MotorController::update(&RL_Motor);
    // MotorController::update(&RR_Motor);
    debug_cnt++;
    motor_debug_cnt++;
}