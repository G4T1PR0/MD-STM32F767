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

#define DEBUG_LOG_NUM 10000

static volatile float debug_log[DEBUG_LOG_NUM][5] = {0};
unsigned int log_mode = 0;
unsigned int log_cnt = 0;

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

    unsigned int motor_mode = 1;

    devices.mcu->pwmSetFrequency(MAL::Peripheral_PWM::FR_PWM, 50000);
    devices.mcu->pwmSetFrequency(MAL::Peripheral_PWM::RR_PWM, 50000);

    log_mode = 1;
    FL_Motor.setMotorConnectionReversed(true);
    FL_Motor.setMode(3);
    ST_Motor.setMotorConnectionReversed(true);
    ST_Motor.setMode(2);
    FR_Motor.setMotorConnectionReversed(true);
    FR_Motor.setMode(3);
    RL_Motor.setMotorConnectionReversed(true);
    RL_Motor.setMode(3);
    RR_Motor.setMotorConnectionReversed(true);
    RR_Motor.setMode(3);

    while (1) {
        float d = RL_Motor.getDuty();
        if (d < 0) {
            d = -d;
        }

        float dd = 0;
        for (int i = 9; -1 < i; i--) {
            dd += 0.1;
            if (dd > d) {
                devices.mcu->gpioSetValue(led[i], 1);
            } else {
                devices.mcu->gpioSetValue(led[i], 0);
            }
        }

        if (log_mode == 0) {
            motor_mode = 0;
            log_mode = 2;
        }

        switch (motor_mode) {
            case 0:
                RL_Motor.setMode(0);
                printf("DEBUG_LOG\r\n\r\n\r\n\r\n");
                for (int i = 0; i < DEBUG_LOG_NUM; i++) {
                    printf("%f, %f, %f, %f, %f\r\n", debug_log[i][0], debug_log[i][1], debug_log[i][2], debug_log[i][3], debug_log[i][4]);
                }
                motor_mode = 10;

                break;
            case 1:
                RL_Motor.setVelocity(500);
                // RL_Motor.setCurrent(0.1);
                //  RL_Motor.setDuty(0.01);
                if (motor_debug_cnt > 10 * 2000) {
                    motor_debug_cnt = 0;
                    motor_mode = 2;
                }
                break;

            case 2:
                RL_Motor.setVelocity(300);
                // RL_Motor.setCurrent(-0.1);
                //  RL_Motor.setDuty(-0.01);
                if (motor_debug_cnt > 10 * 2000) {
                    motor_debug_cnt = 0;
                    motor_mode = 1;
                }
                break;

            case 10:
                break;

            default:
                break;
        }

        // led_cnt++;
        // if (led_cnt > 1000) {
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

        if (debug_cnt > 100 * 10) {
            debug_cnt = 0;
            printf("cnt: %d duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", log_cnt, RL_Motor.getDuty(), RL_Motor.getTargetCurrent(), RL_Motor.getCurrent(), RL_Motor.getTargetVelocity(), RL_Motor.getVelocity());
            // printf("duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", debug_log[log_cnt][0], debug_log[log_cnt][1], debug_log[log_cnt][2], debug_log[log_cnt][3], debug_log[log_cnt][4]);
        }
    }
}

unsigned int update1ms_cnt = 0;

void app_interrupt_100us() {
    update1ms_cnt++;
    if (update1ms_cnt > 10) {
        update1ms_cnt = 0;
        devices.update1ms();
    }
    MotorController::update(&FL_Motor);
    MotorController::update(&FR_Motor);
    MotorController::update(&ST_Motor);
    MotorController::update(&RL_Motor);
    MotorController::update(&RR_Motor);

    // if (log_mode == 1) {
    //     debug_log[log_cnt][0] = FR_Motor.getDuty();
    //     debug_log[log_cnt][1] = FR_Motor.getTargetCurrent();
    //     debug_log[log_cnt][2] = FR_Motor.getCurrent();
    //     debug_log[log_cnt][3] = FR_Motor.getTargetVelocity();
    //     debug_log[log_cnt][4] = FR_Motor.getVelocity();
    //     log_cnt++;
    //     if (log_cnt >= DEBUG_LOG_NUM) {
    //         log_mode = 0;
    //     }
    // }

    debug_cnt++;
    motor_debug_cnt++;
}