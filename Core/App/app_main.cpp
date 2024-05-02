/*
 * app_main.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <app_main.h>
#include <vector>

#include <Devices/Driver/A3921.hpp>
#include <Devices/Driver/batteryVoltageSensor.hpp>
#include <Devices/Driver/currentSensor.hpp>
#include <Devices/Driver/encoder.hpp>
#include <Devices/Driver/steerAngleSensor.hpp>
#include <Devices/McuAbstractionLayer/stm32halAbstractionLayer.hpp>

#include <Algo/CommandReciever.hpp>
#include <Algo/MotorController.hpp>

stm32halAbstractionLayer mcu;

currentSensor fl_current(&mcu, MAL::P_ADC::FL_Current);
currentSensor fr_current(&mcu, MAL::P_ADC::FR_Current);
currentSensor st_current(&mcu, MAL::P_ADC::ST_Current);
currentSensor rl_current(&mcu, MAL::P_ADC::RL_Current);
currentSensor rr_current(&mcu, MAL::P_ADC::RR_Current);

batteryVoltageSensor batt_voltage(&mcu, MAL::P_ADC::Batt_Voltage);
steerAngleSensor steer_angle(&mcu, MAL::P_ADC::ST_Volume);

Encoder fl_encoder(&mcu, MAL::P_Encoder::FL_Encoder);
Encoder fr_encoder(&mcu, MAL::P_Encoder::FR_Encoder);
Encoder rl_encoder(&mcu, MAL::P_Encoder::RL_Encoder);
Encoder rr_encoder(&mcu, MAL::P_Encoder::RR_Encoder);

A3921 fl_driver(&mcu, MAL::P_PWM::FL_PWM, MAL::P_GPIO::FL_PHASE, MAL::P_GPIO::FL_SR);
A3921 fr_driver(&mcu, MAL::P_PWM::FR_PWM, MAL::P_GPIO::FR_PHASE, MAL::P_GPIO::FR_SR);
A3921 st_driver(&mcu, MAL::P_PWM::ST_PWM, MAL::P_GPIO::ST_PHASE, MAL::P_GPIO::ST_SR);
A3921 rl_driver(&mcu, MAL::P_PWM::RL_PWM, MAL::P_GPIO::RL_PHASE, MAL::P_GPIO::RL_SR);
A3921 rr_driver(&mcu, MAL::P_PWM::RR_PWM, MAL::P_GPIO::RR_PHASE, MAL::P_GPIO::RR_SR);

MotorController FL_Motor(&fl_driver, &fl_current, &fl_encoder);
MotorController FR_Motor(&fr_driver, &fr_current, &fr_encoder);
MotorController ST_Motor(&st_driver, &st_current, &steer_angle);
MotorController RL_Motor(&rl_driver, &rl_current, &rl_encoder);
MotorController RR_Motor(&rr_driver, &rr_current, &rr_encoder);

std::vector<MotorController*> mcs = {&FL_Motor, &FR_Motor, &ST_Motor, &RL_Motor, &RR_Motor};
CommandReciever cmd(&mcu, mcs);

void app_interrupt_100us();

unsigned int led_cnt = 0;
int led_index = 0;

unsigned int debug_cnt = 0;

unsigned int motor_debug_cnt = 0;

#define DEBUG_LOG_NUM 10000

static volatile float debug_log[DEBUG_LOG_NUM][5] = {0};
unsigned int log_mode = 0;
unsigned int log_cnt = 0;

void app_init() {
    mcu.init();

    fl_encoder.init();
    fr_encoder.init();
    rl_encoder.init();
    rr_encoder.init();

    fl_driver.init();
    fr_driver.init();
    st_driver.init();
    rl_driver.init();
    rr_driver.init();

    FL_Motor.init();
    FR_Motor.init();
    ST_Motor.init();
    RL_Motor.init();
    RR_Motor.init();

    cmd.init();

    mcu.interruptSetCallback(MAL::P_Interrupt::T100us, &app_interrupt_100us);
}

MAL::P_GPIO led[]{
    MAL::P_GPIO::LED_1,
    MAL::P_GPIO::LED_2,
    MAL::P_GPIO::LED_3,
    MAL::P_GPIO::LED_4,
    MAL::P_GPIO::LED_5,
    MAL::P_GPIO::LED_6,
    MAL::P_GPIO::LED_7,
    MAL::P_GPIO::LED_8,
    MAL::P_GPIO::LED_9,
    MAL::P_GPIO::LED_10,
};

void app_main() {
    app_init();

    for (unsigned int i = 0; i < 10; i++) {
        mcu.gpioSetValue(led[i], 1);
    }

    unsigned int led_mode = 0;

    unsigned int motor_mode = 1;

    mcu.pwmSetFrequency(MAL::P_PWM::FR_PWM, 50000);
    mcu.pwmSetFrequency(MAL::P_PWM::RR_PWM, 50000);

    log_mode = 1;
    FL_Motor.setMotorDirection(false);
    FL_Motor.setMode(0);
    // FL_Motor.setMode(2);
    // FL_Motor.setCurrent(0.6);
    //  FL_Motor.setDuty(0.01);

    ST_Motor.setMotorDirection(true);
    ST_Motor.setMode(0);

    FR_Motor.setMotorDirection(true);
    FR_Motor.setMode(0);
    // FR_Motor.setDuty(0.01);

    RL_Motor.setMotorDirection(true);
    RL_Motor.setMode(2);
    // RL_Motor.setDuty(0.01);

    RR_Motor.setMotorDirection(false);
    RR_Motor.setMode(0);
    // RR_Motor.setDuty(0.01);

    while (1) {
        float d = RL_Motor.getDuty();
        if (d < 0) {
            d = -d;
        }

        float dd = 0;
        for (int i = 9; -1 < i; i--) {
            dd += 0.1;
            if (dd > d) {
                mcu.gpioSetValue(led[i], 1);
            } else {
                mcu.gpioSetValue(led[i], 0);
            }
        }

        // cmd.update();

        if (log_mode == 0) {
            motor_mode = 0;
            log_mode = 2;
        }

        switch (motor_mode) {
            case 0:
                // RL_Motor.setMode(0);
                printf("DEBUG_LOG\r\n\r\n\r\n\r\n");
                for (int i = 0; i < DEBUG_LOG_NUM; i++) {
                    printf("%f, %f, %f, %f, %f\r\n", debug_log[i][0], debug_log[i][1], debug_log[i][2], debug_log[i][3], debug_log[i][4]);
                }
                motor_mode = 10;

                break;
            case 1:
                // RL_Motor.setVelocity(500);
                RL_Motor.setCurrent(0.1);
                //  RL_Motor.setDuty(0.01);
                if (motor_debug_cnt > 10 * 2000) {
                    motor_debug_cnt = 0;
                    motor_mode = 2;
                }
                break;

            case 2:
                // RL_Motor.setVelocity(300);
                RL_Motor.setCurrent(-0.1);
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

        // if (led_cnt > 70 * 10) {
        //     led_cnt = 0;
        //     if (led_mode == 0) {
        //         led_index++;
        //         if (led_index >= 10) {
        //             led_index = 9;
        //             led_mode = 1;
        //         }
        //     } else {
        //         led_index--;
        //         if (led_index <= -1) {
        //             led_index = 0;
        //             led_mode = 0;
        //         }
        //     }
        // }

        // mcu.gpioSetValue(led[led_index], 0);

        // for (unsigned int i = 0; i < 10; i++) {
        //     if (mcu.gpioGetValue(led[i]) == 0 && i != led_index) {
        //         mcu.gpioSetValue(led[i], 1);
        //     }
        // }

        if (debug_cnt > 100 * 10) {
            debug_cnt = 0;
            printf("mode: %d bus_voltage: %f duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", motor_mode, batt_voltage.getVoltage(), RL_Motor.getDuty(), RL_Motor.getTargetCurrent(), RL_Motor.getCurrent(), RL_Motor.getTargetVelocity(), RL_Motor.getVelocity());
            // printf("duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", debug_log[log_cnt][0], debug_log[log_cnt][1], debug_log[log_cnt][2], debug_log[log_cnt][3], debug_log[log_cnt][4]);
        }
    }
}

unsigned int update1ms_cnt = 0;

void app_interrupt_100us() {
    update1ms_cnt++;
    if (update1ms_cnt > 10) {
        update1ms_cnt = 0;
        fl_encoder.update();
        fr_encoder.update();
        rl_encoder.update();
        rr_encoder.update();
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
    led_cnt++;
}
