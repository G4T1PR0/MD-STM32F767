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

void app_interrupt_50us();

unsigned int led_cnt = 0;
int led_index = 0;

unsigned int debug_cnt = 0;

unsigned int motor_debug_cnt = 0;

unsigned int cmd_send_cnt = 0;

#define DEBUG_LOG_NUM 10000

static volatile float debug_log[DEBUG_LOG_NUM][5] = {0};
unsigned int log_mode = 0;
unsigned int log_cnt = 0;

struct md_error_t {
    enum Error_Code {
        BATT_VOLTAGE_ERROR = 0,
        FL_CURRENT_ERROR = 1,
        FR_CURRENT_ERROR = 2,
        ST_CURRENT_ERROR = 3,
        RL_CURRENT_ERROR = 4,
        RR_CURRENT_ERROR = 5,

        FL_CONTROLL_ERROR = 6,
        FR_CONTROLL_ERROR = 7,
        ST_CONTROLL_ERROR = 8,
        RL_CONTROLL_ERROR = 9,
        RR_CONTROLL_ERROR = 10,

        Error_End = 11,
    };

    bool Error[Error_Code::Error_End];

} __attribute__((packed));

union md_error_union {
    md_error_t s;
    uint32_t raw;
};

md_error_union md_error;

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

void app_init() {
    mcu.init();

    mcu.waitMs(100);

    fl_current.init();
    fr_current.init();
    st_current.init();
    rl_current.init();
    rr_current.init();

    mcu.waitMs(100);

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

    mcu.interruptSetCallback(MAL::P_Interrupt::T50us, &app_interrupt_50us);

    mcu.waitMs(500);
    printf("\033[H");
    printf("\033[2J");
    printf(R"EOF(
 __  __  ____ ____    __  __ ____       ____ _____ __  __ 
|  \/  |/ ___|  _ \  |  \/  |  _ \     / ___|_   _|  \/  |
| |\/| | |   | |_) | | |\/| | | | |____\___ \ | | | |\/| |
| |  | | |___|  _ <  | |  | | |_| |_____|__) || | | |  | |
|_|  |_|\____|_| \_\ |_|  |_|____/     |____/ |_| |_|  |_|

)EOF");

    RL_Motor.setBeepTime(250);
    RL_Motor.setBeepFreqKhz(4);
    RL_Motor.setMode(10);
    mcu.waitMs(400);

    RL_Motor.setBeepTime(100);
    RL_Motor.setBeepFreqKhz(6);
    RL_Motor.setMode(10);
    mcu.waitMs(150);
    RL_Motor.setMode(10);
    mcu.waitMs(150);

    for (unsigned int i = 0; i < 10; i++) {
        mcu.gpioSetValue(led[i], 1);
    }

    FL_Motor.setMotorDirection(false);
    FL_Motor.setMode(0);
    FL_Motor.setCurrentPID(1, 0, 0);
    FL_Motor.setCurrent(0);

    ST_Motor.setMotorDirection(false);
    ST_Motor.setCurrentPID(2, 0, 0);
    ST_Motor.setAnglePID(0.04, 0, 0);
    ST_Motor.setAngle(0);
    ST_Motor.setMode(0);
    ST_Motor.setCurrentLimit(8);

    FR_Motor.setMotorDirection(true);
    FR_Motor.setMode(0);
    FR_Motor.setCurrentPID(1, 0, 0);
    FR_Motor.setCurrent(0);

    RL_Motor.setMotorDirection(true);
    RL_Motor.setMode(0);
    RL_Motor.setCurrentPID(1, 0, 0);
    FR_Motor.setCurrent(0);

    RR_Motor.setMotorDirection(false);
    RR_Motor.setMode(0);
    RR_Motor.setCurrentPID(1, 0, 0);
    FR_Motor.setCurrent(0);

    printf("\x1b[32m[Main Thread]\x1b[39m Initialization Complete\n");
}

void app_main() {
    app_init();

    unsigned int led_mode = 0;

    unsigned int motor_mode = 1;

    log_mode = 1;

    // FL_Motor.setMode(2);
    // FL_Motor.setCurrent(0.6);
    // FR_Motor.setMode(2);
    // FR_Motor.setCurrent(0.6);
    // RL_Motor.setMode(2);
    // RL_Motor.setCurrent(0.6);
    // RR_Motor.setMode(2);
    // RR_Motor.setCurrent(0.6);

    while (1) {
        if (cmd.isConnectionLost) {
            for (unsigned int i = 0; i < 10; i++) {
                mcu.gpioSetValue(led[i], 1);
            }

            mcu.gpioSetValue(MAL::P_GPIO::LED_1, 0);
            mcu.gpioSetValue(MAL::P_GPIO::LED_2, 0);
            mcu.gpioSetValue(MAL::P_GPIO::LED_3, 0);
            mcu.gpioSetValue(MAL::P_GPIO::LED_4, 0);
            mcu.gpioSetValue(MAL::P_GPIO::LED_5, 0);

        } else if (FL_Motor.getMode() != 0 || FR_Motor.getMode() != 0 || ST_Motor.getMode() != 0 || RL_Motor.getMode() != 0 || RR_Motor.getMode() != 0) {
            float d = (FL_Motor.getDuty() + FR_Motor.getDuty() + ST_Motor.getDuty() + RL_Motor.getDuty() + RR_Motor.getDuty()) / 5;

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
        } else {
            if (led_cnt > 80 * 10) {
                led_cnt = 0;
                if (led_mode == 0) {
                    led_index++;
                    if (led_index >= 10) {
                        led_index = 9;
                        led_mode = 1;
                    }
                } else {
                    led_index--;
                    if (led_index <= -1) {
                        led_index = 0;
                        led_mode = 0;
                    }
                }
            }

            mcu.gpioSetValue(led[led_index], 0);

            for (unsigned int i = 0; i < 10; i++) {
                if (mcu.gpioGetValue(led[i]) == 0 && i != led_index) {
                    mcu.gpioSetValue(led[i], 1);
                }
            }
        }

        md_error.s.Error[md_error.s.BATT_VOLTAGE_ERROR] = batt_voltage.getVoltage() < 6;

        md_error.s.Error[md_error.s.FL_CURRENT_ERROR] = FL_Motor.OC;
        md_error.s.Error[md_error.s.FR_CURRENT_ERROR] = FR_Motor.OC;
        md_error.s.Error[md_error.s.ST_CURRENT_ERROR] = ST_Motor.OC;
        md_error.s.Error[md_error.s.RL_CURRENT_ERROR] = RL_Motor.OC;
        md_error.s.Error[md_error.s.RR_CURRENT_ERROR] = RR_Motor.OC;

        md_error.s.Error[md_error.s.FL_CONTROLL_ERROR] = FL_Motor.CE;
        md_error.s.Error[md_error.s.FR_CONTROLL_ERROR] = FR_Motor.CE;
        md_error.s.Error[md_error.s.ST_CONTROLL_ERROR] = ST_Motor.CE;
        md_error.s.Error[md_error.s.RL_CONTROLL_ERROR] = RL_Motor.CE;
        md_error.s.Error[md_error.s.RR_CONTROLL_ERROR] = RR_Motor.CE;

        if (md_error.raw != 0) {
            for (const auto& i : mcs) {
                i->setMode(100);
            }

            RL_Motor.setBeepTime(100);
            RL_Motor.setBeepFreqKhz(6);
            RL_Motor.setMode(10);
            mcu.waitMs(150);
            RL_Motor.setMode(10);
            mcu.waitMs(150);
            RL_Motor.setMode(10);
            mcu.waitMs(150);
            RL_Motor.setMode(10);
            mcu.waitMs(150);
            while (1) {
                printf("\x1b[31m[Main Thread]\x1b[39m Error: %u Bit: ", md_error.raw);
                for (int i = 0; i < md_error.s.Error_End; i++) {
                    printf("%u ", md_error.s.Error[i]);
                }
                printf("\n");
                mcu.waitMs(1000);
            }
        }

        if (cmd_send_cnt > 10) {  // 10ms
            cmd_send_cnt = 0;
            cmd.send();  // Send Feedback
        }

        cmd.update();  // Parse Command

        // if (log_mode == 0) {
        //     motor_mode = 0;
        //     log_mode = 2;
        // }

        // switch (motor_mode) {
        //     case 0:
        //         // RL_Motor.setMode(0);
        //         printf("DEBUG_LOG\r\n\r\n\r\n\r\n");
        //         for (int i = 0; i < DEBUG_LOG_NUM; i++) {
        //             printf("%f, %f, %f, %f, %f\r\n", debug_log[i][0], debug_log[i][1], debug_log[i][2], debug_log[i][3], debug_log[i][4]);
        //         }
        //         motor_mode = 10;

        //         break;
        //     case 1:
        //         // RL_Motor.setVelocity(500);
        //         FL_Motor.setCurrent(0.01);
        //         FR_Motor.setCurrent(0.05);
        //         // ST_Motor.setCurrent(0.05);
        //         ST_Motor.setAngle(50);
        //         RL_Motor.setCurrent(0.5);
        //         RR_Motor.setCurrent(2);
        //         //  RL_Motor.setDuty(0.01);
        //         if (motor_debug_cnt > 10 * 2000) {
        //             motor_debug_cnt = 0;
        //             motor_mode = 2;
        //         }
        //         break;

        //     case 2:
        //         // RL_Motor.setVelocity(300);
        //         FL_Motor.setCurrent(0.01);
        //         FR_Motor.setCurrent(0.09);
        //         // ST_Motor.setCurrent(-0.05);
        //         ST_Motor.setAngle(-50);
        //         RL_Motor.setCurrent(-0.5);
        //         RR_Motor.setCurrent(-2);
        //         //  RL_Motor.setDuty(-0.01);
        //         if (motor_debug_cnt > 10 * 2000) {
        //             motor_debug_cnt = 0;
        //             motor_mode = 1;
        //         }
        //         break;

        //     case 10:
        //         break;

        //     default:
        //         break;
        // }

        if (debug_cnt > 100 * 10) {
            debug_cnt = 0;
            // printf("input duty: %f freq: %f\r\n", (mcu.inputPwmGetDuty(MAL::P_IPWM::ST_IPWM)), mcu.inputPwmGetFrequency(MAL::P_IPWM::ST_IPWM));
            // printf("fld: %f frd: %f rld: %f rrd: %f\r\n", FL_Motor.getDuty(), FR_Motor.getDuty(), RL_Motor.getDuty(), RR_Motor.getDuty());
            // printf("mode: %d bus_voltage: %f duty: %f t_current: %f o_current: %f dt: %f dt_avg %f\r\n", FR_Motor.getMode(), batt_voltage.getVoltage(), FR_Motor.getDuty(), FR_Motor.getTargetCurrent(), FR_Motor.getCurrent(), FR_Motor.D_dt, FR_Motor.D_dt_avg);
            // printf("std: %f std_dt: %f std_dt_avg: %f\r\n", FL_Motor.getDuty(), FL_Motor.D_dt, (float)FL_Motor.D_dt_avg);
            // printf("flc: %f frc: %f stc: %f rlc: %f rrc: %f\r\n", FL_Motor.getCurrent(), FR_Motor.getCurrent(), ST_Motor.getCurrent(), RL_Motor.getCurrent(), RR_Motor.getCurrent());
            //  printf("duty: %f t_current: %f o_current: %f t_velocity: %f o_velocity: %f\r\n", debug_log[log_cnt][0], debug_log[log_cnt][1], debug_log[log_cnt][2], debug_log[log_cnt][3], debug_log[log_cnt][4]);
            // printf("mode: %d raw_angle: %f angle: %f duty: %f t_current: %f o_current: %f \n", motor_mode, steer_angle.getRawAngle(), ST_Motor.getAngle(), ST_Motor.getDuty(), ST_Motor.getTargetCurrent(), ST_Motor.getCurrent());
            // printf("dt: %f dtdt %f\n", ST_Motor.D_dt, ST_Motor.D_dtdt);
        }
    }
}

unsigned int update1ms_cnt = 0;

void app_interrupt_50us() {
    update1ms_cnt++;

    cmd.parsePwm();

    if (update1ms_cnt > 20) {
        update1ms_cnt = 0;
        fl_encoder.update();
        fr_encoder.update();
        rl_encoder.update();
        rr_encoder.update();
        cmd_send_cnt++;
        cmd.cnt1ms++;
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
