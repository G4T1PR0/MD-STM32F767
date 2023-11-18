/*
 * Devices.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Devices.hpp>

Devices::Devices() {
    adc = new stmAdc();
    pwm = new stmTimerPwm();
    encoder = new stmTimerEncoder();

    fl_current = new currentSensor(adc, Peripheral::FL_Current);
    fr_current = new currentSensor(adc, Peripheral::FR_Current);
    st_current = new currentSensor(adc, Peripheral::ST_Current);
    rl_current = new currentSensor(adc, Peripheral::RL_Current);
    rr_current = new currentSensor(adc, Peripheral::RR_Current);

    batt_voltage = new batteryVoltageSensor(adc, Peripheral::Batt_Voltage);

    steer_angle = new steerAngleSensor(adc, Peripheral::ST_Volume);

    fl_driver = new A3921(pwm, Peripheral::FL_PWM, Motor_FL_PHASE_GPIO_Port, Motor_FL_PHASE_Pin, Motor_FL_SR_GPIO_Port, Motor_FL_SR_Pin);
    fr_driver = new A3921(pwm, Peripheral::FR_PWM, Motor_FR_PHASE_GPIO_Port, Motor_FR_PHASE_Pin, Motor_FR_SR_GPIO_Port, Motor_FR_SR_Pin);
    st_driver = new A3921(pwm, Peripheral::ST_PWM, Motor_ST_PHASE_GPIO_Port, Motor_ST_PHASE_Pin, Motor_ST_SR_GPIO_Port, Motor_ST_SR_Pin);
    rl_driver = new A3921(pwm, Peripheral::RL_PWM, Motor_RL_PHASE_GPIO_Port, Motor_RL_PHASE_Pin, Motor_RL_SR_GPIO_Port, Motor_RL_SR_Pin);
    rr_driver = new A3921(pwm, Peripheral::RR_PWM, Motor_RR_PHASE_GPIO_Port, Motor_RR_PHASE_Pin, Motor_RR_SR_GPIO_Port, Motor_RR_SR_Pin);

    fl_encoder = new Encoder(Peripheral::FL_Encoder);
    fr_encoder = new Encoder(Peripheral::FR_Encoder);
    rl_encoder = new Encoder(Peripheral::RL_Encoder);
    rr_encoder = new Encoder(Peripheral::RR_Encoder);
}

void Devices::init() {
    adc->init();
    pwm->init();
    encoder->init();
}
