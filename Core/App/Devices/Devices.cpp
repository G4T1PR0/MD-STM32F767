/*
 * Devices.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Devices.hpp>

Devices::Devices() {
    mcu = new stm32f767AbstractionLayer();

    fl_current = new currentSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::FL_Current);
    fr_current = new currentSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::FR_Current);
    st_current = new currentSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::ST_Current);
    rl_current = new currentSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::RL_Current);
    rr_current = new currentSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::RR_Current);

    batt_voltage = new batteryVoltageSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::Batt_Voltage);

    steer_angle = new steerAngleSensor(mcu, baseMcuAbstractionLayer::Peripheral_ADC::ST_Volume);

    fl_encoder = new Encoder(mcu, baseMcuAbstractionLayer::Peripheral_Encoder::FL_Encoder);
    fr_encoder = new Encoder(mcu, baseMcuAbstractionLayer::Peripheral_Encoder::FR_Encoder);
    rl_encoder = new Encoder(mcu, baseMcuAbstractionLayer::Peripheral_Encoder::RL_Encoder);
    rr_encoder = new Encoder(mcu, baseMcuAbstractionLayer::Peripheral_Encoder::RR_Encoder);

    fl_driver = new A3921(mcu, baseMcuAbstractionLayer::Peripheral_PWM::FL_PWM, baseMcuAbstractionLayer::Peripheral_GPIO::FL_PHASE, baseMcuAbstractionLayer::Peripheral_GPIO::FL_SR);
    fr_driver = new A3921(mcu, baseMcuAbstractionLayer::Peripheral_PWM::FR_PWM, baseMcuAbstractionLayer::Peripheral_GPIO::FR_PHASE, baseMcuAbstractionLayer::Peripheral_GPIO::FR_SR);
    st_driver = new A3921(mcu, baseMcuAbstractionLayer::Peripheral_PWM::ST_PWM, baseMcuAbstractionLayer::Peripheral_GPIO::ST_PHASE, baseMcuAbstractionLayer::Peripheral_GPIO::ST_SR);
    rl_driver = new A3921(mcu, baseMcuAbstractionLayer::Peripheral_PWM::RL_PWM, baseMcuAbstractionLayer::Peripheral_GPIO::RL_PHASE, baseMcuAbstractionLayer::Peripheral_GPIO::RL_SR);
    rr_driver = new A3921(mcu, baseMcuAbstractionLayer::Peripheral_PWM::RR_PWM, baseMcuAbstractionLayer::Peripheral_GPIO::RR_PHASE, baseMcuAbstractionLayer::Peripheral_GPIO::RR_SR);
}

void Devices::init() {
    mcu->init();

    fl_encoder->init();
    fr_encoder->init();
    rl_encoder->init();
    rr_encoder->init();
}

void Devices::update1ms() {
    fl_encoder->update();
    fr_encoder->update();
    rl_encoder->update();
    rr_encoder->update();
}
