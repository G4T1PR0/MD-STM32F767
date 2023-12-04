/*
 * Devices.hpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_Devices_H_
#define APP_DEVICES_Devices_H_

#include <Devices/Driver/A3921.hpp>
#include <Devices/Driver/batteryVoltageSensor.hpp>
#include <Devices/Driver/currentSensor.hpp>
#include <Devices/Driver/encoder.hpp>
#include <Devices/Driver/steerAngleSensor.hpp>
#include <Devices/McuAbstractionLayer/stm32f767AbstractionLayer.hpp>

class Devices {
   public:
    Devices();
    void init();

    stm32f767AbstractionLayer* mcu;

    currentSensor* fl_current;
    currentSensor* fr_current;
    currentSensor* st_current;
    currentSensor* rl_current;
    currentSensor* rr_current;

    batteryVoltageSensor* batt_voltage;
    steerAngleSensor* steer_angle;

    A3921* fl_driver;
    A3921* fr_driver;
    A3921* st_driver;
    A3921* rl_driver;
    A3921* rr_driver;

    Encoder* fl_encoder;
    Encoder* fr_encoder;
    Encoder* rl_encoder;
    Encoder* rr_encoder;
};

#endif /* APP_DEVICES_Devices_H_ */
