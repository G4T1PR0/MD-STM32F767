/*
 * batteryVoltageSensor.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_
#define APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_

#include <Devices/Driver/baseAdcDriver.hpp>

class batteryVoltageSensor : public baseAdcDriver {
   public:
    using baseAdcDriver::baseAdcDriver;
    float getVoltage();

   private:
    const float _voltage2batt = 16 / 3.3;
};

#endif /* APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_ */
