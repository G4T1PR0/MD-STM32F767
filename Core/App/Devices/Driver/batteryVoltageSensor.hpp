/*
 * batteryVoltageSensor.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_
#define APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_

#include <Devices/Driver/Interface/baseBatteryVoltageSensor.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class batteryVoltageSensor : public baseBatteryVoltageSensor {
   public:
    batteryVoltageSensor(MAL* mcu, MAL::P_ADC p);

    virtual void init();
    virtual void update();
    float getVoltage();

   private:
    MAL* _mcu;
    MAL::P_ADC _p;

    const float _raw2voltage = 3.3f / (1 << 12);
    const float _voltage2batt = 16 / 3.3;

    float _temp_filter_value;
};

#endif /* APP_DEVICES_DRIVER_BATTERY_VOLTAGE_SENSOR_HPP_ */
