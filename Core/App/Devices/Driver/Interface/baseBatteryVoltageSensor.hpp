/*
 * baseBatteryVoltageSensor.hpp
 *
 *  Created on: Dec 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_INTERFACE_BASEBATTERYVOLTAGESENSOR_HPP_
#define APP_DEVICES_DRIVER_INTERFACE_BASEBATTERYVOLTAGESENSOR_HPP_

#include <stdint.h>
class baseBatteryVoltageSensor {
   public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual float getVoltage() = 0;
};

#endif /* APP_DEVICES_DRIVER_INTERFACE_BASEBATTERYVOLTAGESENSOR_HPP_ */