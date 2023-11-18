/*
 * currentSensor.hpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_current_SENSOR_HPP_
#define APP_DEVICES_DRIVER_current_SENSOR_HPP_
#include <Devices/Driver/baseAdcDriver.hpp>

class currentSensor : public baseAdcDriver {
   public:
    using baseAdcDriver::baseAdcDriver;
    float getCurrent();

   private:
    const float _voltage2current = 0.033f;
};

#endif /* APP_DEVICES_DRIVER_current_SENSOR_HPP_ */
