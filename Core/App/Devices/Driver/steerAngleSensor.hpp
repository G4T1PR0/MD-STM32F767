/*
 * steerAngleSensor.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_
#define APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_

#include <Devices/Driver/baseAdcDriver.hpp>

class steerAngleSensor : public baseAdcDriver {
   public:
    using baseAdcDriver::baseAdcDriver;
    float getAngle();

   private:
    const float _raw2angle = 0.1;
};

#endif /* APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_ */
