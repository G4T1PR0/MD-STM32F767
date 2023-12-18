/*
 * baseSteerAngleSensor.hpp
 *
 *  Created on: Dec 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_INTERFACE_BASESTEERANGLESENSOR_HPP_
#define APP_DEVICES_DRIVER_INTERFACE_BASESTEERANGLESENSOR_HPP_

#include <stdint.h>

class baseSteerAngleSensor {
   public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual float getAngle() = 0;
};

#endif /* APP_DEVICES_DRIVER_INTERFACE_BASESTEERANGLESENSOR_HPP_ */