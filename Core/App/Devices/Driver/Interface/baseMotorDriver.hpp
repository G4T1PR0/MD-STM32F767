/*
 * baseMotorDriver.hpp
 *
 *  Created on: Dec 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_INTERFACE_BASEMOTORDRIVER_HPP_
#define APP_DEVICES_DRIVER_INTERFACE_BASEMOTORDRIVER_HPP_

#include <stdint.h>

class baseMotorDriver {
   public:
    virtual void init() = 0;
    virtual void setDuty(float duty) = 0;
};

#endif /* APP_DEVICES_DRIVER_INTERFACE_BASEMOTORDRIVER_HPP_ */