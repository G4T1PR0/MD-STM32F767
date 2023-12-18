/*
 * baseEncoder.hpp
 *
 *  Created on: Dec 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_INTERFACE_BASEENCODER_HPP_
#define APP_DEVICES_DRIVER_INTERFACE_BASEENCODER_HPP_

#include <stdint.h>

class baseEncoder {
   public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual int32_t getCount() = 0;
    virtual int32_t getTotalCnt() = 0;
    virtual int32_t getAcceleration() = 0;
};

#endif /* APP_DEVICES_DRIVER_INTERFACE_BASEENCODER_HPP_ */