/*
 * encoder.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_ENCODER_H_
#define APP_DEVICES_DRIVER_ENCODER_H_

#include <Devices/Driver/stmTimerEncoder.hpp>
#include "main.h"

class Encoder {
   public:
    Encoder(Peripheral p);
    void init();
    uint32_t getCount();

   private:
    stmTimerEncoder* _encoder;
    Peripheral _p;
};

#endif /* APP_DEVICES_DRIVER_ENCODER_H_ */
