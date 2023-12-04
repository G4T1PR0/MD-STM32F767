/*
 * encoder.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_ENCODER_H_
#define APP_DEVICES_DRIVER_ENCODER_H_

#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class Encoder {
   public:
    Encoder(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::Peripheral_Encoder p);
    uint32_t getCount();

   private:
    baseMcuAbstractionLayer* _mcu;
    baseMcuAbstractionLayer::Peripheral_Encoder _p;
};

#endif /* APP_DEVICES_DRIVER_ENCODER_H_ */
