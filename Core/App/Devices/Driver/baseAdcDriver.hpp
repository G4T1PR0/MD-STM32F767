/*
 * baseAdcDriver.h
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_BASEADCDRIVER_H_
#define APP_DEVICES_DRIVER_BASEADCDRIVER_H_
#include <main.h>
#include <Devices/Driver/stmAdc.hpp>

class baseAdcDriver {
   public:
    baseAdcDriver(stmAdc* adc, Peripheral p);
    uint16_t getRawValue();
    float getVoltage();

   protected:
    stmAdc* _adc;
    Peripheral _p;

    const float _raw2voltage = 3.3f / (1 << 12);
};

#endif /* APP_DEVICES_DRIVER_BASEADCDRIVER_H_ */
