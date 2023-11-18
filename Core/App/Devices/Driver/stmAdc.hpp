/*
 * stmAdc.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_stmAdc_HPP_
#define APP_DEVICES_DRIVER_stmAdc_HPP_
#include <Devices/stmHalConfig.hpp>
#include "adc.h"
#include "main.h"

class stmAdc {
   public:
    stmAdc();
    void init(void);
    uint16_t get(Peripheral p);

   private:
    ADC_HandleTypeDef* _handle;
    static uint16_t _data[16];
};

#endif /* APP_DEVICES_DRIVER_stmAdc_HPP_ */