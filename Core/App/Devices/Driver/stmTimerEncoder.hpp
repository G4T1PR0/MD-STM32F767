/*
 * stmTimerEncoder.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_STMTIMERENCODER_HPP_
#define APP_DEVICES_DRIVER_STMTIMERENCODER_HPP_

#include <Devices/stmHalConfig.hpp>
#include "main.h"
#include "tim.h"

class stmTimerEncoder {
   public:
    stmTimerEncoder();
    void init(void);

    void setCount(Peripheral p, uint32_t count);
    uint32_t getCount(Peripheral p);
};

#endif /* APP_DEVICES_DRIVER_STMTIMERENCODER_HPP_ */
