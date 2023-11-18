/*
 * stmTimerPwm.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_STMTIMERPWM_HPP_
#define APP_DEVICES_DRIVER_STMTIMERPWM_HPP_

#include <Devices/stmHalConfig.hpp>
#include "main.h"
#include "tim.h"

class stmTimerPwm {
   public:
    stmTimerPwm();
    void init(void);
    void setPWM(Peripheral p, float duty);

   private:
};

#endif /* APP_DEVICES_DRIVER_STMTIMERPWM_HPP_ */