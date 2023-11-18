/*
 * A3921.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_A3921_HPP_
#define APP_DEVICES_DRIVER_A3921_HPP_

#include <Devices/Driver/stmTimerPwm.hpp>

class A3921 {
   public:
    A3921(stmTimerPwm* pwm, Peripheral p, GPIO_TypeDef* GPIO_PHASE_Port, uint16_t GPIO_PHASE_Pin, GPIO_TypeDef* GPIO_SR_Port, uint16_t GPIO_SR_Pin);
    void init();
    void setPower(float power);

   private:
    stmTimerPwm* _pwm;
    float _previousPower;

    Peripheral _p;
    GPIO_TypeDef* _GPIO_PHASE_Port;
    uint16_t _GPIO_PHASE_Pin;
    GPIO_TypeDef* _GPIO_SR_Port;
    uint16_t _GPIO_SR_Pin;
};

#endif /* APP_DEVICES_DRIVER_A3921_HPP_ */
