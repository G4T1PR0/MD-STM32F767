/*
 * A3921.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_A3921_HPP_
#define APP_DEVICES_DRIVER_A3921_HPP_

#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class A3921 {
   public:
    A3921(MAL* mcu, MAL::Peripheral_PWM pwm, MAL::Peripheral_GPIO phase, MAL::Peripheral_GPIO sr);
    void init();
    void setDuty(float duty);

   private:
    MAL* _mcu;
    float _previousDuty;

    MAL::Peripheral_PWM _pwm;
    MAL::Peripheral_GPIO _phase;
    MAL::Peripheral_GPIO _sr;
};

#endif /* APP_DEVICES_DRIVER_A3921_HPP_ */
