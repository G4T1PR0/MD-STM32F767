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
    A3921(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::Peripheral_PWM pwm, baseMcuAbstractionLayer::Peripheral_GPIO phase, baseMcuAbstractionLayer::Peripheral_GPIO sr);
    void init();
    void setPower(float power);

   private:
    baseMcuAbstractionLayer* _mcu;
    float _previousPower;

    baseMcuAbstractionLayer::Peripheral_PWM _pwm;
    baseMcuAbstractionLayer::Peripheral_GPIO _phase;
    baseMcuAbstractionLayer::Peripheral_GPIO _sr;
};

#endif /* APP_DEVICES_DRIVER_A3921_HPP_ */
