/*
 * A3921.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_A3921_HPP_
#define APP_DEVICES_DRIVER_A3921_HPP_

#include <Devices/Driver/Interface/baseMotorDriver.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class A3921 : public baseMotorDriver {
   public:
    A3921(MAL* mcu, MAL::P_PWM pwm, MAL::P_GPIO phase, MAL::P_GPIO sr);
    virtual void init();
    virtual void setDuty(float duty);
    virtual void setBrakeEnabled(bool enabled);

   private:
    MAL* _mcu;
    float _previousDuty;
    bool _brakeEnabled = 0;

    MAL::P_PWM _pwm;
    MAL::P_GPIO _phase;
    MAL::P_GPIO _sr;
};

#endif /* APP_DEVICES_DRIVER_A3921_HPP_ */
