/*
 * steerAngleSensor.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_
#define APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_

#include <Devices/Driver/Interface/baseSteerAngleSensor.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class steerAngleSensor : public baseSteerAngleSensor {
   public:
    steerAngleSensor(MAL* mcu, MAL::Peripheral_ADC p);

    virtual void init();
    virtual void update();
    virtual float getAngle();

   private:
    MAL* _mcu;
    MAL::Peripheral_ADC _p;
    const float _raw2angle = 0.1;
};

#endif /* APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_ */
