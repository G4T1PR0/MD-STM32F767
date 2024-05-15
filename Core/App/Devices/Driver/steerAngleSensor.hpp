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
    steerAngleSensor(MAL* mcu, MAL::P_ADC p);

    virtual void init();
    virtual void update();
    virtual float getAngle();
    virtual float getRawAngle();

   private:
    MAL* _mcu;
    MAL::P_ADC _p;
    const float _raw2angle = 90.0f / 2257.0f;

    float _temp_filter_value = 0;
};

#endif /* APP_DEVICES_DRIVER_STEERANGLESENSOR_HPP_ */
