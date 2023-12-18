/*
 * currentSensor.hpp
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_current_SENSOR_HPP_
#define APP_DEVICES_DRIVER_current_SENSOR_HPP_

#include <Devices/Driver/Interface/baseCurrentSensor.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class currentSensor : public baseCurrentSensor {
   public:
    currentSensor(MAL* mcu, MAL::Peripheral_ADC p);
    virtual void init();
    virtual void update();
    virtual float getCurrent();

   private:
    MAL* _mcu;
    MAL::Peripheral_ADC _p;
    const float _raw2voltage = 3.3f / (1 << 12);
    const float _voltage2current = 0.033f;
};

#endif /* APP_DEVICES_DRIVER_current_SENSOR_HPP_ */
