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
#include <Lib/MovingAverageFilter.hpp>

class currentSensor : public baseCurrentSensor {
   public:
    currentSensor(MAL* mcu, MAL::P_ADC p);
    virtual void init();
    virtual void update();
    virtual float getCurrent();

   private:
    MAL* _mcu;
    MAL::P_ADC _p;
    MovingAverageFilter<uint16_t, 10> _filter;

    float _offset = 0;

    float _temp_filter_value = 0;

    const float _raw2voltage = 3.3f / (1 << 12);
    const float _voltage2current = 0.033f;
};

#endif /* APP_DEVICES_DRIVER_current_SENSOR_HPP_ */
