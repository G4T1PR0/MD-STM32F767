/*
 * encoder.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_DRIVER_ENCODER_H_
#define APP_DEVICES_DRIVER_ENCODER_H_

#include <Devices/Driver/Interface/baseEncoder.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class Encoder : public baseEncoder {
   public:
    Encoder(MAL* mcu, MAL::Peripheral_Encoder p);

    virtual void init();
    virtual void update();
    virtual int32_t getCount();
    virtual int32_t getTotalCnt();
    virtual int32_t getAcceleration();

   private:
    MAL* _mcu;
    MAL::Peripheral_Encoder _p;

    const uint32_t _offset = 32767;
    int32_t _cnt;
    int32_t _prev_cnt;
    int32_t _total_cnt;
    int32_t _acceleration;
};

#endif /* APP_DEVICES_DRIVER_ENCODER_H_ */
