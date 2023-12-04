/*
 * mcuAbstractionLayer.hpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_
#define APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_

#include <stdint.h>

class baseMcuAbstractionLayer {
   public:
    enum Peripheral_ADC {
        FL_Current,
        FR_Current,
        ST_Current,
        RL_Current,
        RR_Current,
        Batt_Voltage,
        ST_Volume,
    };

    enum Peripheral_PWM {
        FL_PWM,
        FR_PWM,
        ST_PWM,
        RL_PWM,
        RR_PWM,
    };

    enum Peripheral_Encoder {
        FL_Encoder,
        FR_Encoder,
        RL_Encoder,
        RR_Encoder,
    };

    enum Peripheral_GPIO {
        FL_PHASE,
        FL_SR,
        FR_PHASE,
        FR_SR,
        ST_PHASE,
        ST_SR,
        RL_PHASE,
        RL_SR,
        RR_PHASE,
        RR_SR,
    };

    virtual void init(void) = 0;

    virtual uint16_t getAdcValue(Peripheral_ADC p) = 0;

    virtual void setPwmValue(Peripheral_PWM p, float duty) = 0;

    virtual void setEncoderCntValue(Peripheral_Encoder p, uint32_t cnt) = 0;
    virtual uint32_t getEncoderCntValue(Peripheral_Encoder p) = 0;

    virtual void setGpioValue(Peripheral_GPIO p, bool value) = 0;
    virtual bool getGpioValue(Peripheral_GPIO p) = 0;
};

#endif /* APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_ */