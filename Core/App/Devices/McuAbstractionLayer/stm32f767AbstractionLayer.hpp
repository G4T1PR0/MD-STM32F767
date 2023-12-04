/*
 * stm32f767AbstractionLayer.hpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_
#define APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_

#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class stm32f767AbstractionLayer : public baseMcuAbstractionLayer {
   public:
    virtual void init(void);

    virtual uint16_t getAdcValue(Peripheral_ADC p);

    virtual void setPwmValue(Peripheral_PWM p, float duty);

    virtual void setEncoderCntValue(Peripheral_Encoder p, uint32_t cnt);
    virtual uint32_t getEncoderCntValue(Peripheral_Encoder p);

    virtual void setGpioValue(Peripheral_GPIO p, bool value);
    virtual bool getGpioValue(Peripheral_GPIO p);

   private:
    // ADC
    void _initADC();
    static uint16_t _data[16];

    // Timer Encoder
    void _initEncoder();

    // Timer PWM
    void _initPWM();
};

#endif /* APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_ */