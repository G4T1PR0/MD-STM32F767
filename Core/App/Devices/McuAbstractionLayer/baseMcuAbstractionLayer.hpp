/*
 * baseMcuAbstractionLayer.hpp
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
        End_A,
    };

    enum Peripheral_PWM {
        FL_PWM,
        FR_PWM,
        ST_PWM,
        RL_PWM,
        RR_PWM,
        End_P,
    };

    enum Peripheral_Encoder {
        FL_Encoder,
        FR_Encoder,
        RL_Encoder,
        RR_Encoder,
        End_E,
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
        End_G,
    };

    enum Peripheral_UART {
        Controller,
        Debug,
        End_U,
    };

    virtual void init(void) = 0;

    virtual uint16_t adcGetValue(Peripheral_ADC p) = 0;

    virtual void pwmSetDuty(Peripheral_PWM p, float duty) = 0;

    virtual void encoderSetCnt(Peripheral_Encoder p, uint32_t cnt) = 0;
    virtual uint32_t encoderGetCnt(Peripheral_Encoder p) = 0;

    virtual void gpioSetValue(Peripheral_GPIO p, bool value) = 0;
    virtual bool gpioGetValue(Peripheral_GPIO p) = 0;

    virtual void uartPutChar(Peripheral_UART p, uint8_t data) = 0;
    virtual uint8_t uartGetChar(Peripheral_UART p) = 0;

    virtual void uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual void uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual uint32_t uartGetRxBufferSize(Peripheral_UART p) = 0;
};

typedef baseMcuAbstractionLayer MAL;

#endif /* APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_ */