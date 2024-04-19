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
        LED_1,
        LED_2,
        LED_3,
        LED_4,
        LED_5,
        LED_6,
        LED_7,
        LED_8,
        LED_9,
        LED_10,
        End_G,
    };

    enum Peripheral_UART {
        Controller,
        Debug,
        End_U,
    };

    enum Peripheral_Interrupt {
        T100us,
        End_T
    };

    virtual void init(void) = 0;

    // ADC
    virtual uint16_t adcGetValue(Peripheral_ADC p) = 0;

    // PWM
    virtual void pwmSetDuty(Peripheral_PWM p, float duty) = 0;
    virtual void pwmSetFrequency(Peripheral_PWM p, uint32_t frequency) = 0;

    // Encoder
    virtual void encoderSetCnt(Peripheral_Encoder p, uint32_t cnt) = 0;
    virtual uint32_t encoderGetCnt(Peripheral_Encoder p) = 0;

    // GPIO
    virtual void gpioSetValue(Peripheral_GPIO p, bool value) = 0;
    virtual bool gpioGetValue(Peripheral_GPIO p) = 0;

    // UART
    virtual void uartPutChar(Peripheral_UART p, uint8_t data) = 0;
    virtual uint8_t uartGetChar(Peripheral_UART p) = 0;
    virtual void uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual void uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual uint32_t uartGetRxDataSize(Peripheral_UART p) = 0;

    // Interrupt
    virtual void interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void)) = 0;
};

typedef baseMcuAbstractionLayer MAL;

#endif /* APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_ */