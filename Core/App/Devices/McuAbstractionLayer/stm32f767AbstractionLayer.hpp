/*
 * stm32f767AbstractionLayer.hpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_
#define APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_

#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define UART_BUFFER_SIZE 64

class stm32f767AbstractionLayer : public baseMcuAbstractionLayer {
   public:
    stm32f767AbstractionLayer();
    virtual void init(void);

    // ADC
    virtual uint16_t adcGetValue(Peripheral_ADC p);

    // PWM
    virtual void pwmSetDuty(Peripheral_PWM p, float duty);

    // Encoder
    virtual void encoderSetCnt(Peripheral_Encoder p, uint32_t cnt);
    virtual uint32_t encoderGetCnt(Peripheral_Encoder p);

    // GPIO
    virtual void gpioSetValue(Peripheral_GPIO p, bool value);
    virtual bool gpioGetValue(Peripheral_GPIO p);

    // UART
    virtual void uartPutChar(Peripheral_UART p, uint8_t data);
    virtual uint8_t uartGetChar(Peripheral_UART p);
    virtual void uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size);
    virtual void uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size);
    virtual uint32_t uartGetRxDataSize(Peripheral_UART p);

    // Timer Interrupt
    virtual void timerInterruptSetCallback(Peripheral_TimerInterrupt p, void (*callback)(void));
    static void (*_timerInterruptCallback[Peripheral_TimerInterrupt::End_T - 1])(void);

   private:
    // ADC
    void _initADC();

    static uint16_t _data[16];

    // Timer Encoder
    void _initEncoder();

    // Timer PWM
    void _initPWM();

    // UART
    void _initUART();
    uint32_t _uartCheckRxBufferDmaWriteAddress(Peripheral_UART p);
    static uint8_t _uartRxBuffer[Peripheral_UART::End_U - 1][UART_BUFFER_SIZE];
    uint32_t _uartRxBufferReadAddress[Peripheral_UART::End_U - 1] = {0};

    // Timer Interrupt
    void _initTimerInterrupt();
};

#endif /* APP_DEVICES_STM32F767ABSTRACTIONLAYER_STM32F767ABSTRACTIONLAYER_HPP_ */