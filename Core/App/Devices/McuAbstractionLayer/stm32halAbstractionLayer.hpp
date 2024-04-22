/*
 * stm32halAbstractionLayer.hpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_
#define APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_

#include <Devices/McuAbstractionLayer/RingBuffer.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define UART_BUFFER_SIZE 64
#define ADC_BUFFER_SIZE 50

class stm32halAbstractionLayer : public baseMcuAbstractionLayer {
   public:
    stm32halAbstractionLayer();
    virtual void init(void);

    // ADC
    virtual uint16_t adcGetValue(Peripheral_ADC p);
    virtual void adcGetBufferValue(Peripheral_ADC p, uint16_t* buffer, uint16_t size);

    // PWM
    virtual void pwmSetDuty(Peripheral_PWM p, float duty);
    virtual void pwmSetFrequency(Peripheral_PWM p, uint32_t frequency);

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

    // Interrupt
    virtual void interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void));
    static void (*_timerInterruptCallback[Peripheral_Interrupt::End_T])(void);

   private:
    // ADC
    void _initADC();

    static uint16_t _data[3][3 * ADC_BUFFER_SIZE];

    // Timer Encoder
    void _initEncoder();

    // Timer PWM
    void _initPWM();
    unsigned int _current_pwm_hz[Peripheral_PWM::End_P] = {0};

    // UART
    void _initUART();
    uint32_t _uartGetRxBufferDmaWriteAddress(Peripheral_UART p);
    static RingBuffer<uint8_t, UART_BUFFER_SIZE> _uartRxBuffer[Peripheral_UART::End_U];
    uint32_t _uartRxBufferReadAddress[Peripheral_UART::End_U] = {0};

    // Interrupt
    void _initTimerInterrupt();
};

#endif /* APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_ */