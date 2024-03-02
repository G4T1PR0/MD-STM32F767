/*
 * stm32halAbstractionLayer.cpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#include <Devices/McuAbstractionLayer/stm32halAbstractionLayer.hpp>
#include <cstring>
#include "adc.h"
#include "tim.h"
#include "usart.h"

struct PeripheralAllocation {
    enum STM_ADC {
        ADC_1,
        ADC_END
    };

    ADC_HandleTypeDef* ADC_Ins[ADC_END];
    STM_ADC ADC_Connected[MAL::Peripheral_ADC::End_A];
    uint8_t ADC_RANK[MAL::Peripheral_ADC::End_A];

    TIM_HandleTypeDef* PWM_TIM[MAL::Peripheral_PWM::End_P];
    uint32_t PWM_CH[MAL::Peripheral_PWM::End_P];

    TIM_HandleTypeDef* Encoder_TIM[MAL::Peripheral_Encoder::End_E];

    GPIO_TypeDef* GPIO_PORT[MAL::Peripheral_GPIO::End_G];
    uint16_t GPIO_PIN[MAL::Peripheral_GPIO::End_G];

    UART_HandleTypeDef* UART[MAL::Peripheral_UART::End_U];

    TIM_HandleTypeDef* TimerInterrupt_TIM[MAL::Peripheral_Interrupt::End_T];
};

static PeripheralAllocation PAL;

stm32halAbstractionLayer::stm32halAbstractionLayer() {
    // ADC
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1] = &hadc1;

    PAL.ADC_Connected[MAL::Peripheral_ADC::FL_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::FR_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::ST_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::RL_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::RR_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::Batt_Voltage] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::ST_Volume] = PeripheralAllocation::STM_ADC::ADC_1;

    PAL.ADC_RANK[MAL::Peripheral_ADC::FL_Current] = 0;
    PAL.ADC_RANK[MAL::Peripheral_ADC::FR_Current] = 1;
    PAL.ADC_RANK[MAL::Peripheral_ADC::ST_Current] = 2;
    PAL.ADC_RANK[MAL::Peripheral_ADC::RL_Current] = 3;
    PAL.ADC_RANK[MAL::Peripheral_ADC::RR_Current] = 4;
    PAL.ADC_RANK[MAL::Peripheral_ADC::Batt_Voltage] = 5;
    PAL.ADC_RANK[MAL::Peripheral_ADC::ST_Volume] = 6;

    // PWM
    PAL.PWM_TIM[MAL::Peripheral_PWM::FL_PWM] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::FL_PWM] = TIM_CHANNEL_1;

    PAL.PWM_TIM[MAL::Peripheral_PWM::FR_PWM] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::FR_PWM] = TIM_CHANNEL_2;

    PAL.PWM_TIM[MAL::Peripheral_PWM::ST_PWM] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::ST_PWM] = TIM_CHANNEL_3;

    PAL.PWM_TIM[MAL::Peripheral_PWM::RL_PWM] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::RL_PWM] = TIM_CHANNEL_4;

    PAL.PWM_TIM[MAL::Peripheral_PWM::RR_PWM] = &htim12;
    PAL.PWM_CH[MAL::Peripheral_PWM::RR_PWM] = TIM_CHANNEL_1;

    // Encoder
    PAL.Encoder_TIM[MAL::Peripheral_Encoder::FL_Encoder] = &htim3;
    PAL.Encoder_TIM[MAL::Peripheral_Encoder::FR_Encoder] = &htim2;
    PAL.Encoder_TIM[MAL::Peripheral_Encoder::RL_Encoder] = &htim8;
    PAL.Encoder_TIM[MAL::Peripheral_Encoder::RR_Encoder] = &htim4;

    // GPIO
    PAL.GPIO_PORT[MAL::Peripheral_GPIO::FL_PHASE] = Motor_FL_PHASE_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::FL_PHASE] = Motor_FL_PHASE_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::FL_SR] = Motor_FL_SR_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::FL_SR] = Motor_FL_SR_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::FR_PHASE] = Motor_FR_PHASE_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::FR_PHASE] = Motor_FR_PHASE_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::FR_SR] = Motor_FR_SR_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::FR_SR] = Motor_FR_SR_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::ST_PHASE] = Motor_ST_PHASE_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::ST_PHASE] = Motor_ST_PHASE_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::ST_SR] = Motor_ST_SR_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::ST_SR] = Motor_ST_SR_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::RL_PHASE] = Motor_RL_PHASE_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::RL_PHASE] = Motor_RL_PHASE_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::RL_SR] = Motor_RL_SR_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::RL_SR] = Motor_RL_SR_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::RR_PHASE] = Motor_RR_PHASE_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::RR_PHASE] = Motor_RR_PHASE_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::RR_SR] = Motor_RR_SR_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::RR_SR] = Motor_RR_SR_Pin;

    // UART
    PAL.UART[MAL::Peripheral_UART::Controller] = &huart5;

    PAL.UART[MAL::Peripheral_UART::Debug] = &huart3;

    // Timer Interrupt
    PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T100us] = &htim14;
}

void stm32halAbstractionLayer::init() {
    _initADC();
    _initEncoder();
    _initPWM();
    _initUART();
    _initTimerInterrupt();
}

// ADC
uint16_t stm32halAbstractionLayer::_data[MAL::Peripheral_ADC::End_A] = {0};

void stm32halAbstractionLayer::_initADC(void) {
    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1], (uint32_t*)this->_data, hadc1.Init.NbrOfConversion) !=
        HAL_OK) {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->DMA_Handle, DMA_IT_TC | DMA_IT_HT);
}

uint16_t stm32halAbstractionLayer::adcGetValue(Peripheral_ADC p) {
    if (p != Peripheral_ADC::End_A) {
        return this->_data[PAL.ADC_RANK[p]];
    }
    return 0;
}

// Encoder

void stm32halAbstractionLayer::_initEncoder() {
    HAL_TIM_Encoder_Start(PAL.Encoder_TIM[MAL::Peripheral_Encoder::FL_Encoder], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(PAL.Encoder_TIM[MAL::Peripheral_Encoder::FR_Encoder], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(PAL.Encoder_TIM[MAL::Peripheral_Encoder::RL_Encoder], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(PAL.Encoder_TIM[MAL::Peripheral_Encoder::RR_Encoder], TIM_CHANNEL_ALL);
}

void stm32halAbstractionLayer::encoderSetCnt(Peripheral_Encoder p, uint32_t cnt) {
    if (p != Peripheral_Encoder::End_E) {
        __HAL_TIM_SET_COUNTER(PAL.Encoder_TIM[p], cnt);
    }
}

uint32_t stm32halAbstractionLayer::encoderGetCnt(Peripheral_Encoder p) {
    if (p != Peripheral_Encoder::End_E) {
        return __HAL_TIM_GET_COUNTER(PAL.Encoder_TIM[p]);
    }
    return 0;
}

// PWM

void stm32halAbstractionLayer::_initPWM() {
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::FL_PWM], PAL.PWM_CH[MAL::Peripheral_PWM::FL_PWM]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::FR_PWM], PAL.PWM_CH[MAL::Peripheral_PWM::FR_PWM]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::ST_PWM], PAL.PWM_CH[MAL::Peripheral_PWM::ST_PWM]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::RL_PWM], PAL.PWM_CH[MAL::Peripheral_PWM::RL_PWM]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::RR_PWM], PAL.PWM_CH[MAL::Peripheral_PWM::RR_PWM]);
}

void stm32halAbstractionLayer::pwmSetDuty(Peripheral_PWM p, float duty) {
    if (p != Peripheral_PWM::End_P) {
        __HAL_TIM_SET_COMPARE(PAL.PWM_TIM[p], PAL.PWM_CH[p], duty * __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p]));
    }
}

// GPIO

void stm32halAbstractionLayer::gpioSetValue(Peripheral_GPIO p, bool value) {
    if (p != Peripheral_GPIO::End_G) {
        if (value) {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_RESET);
        }
    }
}

bool stm32halAbstractionLayer::gpioGetValue(Peripheral_GPIO p) {
    if (p != Peripheral_GPIO::End_G) {
        return HAL_GPIO_ReadPin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p]) == GPIO_PIN_SET;
    }
    return false;
}

// UART

RingBuffer<uint8_t, UART_BUFFER_SIZE> stm32halAbstractionLayer::_uartRxBuffer[Peripheral_UART::End_U];

void stm32halAbstractionLayer::_initUART() {
    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Controller], _uartRxBuffer[MAL::Peripheral_UART::Controller].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }

    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Debug], _uartRxBuffer[MAL::Peripheral_UART::Debug].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }
}

uint32_t stm32halAbstractionLayer::_uartGetRxBufferDmaWriteAddress(Peripheral_UART p) {
    if (p != Peripheral_UART::End_U) {
        return (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(PAL.UART[p]->hdmarx)) % UART_BUFFER_SIZE;
    }
    return 0;
}

void stm32halAbstractionLayer::uartPutChar(Peripheral_UART p, uint8_t data) {
    if (p != Peripheral_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], &data, 1) != HAL_OK) {
        }
    }
}

uint8_t stm32halAbstractionLayer::uartGetChar(Peripheral_UART p) {
    uint8_t data = 0;
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        data = _uartRxBuffer[p].pop();
    }
    return data;
}

void stm32halAbstractionLayer::uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    if (p != Peripheral_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], data, size) != HAL_OK) {
        }
    }
}

void stm32halAbstractionLayer::uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        _uartRxBuffer[p].pop(data, size);
    }
}

uint32_t stm32halAbstractionLayer::uartGetRxDataSize(Peripheral_UART p) {
    uint32_t size = 0;
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        size = _uartRxBuffer[p].size();
    }
    return size;
}

// Interrupt

void (*stm32halAbstractionLayer::_timerInterruptCallback[Peripheral_Interrupt::End_T])(void);

void stm32halAbstractionLayer::_initTimerInterrupt() {
    HAL_TIM_Base_Start(PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T100us]);
}

void stm32halAbstractionLayer::interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void)) {
    if (p != Peripheral_Interrupt::End_T) {
        _timerInterruptCallback[p] = callback;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T100us]) {
        if (stm32halAbstractionLayer::_timerInterruptCallback[MAL::Peripheral_Interrupt::T100us] != NULL) {
            stm32halAbstractionLayer::_timerInterruptCallback[MAL::Peripheral_Interrupt::T100us]();
        }
    }
}