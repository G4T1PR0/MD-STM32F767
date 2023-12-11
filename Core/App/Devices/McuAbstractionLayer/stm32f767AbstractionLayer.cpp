/*
 * stm32f767AbstractionLayer.cpp
 *
 *  Created on: Dec 4, 2023
 *      Author: G4T1PR0
 */

#include <Devices/McuAbstractionLayer/stm32f767AbstractionLayer.hpp>
#include <cstring>
#include "adc.h"
#include "tim.h"
#include "usart.h"

void stm32f767AbstractionLayer::init() {
    _initADC();
    _initEncoder();
    _initPWM();
    _initUART();
}

// ADC
uint16_t stm32f767AbstractionLayer::_data[16] = {0};

void stm32f767AbstractionLayer::_initADC(void) {
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)this->_data, sizeof(uint16_t)) * hadc1.Init.NbrOfConversion !=
        HAL_OK) {
        Error_Handler();
    }
}

uint16_t stm32f767AbstractionLayer::adcGetValue(Peripheral_ADC p) {
    switch (p) {
        case Peripheral_ADC::FL_Current:
            return this->_data[0];
            break;

        case Peripheral_ADC::FR_Current:
            return this->_data[1];
            break;

        case Peripheral_ADC::ST_Current:
            return this->_data[2];
            break;

        case Peripheral_ADC::RL_Current:
            return this->_data[3];
            break;

        case Peripheral_ADC::RR_Current:
            return this->_data[4];
            break;

        case Peripheral_ADC::Batt_Voltage:
            return this->_data[5];
            break;

        case Peripheral_ADC::ST_Volume:
            return this->_data[6];
            break;

        default:
            return 0;
            break;
    }
    return 0;
}

// Encoder

void stm32f767AbstractionLayer::_initEncoder() {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

void stm32f767AbstractionLayer::encoderSetCnt(Peripheral_Encoder p, uint32_t cnt) {
    switch (p) {
        case Peripheral_Encoder::FL_Encoder:
            __HAL_TIM_SET_COUNTER(&htim3, cnt);
            break;

        case Peripheral_Encoder::FR_Encoder:
            __HAL_TIM_SET_COUNTER(&htim2, cnt);
            break;

        case Peripheral_Encoder::RL_Encoder:
            __HAL_TIM_SET_COUNTER(&htim8, cnt);
            break;

        case Peripheral_Encoder::RR_Encoder:
            __HAL_TIM_SET_COUNTER(&htim4, cnt);

            break;
        default:
            break;
    }
}

uint32_t stm32f767AbstractionLayer::encoderGetCnt(Peripheral_Encoder p) {
    switch (p) {
        case Peripheral_Encoder::FL_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim3);
            break;

        case Peripheral_Encoder::FR_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim2);
            break;

        case Peripheral_Encoder::RL_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim8);
            break;

        case Peripheral_Encoder::RR_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim4);
            break;

        default:
            return 0;
            break;
    }
    return 0;
}

// PWM

void stm32f767AbstractionLayer::_initPWM() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

void stm32f767AbstractionLayer::pwmSetDuty(Peripheral_PWM p, float duty) {
    switch (p) {
        case Peripheral_PWM::RL_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral_PWM::FR_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral_PWM::ST_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral_PWM::FL_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral_PWM::RR_PWM:
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        default:
            break;
    }
}

// GPIO

void stm32f767AbstractionLayer::gpioSetValue(Peripheral_GPIO p, bool value) {
    switch (p) {
        case Peripheral_GPIO::FL_PHASE:
            if (value)
                HAL_GPIO_WritePin(Motor_FL_PHASE_GPIO_Port, Motor_FL_PHASE_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_FL_PHASE_GPIO_Port, Motor_FL_PHASE_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::FL_SR:
            if (value)
                HAL_GPIO_WritePin(Motor_FL_SR_GPIO_Port, Motor_FL_SR_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_FL_SR_GPIO_Port, Motor_FL_SR_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::FR_PHASE:
            if (value)
                HAL_GPIO_WritePin(Motor_FR_PHASE_GPIO_Port, Motor_FR_PHASE_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_FR_PHASE_GPIO_Port, Motor_FR_PHASE_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::FR_SR:
            if (value)
                HAL_GPIO_WritePin(Motor_FR_SR_GPIO_Port, Motor_FR_SR_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_FR_SR_GPIO_Port, Motor_FR_SR_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::ST_PHASE:
            if (value)
                HAL_GPIO_WritePin(Motor_ST_PHASE_GPIO_Port, Motor_ST_PHASE_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_ST_PHASE_GPIO_Port, Motor_ST_PHASE_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::ST_SR:
            if (value)
                HAL_GPIO_WritePin(Motor_ST_SR_GPIO_Port, Motor_ST_SR_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_ST_SR_GPIO_Port, Motor_ST_SR_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::RL_PHASE:
            if (value)
                HAL_GPIO_WritePin(Motor_RL_PHASE_GPIO_Port, Motor_RL_PHASE_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_RL_PHASE_GPIO_Port, Motor_RL_PHASE_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::RL_SR:
            if (value)
                HAL_GPIO_WritePin(Motor_RL_SR_GPIO_Port, Motor_RL_SR_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_RL_SR_GPIO_Port, Motor_RL_SR_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::RR_PHASE:
            if (value)
                HAL_GPIO_WritePin(Motor_RR_PHASE_GPIO_Port, Motor_RR_PHASE_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_RR_PHASE_GPIO_Port, Motor_RR_PHASE_Pin, GPIO_PIN_RESET);
            break;

        case Peripheral_GPIO::RR_SR:
            if (value)
                HAL_GPIO_WritePin(Motor_RR_SR_GPIO_Port, Motor_RR_SR_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(Motor_RR_SR_GPIO_Port, Motor_RR_SR_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

bool stm32f767AbstractionLayer::gpioGetValue(Peripheral_GPIO p) {
    return false;
}

// UART

// uint8_t stm32f767AbstractionLayer::_uart5TxBuffer[UART_BUFFER_SIZE] = {0};
uint8_t stm32f767AbstractionLayer::_uart5RxBuffer[UART_BUFFER_SIZE] = {0};

// uint8_t stm32f767AbstractionLayer::_usart3TxBuffer[UART_BUFFER_SIZE] = {0};
uint8_t stm32f767AbstractionLayer::_usart3RxBuffer[UART_BUFFER_SIZE] = {0};

void stm32f767AbstractionLayer::_initUART() {
    while (HAL_UART_Receive_DMA(&huart3, _usart3RxBuffer, UART_BUFFER_SIZE) != HAL_OK) {
    }

    while (HAL_UART_Receive_DMA(&huart5, _uart5RxBuffer, UART_BUFFER_SIZE) != HAL_OK) {
    }
}

uint32_t stm32f767AbstractionLayer::_uartCheckRxBufferDmaWriteAddress(Peripheral_UART p) {
    uint32_t size = 0;
    switch (p) {
        case Peripheral_UART::Controller:
            size = (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx)) % UART_BUFFER_SIZE;

            break;

        case Peripheral_UART::Debug:
            size = (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx)) % UART_BUFFER_SIZE;
            break;

        default:
            break;
    }
    return size;
}

void stm32f767AbstractionLayer::uartPutChar(Peripheral_UART p, uint8_t data) {
    switch (p) {
        case Peripheral_UART::Controller:
            while (HAL_UART_Transmit_DMA(&huart5, &data, 1) != HAL_OK) {
            }

            break;

        case Peripheral_UART::Debug:
            while (HAL_UART_Transmit_DMA(&huart3, &data, 1) != HAL_OK) {
            }
            break;

        default:
            break;
    }
}

uint8_t stm32f767AbstractionLayer::uartGetChar(Peripheral_UART p) {
    uint8_t data = 0;
    switch (p) {
        case Peripheral_UART::Controller:
            if (_uartCheckRxBufferDmaWriteAddress(Peripheral_UART::Controller) != _uart5RxBufferReadAddress) {
                data = _uart5RxBuffer[_uart5RxBufferReadAddress++];
                _uart5RxBufferReadAddress %= UART_BUFFER_SIZE;
            }
            break;

        case Peripheral_UART::Debug:
            if (_uartCheckRxBufferDmaWriteAddress(Peripheral_UART::Debug) != _usart3RxBufferReadAddress) {
                data = _usart3RxBuffer[_usart3RxBufferReadAddress++];
                _usart3RxBufferReadAddress %= UART_BUFFER_SIZE;
            }
            break;

        default:
            break;
    }
    return data;
}

void stm32f767AbstractionLayer::uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    switch (p) {
        case Peripheral_UART::Controller:
            while (HAL_UART_Transmit_DMA(&huart5, data, size) != HAL_OK) {
            }
            break;

        case Peripheral_UART::Debug:
            while (HAL_UART_Transmit_DMA(&huart3, data, size) != HAL_OK) {
            }
            break;

        default:
            break;
    }
}

void stm32f767AbstractionLayer::uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    uint32_t dmaWriteAddress = _uartCheckRxBufferDmaWriteAddress(p);
    switch (p) {
        case Peripheral_UART::Controller:

            if (dmaWriteAddress > _uart5RxBufferReadAddress) {
                uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _uart5RxBufferReadAddress;

                if (diff_readAddress2dmaWriteAddress > size) {
                    memcpy(data, &_uart5RxBuffer[_uart5RxBufferReadAddress], size);
                    _uart5RxBufferReadAddress += size;
                } else {
                    memcpy(data, &_uart5RxBuffer[_uart5RxBufferReadAddress], diff_readAddress2dmaWriteAddress);
                    _uart5RxBufferReadAddress = dmaWriteAddress;
                }

            } else if (dmaWriteAddress < _uart5RxBufferReadAddress) {
                uint32_t diff_readAddress2BufferSize = UART_BUFFER_SIZE - _uart5RxBufferReadAddress;
                uint32_t buff_cp_cnt = 0;

                if (diff_readAddress2BufferSize > size) {
                    memcpy(data, &_uart5RxBuffer[_uart5RxBufferReadAddress], size);
                    _uart5RxBufferReadAddress += size;
                } else {
                    memcpy(data, &_uart5RxBuffer[_uart5RxBufferReadAddress], diff_readAddress2BufferSize);
                    _uart5RxBufferReadAddress = 0;
                    buff_cp_cnt = diff_readAddress2BufferSize;

                    uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _uart5RxBufferReadAddress;

                    if (diff_readAddress2dmaWriteAddress + buff_cp_cnt > size) {
                        memcpy(&data[diff_readAddress2dmaWriteAddress], &_uart5RxBuffer[_uart5RxBufferReadAddress], size - buff_cp_cnt);
                        _uart5RxBufferReadAddress += size - buff_cp_cnt;
                    } else {
                        memcpy(&data[diff_readAddress2BufferSize], &_uart5RxBuffer[_uart5RxBufferReadAddress], diff_readAddress2dmaWriteAddress);
                        _uart5RxBufferReadAddress = dmaWriteAddress;
                    }
                }
            }
            break;

        case Peripheral_UART::Debug:
            if (dmaWriteAddress > _usart3RxBufferReadAddress) {
                uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _usart3RxBufferReadAddress;

                if (diff_readAddress2dmaWriteAddress > size) {
                    memcpy(data, &_usart3RxBuffer[_usart3RxBufferReadAddress], size);
                    _usart3RxBufferReadAddress += size;
                } else {
                    memcpy(data, &_usart3RxBuffer[_usart3RxBufferReadAddress], diff_readAddress2dmaWriteAddress);
                    _usart3RxBufferReadAddress = dmaWriteAddress;
                }

            } else if (dmaWriteAddress < _usart3RxBufferReadAddress) {
                uint32_t diff_readAddress2BufferSize = UART_BUFFER_SIZE - _usart3RxBufferReadAddress;
                uint32_t buff_cp_cnt = 0;

                if (diff_readAddress2BufferSize > size) {
                    memcpy(data, &_usart3RxBuffer[_usart3RxBufferReadAddress], size);
                    _usart3RxBufferReadAddress += size;
                } else {
                    memcpy(data, &_usart3RxBuffer[_usart3RxBufferReadAddress], diff_readAddress2BufferSize);
                    _usart3RxBufferReadAddress = 0;
                    buff_cp_cnt = diff_readAddress2BufferSize;

                    uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _usart3RxBufferReadAddress;

                    if (diff_readAddress2dmaWriteAddress + buff_cp_cnt > size) {
                        memcpy(&data[diff_readAddress2dmaWriteAddress], &_usart3RxBuffer[_usart3RxBufferReadAddress], size - buff_cp_cnt);
                        _usart3RxBufferReadAddress += size - buff_cp_cnt;
                    } else {
                        memcpy(&data[diff_readAddress2BufferSize], &_usart3RxBuffer[_usart3RxBufferReadAddress], diff_readAddress2dmaWriteAddress);
                        _usart3RxBufferReadAddress = dmaWriteAddress;
                    }
                }
            }

            break;

        default:
            break;
    }
}

uint32_t stm32f767AbstractionLayer::uartGetRxBufferSize(Peripheral_UART p) {
    uint32_t dmaWriteAddress = _uartCheckRxBufferDmaWriteAddress(p);
    uint32_t size = 0;
    switch (p) {
        case Peripheral_UART::Controller:
            if (dmaWriteAddress > _uart5RxBufferReadAddress) {
                size = dmaWriteAddress - _uart5RxBufferReadAddress;
            } else if (dmaWriteAddress < _uart5RxBufferReadAddress) {
                size = UART_BUFFER_SIZE - _uart5RxBufferReadAddress + dmaWriteAddress;
            }
            break;

        case Peripheral_UART::Debug:
            if (dmaWriteAddress > _usart3RxBufferReadAddress) {
                size = dmaWriteAddress - _usart3RxBufferReadAddress;
            } else if (dmaWriteAddress < _usart3RxBufferReadAddress) {
                size = UART_BUFFER_SIZE - _usart3RxBufferReadAddress + dmaWriteAddress;
            }
            break;

        default:
            break;
    }
    return size;
}