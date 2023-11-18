/*
 * stmTimerEncoder.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/stmTimerEncoder.hpp>

stmTimerEncoder::stmTimerEncoder() {
}

void stmTimerEncoder::init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

void stmTimerEncoder::setCount(Peripheral p, uint32_t count) {  // check ioc file
    switch (p) {
        case Peripheral::FL_Encoder:
            __HAL_TIM_SET_COUNTER(&htim3, count);
            break;

        case Peripheral::FR_Encoder:
            __HAL_TIM_SET_COUNTER(&htim2, count);
            break;

        case Peripheral::RL_Encoder:
            __HAL_TIM_SET_COUNTER(&htim8, count);
            break;

        case Peripheral::RR_Encoder:
            __HAL_TIM_SET_COUNTER(&htim4, count);

            break;
        default:
            break;
    }
}
uint32_t stmTimerEncoder::getCount(Peripheral p) {
    switch (p) {
        case Peripheral::FL_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim3);
            break;

        case Peripheral::FR_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim2);
            break;

        case Peripheral::RL_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim8);
            break;

        case Peripheral::RR_Encoder:
            return __HAL_TIM_GET_COUNTER(&htim4);

            break;
        default:
            break;
    }
    return 0;
}
