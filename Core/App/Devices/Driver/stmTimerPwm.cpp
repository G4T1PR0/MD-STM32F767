/*
 * stmTimerPwm.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/stmTimerPwm.hpp>

stmTimerPwm::stmTimerPwm() {
}

void stmTimerPwm::init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

void stmTimerPwm::setPWM(Peripheral p, float duty) {  // check ioc
    switch (p) {
        case Peripheral::RL_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral::FR_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral::ST_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral::FL_PWM:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        case Peripheral::RR_PWM:
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
            break;

        default:
            break;
    }
}