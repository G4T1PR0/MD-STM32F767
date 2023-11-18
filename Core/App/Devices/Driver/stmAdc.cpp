/*
 * stmAdc.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/stmAdc.hpp>

uint16_t stmAdc::_data[16] = {0};

stmAdc::stmAdc() {
}

void stmAdc::init(void) {
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)this->_data, sizeof(uint16_t)) * hadc1.Init.NbrOfConversion !=
        HAL_OK) {
        Error_Handler();
    }
}

uint16_t stmAdc::get(Peripheral p) {  // ioc file
    switch (p) {
        case Peripheral::FL_Current:
            return this->_data[0];
            break;

        case Peripheral::FR_Current:
            return this->_data[1];
            break;

        case Peripheral::ST_Current:
            return this->_data[2];
            break;

        case Peripheral::RL_Current:
            return this->_data[3];
            break;

        case Peripheral::RR_Current:
            return this->_data[4];
            break;

        case Peripheral::Batt_Voltage:
            return this->_data[5];
            break;

        case Peripheral::ST_Volume:
            return this->_data[6];
            break;

        default:
            break;
    }
    return 0;
}
