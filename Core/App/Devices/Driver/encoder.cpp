/*
 * encoder.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/encoder.hpp>

Encoder::Encoder(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::Peripheral_Encoder p) {
    _mcu = mcu;
    _p = p;
}

uint32_t Encoder::getCount() {
    return _mcu->getEncoderCntValue(_p);
}
