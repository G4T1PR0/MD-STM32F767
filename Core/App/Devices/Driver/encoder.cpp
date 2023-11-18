/*
 * encoder.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/encoder.hpp>

Encoder::Encoder(Peripheral p) {
    _p = p;
}

void Encoder::init() {
}

uint32_t Encoder::getCount() {
    return _encoder->getCount(_p);
}
