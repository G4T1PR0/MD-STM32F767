/*
 * encoder.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#include <Devices/Driver/encoder.hpp>

Encoder::Encoder(MAL* mcu, MAL::Peripheral_Encoder p) {
    _mcu = mcu;
    _p = p;
}

void Encoder::init() {
    _cnt = 0;
    _total_cnt = 0;
    _mcu->encoderSetCnt(_p, _offset);
}

void Encoder::update() {
    _prev_cnt = _cnt;
    _cnt = _mcu->encoderGetCnt(_p) - _offset;
    _acceleration = _cnt - _prev_cnt;
    _total_cnt += _cnt;
    _mcu->encoderSetCnt(_p, _offset);
}

int32_t Encoder::getCount() {
    return _cnt;
}

int32_t Encoder::getTotalCnt() {
    return _total_cnt;
}

int32_t Encoder::getAcceleration() {
    return _acceleration;
}
