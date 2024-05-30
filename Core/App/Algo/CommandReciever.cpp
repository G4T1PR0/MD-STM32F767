/*
 * CommandReciever.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: G4T1PR0
 */

#include <Algo/CommandReciever.hpp>
#include <cstring>
#include "stdio.h"

uint8_t CommandReciever::_tx_buffer[CMD_BUFFER_SIZE] = {0};

CommandReciever::CommandReciever(baseMcuAbstractionLayer* mcu, std::vector<MotorController*> mcs) {
    _mcu = mcu;
    _mcs = mcs;
}

void CommandReciever::init() {
    for (int i = 0; i < _mcs.size(); i++) {
        _feedback_data.push_back(feedback_data_t{0});
    }
}

void CommandReciever::update() {
    // printf("rx size: %d\n", _mcu->uartGetRxDataSize(MAL::P_UART::Controller));
    for (int i = 0; i < CMD_BUFFER_SIZE; i++) {
        _rx_buffer[i] = 0;
    }
    unsigned int data_size = _mcu->uartGetRxDataSize(MAL::P_UART::Controller);
    _mcu->uartReadViaBuffer(MAL::P_UART::Controller, _rx_buffer, data_size);

    bool is_rx = false;

    for (int i = 0; i < data_size; i++) {
        is_rx = true;
        // printf("\nrx: %d \n", _rx_buffer[i]);
        switch (_rx_mode) {
            case 0:
                if (_rx_buffer[i] == 0xFF) {
                    _rx_mode = 1;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 1:
                if (_rx_buffer[i] == 0xFF) {
                    _rx_mode = 2;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 2:
                if (_rx_buffer[i] == 0xFD) {
                    _rx_mode = 3;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 3:
                if (_rx_buffer[i] == 0x00) {
                    _rx_mode = 4;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 4:
                // printf("\nCMD RX: %d\n", _rx_buffer[i]);
                switch (_rx_buffer[i]) {
                    case 0:
                        _rx_mode = 0;
                        break;

                    case 1:
                        _rx_mode = 10;  // set motor mode
                        _rx_set_mode_temp = 0;
                        break;

                    case 2:
                        _rx_mode = 20;  // set motor target
                        _rx_set_target_temp = 0;
                        break;

                    case 0xFE:
                        _rx_mode = 0;
                        break;

                    default:
                        _rx_mode = 0;
                        break;
                }
                break;

            case 10:  // set mode
                // printf("motor_id: %d set mode: %d\n", _rx_set_mode_temp, _rx_buffer[i]);
                _mcs[_rx_set_mode_temp++]->setMode(_rx_buffer[i]);
                if (_rx_buffer[i] > 4) {
                    // printf("ERROR: set mode: %d\n", _rx_buffer[i]);
                }
                if (_rx_set_mode_temp >= _mcs.size()) {
                    _rx_mode = 4;
                }
                break;

            case 20:  // set target
                switch (_mcs[_rx_set_target_temp]->getMode()) {
                    case 0:
                        _rx_cnt++;
                        if (_rx_cnt == 2) {
                            _rx_cnt = 0;
                            _rx_set_target_temp++;
                        }
                        break;

                    case 1:
                        _int16_to_uint8.u8[_rx_cnt++] = _rx_buffer[i];
                        if (_rx_cnt == 2) {
                            // printf("\nmotor_id: %d set duty: %d %f\n", _rx_set_target_temp, _int16_to_uint8.i16, _int16_to_uint8.i16 / (float)32767);
                            //_mcs[_rx_set_target_temp]->setDuty(_int16_to_uint8.i16 / (float)32767);
                            _rx_cnt = 0;
                            _rx_set_target_temp++;
                        }
                        break;

                    case 2:
                        _int16_to_uint8.u8[_rx_cnt++] = _rx_buffer[i];
                        if (_rx_cnt == 2) {
                            // printf("\nmotor_id: %d set current: %d %f\n", _rx_set_target_temp, _int16_to_uint8.i16, _int16_to_uint8.i16 / (float)1638);
                            _mcs[_rx_set_target_temp]->setCurrent(_int16_to_uint8.i16 / (float)1638);
                            _rx_cnt = 0;
                            _rx_set_target_temp++;
                        }
                        break;

                    case 3:
                        _int16_to_uint8.u8[_rx_cnt++] = _rx_buffer[i];
                        if (_rx_cnt == 2) {
                            _mcs[_rx_set_target_temp]->setVelocity(_int16_to_uint8.i16);
                            _rx_cnt = 0;
                            _rx_set_target_temp++;
                        }
                        break;

                    case 4:
                        _int16_to_uint8.u8[_rx_cnt++] = _rx_buffer[i];
                        if (_rx_cnt == 2) {
                            // printf("\nmotor_id: %d set angle: %d %f\n", _rx_set_target_temp, _int16_to_uint8.i16, _int16_to_uint8.i16 / (float)655);
                            // _mcs[_rx_set_target_temp]->setAngle(_int16_to_uint8.i16 / (float)655);
                            _rx_cnt = 0;
                            _rx_set_target_temp++;
                        }
                        break;
                }
                if (_rx_set_target_temp >= _mcs.size()) {
                    _rx_mode = 4;
                    // printf("return mode: %d\n", _rx_mode);
                }
                break;

            default:
                _rx_mode = 0;
                break;
        }
    }
    if (is_rx) {
        is_rx = false;
        // printf("/////////////////////////////////////////////////\n");
    }
}

void CommandReciever::send() {
    unsigned int i = 0;

    _tx_buffer[0] = 0xFF;
    _tx_buffer[1] = 0xFF;
    _tx_buffer[2] = 0xFD;
    _tx_buffer[3] = 0x00;
    _tx_buffer_index = 4;

    for (MotorController* mc : _mcs) {
        switch (mc->getMode()) {
            case 0:
                _feedback_data[i].mode0_feedback_data.mode = mc->getMode();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode0_feedback_data, sizeof(mode0_feedback_data_t));
                _tx_buffer_index += sizeof(mode0_feedback_data_t);
                break;

            case 1:
                _feedback_data[i].mode1_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode1_feedback_data.duty = mc->getDuty() * 32767;
                _feedback_data[i].mode1_feedback_data.current = mc->getCurrent() * 1638;
                _feedback_data[i].mode1_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode1_feedback_data, sizeof(mode1_feedback_data_t));
                _tx_buffer_index += sizeof(mode1_feedback_data_t);
                break;

            case 2:
                _feedback_data[i].mode2_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode2_feedback_data.duty = mc->getDuty() * 32767;
                _feedback_data[i].mode2_feedback_data.target_current = mc->getTargetCurrent() * 1638;
                _feedback_data[i].mode2_feedback_data.current = mc->getCurrent() * 1638;
                _feedback_data[i].mode2_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode2_feedback_data, sizeof(mode2_feedback_data_t));
                _tx_buffer_index += sizeof(mode2_feedback_data_t);
                break;

            case 3:
                _feedback_data[i].mode3_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode3_feedback_data.duty = mc->getDuty() * 32767;
                _feedback_data[i].mode3_feedback_data.target_current = mc->getTargetCurrent() * 1638;
                _feedback_data[i].mode3_feedback_data.current = mc->getCurrent() * 1638;
                _feedback_data[i].mode3_feedback_data.target_velocity = mc->getTargetVelocity();
                _feedback_data[i].mode3_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode3_feedback_data, sizeof(mode3_feedback_data_t));
                _tx_buffer_index += sizeof(mode3_feedback_data_t);
                break;

            case 4:
                _feedback_data[i].mode4_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode4_feedback_data.duty = mc->getDuty() * 32767;
                _feedback_data[i].mode4_feedback_data.target_current = mc->getTargetCurrent() * 1638;
                _feedback_data[i].mode4_feedback_data.current = mc->getCurrent() * 1638;
                _feedback_data[i].mode4_feedback_data.target_angle = mc->getTargetAngle() * 655;
                _feedback_data[i].mode4_feedback_data.angle = mc->getAngle() * 655;
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode4_feedback_data, sizeof(mode4_feedback_data_t));
                _tx_buffer_index += sizeof(mode4_feedback_data_t);
                break;

            default:
                break;
        };
        i++;
    }

    //    printf("TX: ");
    //    for (int i = 0; i < _tx_buffer_index; i++) {
    //        printf("%02X ", _tx_buffer[i]);
    //    }
    //    printf("\r\n");

    // union aa {
    //     mode1_feedback_data_t data;
    //     uint8_t bytes[sizeof(mode2_feedback_data_t)];
    // };

    // aa lklk;

    // for (int i = 0; i < 5; i++) {
    //     printf("N %d: ", i);
    //     lklk.data = _feedback_data[i].mode1_feedback_data;

    //     for (int i = 0; i < sizeof(mode1_feedback_data_t); i++) {
    //         printf("%d ", lklk.bytes[i]);
    //     }
    //     printf("\r\n");
    // }

    // printf("DEC TX: ");
    // for (int i = 0; i < _tx_buffer_index; i++) {
    //     printf("%d ", _tx_buffer[i]);
    // }
    // printf("\r\n");

    _mcu->uartWriteViaBuffer(MAL::P_UART::Controller, _tx_buffer, _tx_buffer_index);
}
