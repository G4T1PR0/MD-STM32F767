/*
 * CommandReciever.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: G4T1PR0
 */

#include <Algo/CommandReciever.hpp>
#include <cstring>

CommandReciever::CommandReciever(baseMcuAbstractionLayer* mcu, std::vector<MotorController*> mcs) {
    _mcs = mcs;
}

void CommandReciever::init() {
    for (int i = 0; i < _mcs.size(); i++) {
        _feedback_data.push_back(feedback_data_t{0});
    }
}

void CommandReciever::update() {
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
                _feedback_data[i].mode1_feedback_data.duty = mc->getDuty() * 65535;
                _feedback_data[i].mode1_feedback_data.current = mc->getCurrent();
                _feedback_data[i].mode1_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode1_feedback_data, sizeof(mode1_feedback_data_t));
                _tx_buffer_index += sizeof(mode1_feedback_data_t);
                break;

            case 2:
                _feedback_data[i].mode2_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode2_feedback_data.duty = mc->getDuty() * 65535;
                _feedback_data[i].mode2_feedback_data.target_current = mc->getTargetCurrent() * 3276.75;
                _feedback_data[i].mode2_feedback_data.current = mc->getCurrent() * 3276.75;
                _feedback_data[i].mode2_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode2_feedback_data, sizeof(mode2_feedback_data_t));
                _tx_buffer_index += sizeof(mode2_feedback_data_t);
                break;

            case 3:
                _feedback_data[i].mode3_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode3_feedback_data.duty = mc->getDuty() * 65535;
                _feedback_data[i].mode3_feedback_data.target_current = mc->getTargetCurrent() * 3276.75;
                _feedback_data[i].mode3_feedback_data.current = mc->getCurrent() * 3276.75;
                _feedback_data[i].mode3_feedback_data.target_velocity = mc->getTargetVelocity();
                _feedback_data[i].mode3_feedback_data.velocity = mc->getVelocity();
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode3_feedback_data, sizeof(mode3_feedback_data_t));
                _tx_buffer_index += sizeof(mode3_feedback_data_t);
                break;

            case 4:
                _feedback_data[i].mode4_feedback_data.mode = mc->getMode();
                _feedback_data[i].mode4_feedback_data.duty = mc->getDuty() * 65535;
                _feedback_data[i].mode4_feedback_data.target_current = mc->getTargetCurrent() * 3276.75;
                _feedback_data[i].mode4_feedback_data.current = mc->getCurrent() * 3276.75;
                _feedback_data[i].mode4_feedback_data.target_angle = mc->getTargetAngle() * 655.35;
                _feedback_data[i].mode4_feedback_data.angle = mc->getAngle() * 655.35;
                memcpy(&_tx_buffer[_tx_buffer_index], &_feedback_data[i].mode4_feedback_data, sizeof(mode4_feedback_data_t));
                _tx_buffer_index += sizeof(mode4_feedback_data_t);
                break;

            default:
                break;
        };
        i++;
    }
}
