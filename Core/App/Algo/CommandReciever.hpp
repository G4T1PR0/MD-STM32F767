/*
 * CommandReciever.hpp
 *
 *  Created on: Apr 25, 2024
 *      Author: G4T1PR0
 */

#ifndef APP_ALGO_COMMANDRECIEVER_HPP_
#define APP_ALGO_COMMANDRECIEVER_HPP_

#include <Algo/MotorController.hpp>
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>
#include <vector>

class CommandReciever {
   public:
    CommandReciever(baseMcuAbstractionLayer* mcu, std::vector<MotorController*> mcs);

    void init();
    void update();

   private:
    std::vector<MotorController*> _mcs;

    struct mode0_feedback_data_t {
        uint8_t mode;
    };

    struct mode1_feedback_data_t {
        uint8_t mode;
        uint16_t duty;
        uint16_t current;
        uint16_t velocity;
    };

    struct mode2_feedback_data_t {
        uint8_t mode;
        uint16_t duty;
        uint16_t target_current;
        uint16_t current;
        uint16_t velocity;
    };

    struct mode3_feedback_data_t {
        uint8_t mode;
        uint16_t duty;
        uint16_t target_current;
        uint16_t current;
        uint16_t target_velocity;
        uint16_t velocity;
    };

    struct mode4_feedback_data_t {
        uint8_t mode;
        uint16_t duty;
        uint16_t target_current;
        uint16_t current;
        uint16_t target_angle;
        uint16_t angle;
    };

    struct feedback_data_t {
        mode0_feedback_data_t mode0_feedback_data;
        mode1_feedback_data_t mode1_feedback_data;
        mode2_feedback_data_t mode2_feedback_data;
        mode3_feedback_data_t mode3_feedback_data;
        mode4_feedback_data_t mode4_feedback_data;
    };

    std::vector<feedback_data_t> _feedback_data;

    uint8_t _tx_buffer[256] = {0};
    unsigned int _tx_buffer_index = 0;
};

#endif /* APP_ALGO_COMMANDRECIEVER_HPP_ */