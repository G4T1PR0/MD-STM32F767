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

#define CMD_BUFFER_SIZE 512

class CommandReciever {
   public:
    CommandReciever(baseMcuAbstractionLayer* mcu, std::vector<MotorController*> mcs);

    void init();
    void update();
    void send();
    void parsePwm();

    unsigned int cnt1ms = 0;

    bool isConnectionLost = false;

   private:
    baseMcuAbstractionLayer* _mcu;
    std::vector<MotorController*> _mcs;

    struct mode0_feedback_data_t {
        uint8_t mode;
    };

    struct mode1_feedback_data_t {
        uint8_t mode;
        int16_t duty;
        int16_t current;
        int16_t velocity;
    } __attribute__((packed));

    struct mode2_feedback_data_t {
        uint8_t mode;
        int16_t duty;
        int16_t target_current;
        int16_t current;
        int16_t velocity;
    } __attribute__((packed));

    struct mode3_feedback_data_t {
        uint8_t mode;
        int16_t duty;
        int16_t target_current;
        int16_t current;
        int16_t target_velocity;
        int16_t velocity;
    } __attribute__((packed));

    struct mode4_feedback_data_t {
        uint8_t mode;
        int16_t duty;
        int16_t target_current;
        int16_t current;
        int16_t target_angle;
        int16_t angle;
    } __attribute__((packed));

    struct mode5_feedback_data_t {
        uint8_t mode;
        int16_t duty;
        int16_t current;
        int16_t angle;
    } __attribute__((packed));

    struct feedback_data_t {
        mode0_feedback_data_t mode0_feedback_data;
        mode1_feedback_data_t mode1_feedback_data;
        mode2_feedback_data_t mode2_feedback_data;
        mode3_feedback_data_t mode3_feedback_data;
        mode4_feedback_data_t mode4_feedback_data;
        mode5_feedback_data_t mode5_feedback_data;
    };

    union uint16_to_uint8_t {
        uint16_t u16;
        uint8_t u8[2];
    };

    union int16_to_uint8_t {
        int16_t i16;
        uint8_t u8[2];
    };

    uint16_to_uint8_t _uint16_to_uint8;
    int16_to_uint8_t _int16_to_uint8;

    std::vector<feedback_data_t>
        _feedback_data;

    static uint8_t _tx_buffer[CMD_BUFFER_SIZE];
    unsigned int _tx_buffer_index = 0;

    uint8_t _rx_buffer[CMD_BUFFER_SIZE] = {0};

    unsigned int _rx_mode = 0;
    unsigned int _rx_cnt = 0;
    unsigned int _rx_set_mode_temp = 0;
    unsigned int _rx_set_target_temp = 0;
};

#endif /* APP_ALGO_COMMANDRECIEVER_HPP_ */