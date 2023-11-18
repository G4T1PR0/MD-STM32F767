/*
 * stmHalConfig.hpp
 *
 *  Created on: Nov 17, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_STMHALCONFIG_H_
#define APP_DEVICES_STMHALCONFIG_H_

enum Peripheral {
    FL_PWM,
    FR_PWM,
    ST_PWM,
    RL_PWM,
    RR_PWM,

    FL_Current,
    FR_Current,
    ST_Current,
    RL_Current,
    RR_Current,
    Batt_Voltage,
    ST_Volume,

    FL_Encoder,
    FR_Encoder,
    RL_Encoder,
    RR_Encoder
};

#endif /*APP_DEVICES_STMHALCONFIG_H_ */
