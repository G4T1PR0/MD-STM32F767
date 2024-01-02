/*
 * HardwareController.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_

#include <Devices/Devices.hpp>
#include <HardwareController/MotorController.hpp>

class HardwareController {
   public:
    HardwareController(Devices* devices);
    void init(void);
    void update(void);
    static void update100us(void);

    static MotorController* FL_Motor;
    static MotorController* FR_Motor;
    static MotorController* ST_Motor;
    static MotorController* RL_Motor;
    static MotorController* RR_Motor;

   private:
    Devices* _devices;
};

#endif /* APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_ */
