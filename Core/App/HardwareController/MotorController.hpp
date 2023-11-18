/*
 * MotorController.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_

#include <Devices/Devices.hpp>

class MotorController {
   public:
    MotorController(A3921* A3921, currentSensor* currentSensor, Encoder* encoder);
    MotorController(A3921* A3921, currentSensor* currentSensor, steerAngleSensor* steerAngleSensor);
    void init(void);
    void update(void);

   private:
    A3921* _driver;
    currentSensor* _current;
    Encoder* _encoder;
    steerAngleSensor* _steerAngle;
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_ */
