/*
 * MotorController.hpp
 *
 *  Created on: Nov 18, 2023
 *      Author: G4T1PR0
 */

#ifndef APP_ALGO_MOTORCONTROLLER_HPP_
#define APP_ALGO_MOTORCONTROLLER_HPP_

#include <Devices/Driver/Interface/baseCurrentSensor.hpp>
#include <Devices/Driver/Interface/baseEncoder.hpp>
#include <Devices/Driver/Interface/baseMotorDriver.hpp>
#include <Devices/Driver/Interface/baseSteerAngleSensor.hpp>
#include <Lib/pid.hpp>

class MotorController {
   public:
    MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseEncoder* encoder);
    MotorController(baseMotorDriver* driver, baseCurrentSensor* currentSensor, baseSteerAngleSensor* steerAngleSensor);
    void init(void);

    static void update(MotorController* instance);

    void setMode(int mode);
    int getMode();

    void setDuty(float duty);
    float getDuty();

    void setCurrentPID(float p, float i, float d);
    void setCurrent(float current);
    float getCurrent();
    float getTargetCurrent();

    void setVelocity(float velocity);
    float getVelocity();
    float getTargetVelocity();

    void setAnglePID(float p, float i, float d);
    void setAngle(float angle);
    float getAngle();
    float getTargetAngle();

    void setMotorConnectionReversed(bool isReversed);
    void setMotorDirection(bool d);

    void setBeepTime(int time);
    void setBeepFreqKhz(int freq);

   private:
    baseMotorDriver* _driver;
    baseCurrentSensor* _current;
    baseEncoder* _encoder;
    baseSteerAngleSensor* _steerAngle;

    PID<float> _current_pid;
    PID<float> _velocity_pid;
    PID<float> _angle_pid;

    void _update(void);

    bool _isSteer;
    int _mode;

    float _motorInputDuty;
    float _pidTargetCurrent;

    float _observedCurrent = 0;
    float _observedVelocity = 0;
    float _observedAngle = 0;

    float _targetDuty;
    float _targetCurrent;
    float _targetVelocity;
    float _targetAngle;

    bool _isMotorConnectionReversed = 0;
    bool _isMotorDirectionReversed = 0;

    unsigned int _beep_time = 0;
    unsigned int _beep_freq = 0;

    unsigned int _beep_cnt = 0;
    bool _beep_flag = 0;
};

#endif /* APP_ALGO_MOTORCONTROLLER_HPP_ */
