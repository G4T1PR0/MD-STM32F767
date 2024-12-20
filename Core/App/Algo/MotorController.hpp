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
#include <Lib/MovingAverageFilter.hpp>
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
    void setCurrentLimit(float limit);
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

    bool OC = false;
    bool CE = false;

    float D_dt = 0;
    uint32_t D_dt_avg = 0;

    float prev_D_dt = 0;

   private:
    baseMotorDriver* _driver;
    baseCurrentSensor* _current;
    baseEncoder* _encoder;
    baseSteerAngleSensor* _steerAngle;

    PID<float> _current_pid;
    PID<float> _velocity_pid;
    PID<float> _angle_pid;

    MovingAverageFilter<uint32_t, 8> _duty_dt_filter;

    void _update(void);

    bool _isSteer = 0;
    int _mode = 0;

    bool _isMotorShutDown = 0;

    float _motorInputDuty = 0;
    float _pidTargetCurrent = 0;

    float _observedCurrent = 0;
    float _observedVelocity = 0;
    float _observedAngle = 0;

    float _targetDuty = 0;
    float _targetCurrent = 0;
    float _targetVelocity = 0;
    float _targetAngle = 0;

    float _currentLimit = 13;
    float _maxiumCurrentLimit = 15;
    uint32_t _dutyDtffLimit = 2000;

    bool _isMotorConnectionReversed = 0;
    bool _isMotorDirectionReversed = 0;

    unsigned int _beep_time = 0;
    unsigned int _beep_freq = 0;

    unsigned int _beep_cnt = 0;
    bool _beep_flag = 0;

    // DEBUG

    float _d_prev_angle = 0;
};

#endif /* APP_ALGO_MOTORCONTROLLER_HPP_ */
