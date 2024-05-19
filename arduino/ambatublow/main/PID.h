#include "Arduino.h"
#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float p, float i, float d, float _Ts);

    void setTunings(float p, float i, float d);

    float createpwm(float setpoint, float feedback);

private:
    float kp;
    float ki;
    float kd;
    float curr_e;
    float int_e;
    float dif_e;
    float prev_e;
    float u;
    float sampleTimeSec;
    int _Ts;
};

#endif