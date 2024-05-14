#include "PID.h"

PID::PID(float p, float i, float d, float Ts)
{ 
    _Ts = Ts;
    setTunings(p, i, d);
}

float PID::createpwm(float setpoint, float feedback)
{
    curr_e = setpoint - feedback;
    int_e += curr_e; // Menggunakan operator kumulatif +=
    dif_e = curr_e - prev_e;
    
    u = kp * curr_e + ki * int_e + kd * dif_e;
    prev_e = curr_e;
    
    if (u >= 255) {
        u = 255;
    } else if (u <= -255) {
        u = -255;
    }      
    return u;   
}

void PID::setTunings(float p, float i, float d)
{
    // reference: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-tuning-changes/
    if (p < 0 || i < 0 || d < 0) return;
    
    sampleTimeSec = _Ts / 1000; // Menggunakan _Ts yang telah diset di konstruktor
    
    kp = p;
    ki = i * sampleTimeSec;
    kd = d / sampleTimeSec;   
}
