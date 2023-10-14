#include "PID.h"


PID :: PID(double p , double i , double d , int Ts)
{ 
    _Ts = Ts;
    
    setTunings(p, i, d);
}

double PID::createpwm( float setpoint, float feedback)
{
    if (setpoint == 0){
        u = 0;
        curr_e = 0;
        int_e = 0;
        dif_e = 0;
        return u;
    } 

    curr_e = setpoint - feedback;
    int_e = int_e + curr_e;
    dif_e = curr_e - prev_e;
    
    u = kp*curr_e+ki*int_e+kd*dif_e;
    prev_e = curr_e;
    
    if (u >= 1){
        u = 1;
        }
    else if (u <= -1){
        u = -1;
        }      
    return u;   
}

void PID::setTunings(double p, double i, double d){
//reference: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-tuning-changes/
      if (p<0 || i<0 || d <0) return;
      
      sampleTimeSec = _Ts/1000;
      
      kp = p;
      ki = i*sampleTimeSec;
      kd = d/sampleTimeSec;
      
}
