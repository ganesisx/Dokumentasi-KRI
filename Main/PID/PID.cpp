#include "PID.h"


PID :: PID(float p , float i , float d , float Ts)
{ 
    _Ts = Ts;
    
    setTunings(p, i, d);
}

float PID::createpwm( float setpoint, float feedback)
{
    curr_e = setpoint - feedback;
    int_e = int_e + curr_e; // Bagian ini masih salah, integral itu selalu akumulatif bukan cuma dari satu siklus doang
                            // Harusnya int_e += curr_e;
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

void PID::setTunings(float p, float i, float d){
//reference: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-tuning-changes/
      if (p<0 || i<0 || d <0) return;
      
      sampleTimeSec = Ts/1000;
      
      kp = p;
      ki = i*sampleTimeSec;
      kd = d/sampleTimeSec;
      
}
