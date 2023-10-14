#ifndef Driver_h
#define Driver_h

#include "mbed.h"
class Driver
{
  public:
    Driver(PinName rev, PinName fwd, PinName pwm);
    
    void Stop(int highlow);
    void Motion(float pwm);
    
  private:
    DigitalOut _fwd;
    DigitalOut _rev;
    PwmOut _pwm;
};
#endif