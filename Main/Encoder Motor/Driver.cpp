#include "mbed.h"
#include "Driver.h"

Driver::Driver(PinName rev, PinName fwd, PinName pwm):
    _fwd(fwd), _rev(rev), _pwm(pwm){
        _pwm = 0;
        _fwd = 0;
        _rev = 0;
}

void Driver::Motion(float speed){
    _fwd = speed > (float)0.0;
    _rev = speed < (float)0.0;
    _pwm = fabs(speed);
}

void Driver::Stop(int highlow){
    if(highlow == 1){
        _pwm = 1;
        _fwd = 1;
        _rev = 1;
    }
    else if(highlow == 0){
        _fwd = 0;
        _rev = 0;
    }
}