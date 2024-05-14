#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>

class Driver{
  public:
    Driver(uint8_t outCW, uint8_t outCCW);
    Driver(uint8_t outCW, uint8_t outCCW, uint8_t en);
    void set_motor_speed(int pwm);
    

  private:
    uint8_t _outCW, _outCCW, _pwm_pin;

    // break_state = 0 for no break
    // break_state = 1 for with break
    int break_state = 1;
    void stop(int state);
};
#endif