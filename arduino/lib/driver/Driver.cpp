#include <Arduino.h>
#include "Driver.h"

Driver::Driver(uint8_t outCW, uint8_t outCCW) {
  // For motor driver with 2 pin
  _outCW = outCW;
  _outCCW = outCCW;

  pinMode(_outCW, OUTPUT);  pinMode(_outCCW, OUTPUT);
}

Driver::Driver(uint8_t outCW, uint8_t outCCW, uint8_t en) {
  // For motor driver with 3 pin
  _outCW = outCW;
  _outCCW = outCCW;
  _pwm_pin = en;

  pinMode(_outCW, OUTPUT);  pinMode(_outCCW, OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
}

void Driver::set_motor_speed(int pwm) {
  if (pwm < 0) {
    digitalWrite(_outCW, LOW);

    #ifdef DRIVER_3_PIN
      digitalWrite(_outCCW, HIGH);
      analogWrite(_pwm_pin, -pwm);
    #else
      analogWrite(_outCCW, -pwm);
    #endif
      
  } else if (pwm > 0) {
    digitalWrite(_outCCW, LOW);

    #ifdef DRIVER_3_PIN
      digitalWrite(_outCW, HIGH);
      analogWrite(_pwm_pin, pwm);
    #else
      analogWrite(_outCW, pwm);
    #endif

  } else {
      stop(break_state);
  }
}

void Driver::stop(int state) {
    if (state == 0) {
        digitalWrite(_outCW, LOW);
        digitalWrite(_outCCW, LOW);
    } else {
        digitalWrite(_outCW, HIGH);
        digitalWrite(_outCCW, HIGH);
    }
}