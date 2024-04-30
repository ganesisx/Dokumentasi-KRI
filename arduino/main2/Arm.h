// #ifndef ARM_H
// #define ARM_H

// #include <AccelStepper.h>
// #include <Arduino.h>

// class Arm{
//   public:
//     Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2);
//     void move(float angle1, float angle2);
//     void done();
//     void calculate(float dx, float dy);
    

//   private:
//     AccelStepper stepper(1, _stepPin1, _dirPin1);
//     AccelStepper stepper2(1, _stepPin2, _dirPin2);
//     uint8_t _conf, _stepPin1, _dirPin1, _stepPin2, _dirPin2;
//     float angle1 = 0, angle2 = 0;
//     bool selesai = false;
// };
// #endif
#ifndef ARM_H
#define ARM_H

#include <AccelStepper.h>
#include <Arduino.h>

class Arm {
public:
    Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2, uint8_t stepPin1R, uint8_t dirPin1R, uint8_t stepPin2R, uint8_t dirPin2R);

    void move();
    void moveR();
    void done();
    void calculate(float dx, float dy);
    void calculateR(float dx, float dy);

private:
    AccelStepper stepper;
    AccelStepper stepper2;
    AccelStepper stepperR;
    AccelStepper stepper2R;
    uint8_t _stepPin1, _dirPin1, _stepPin2, _dirPin2, _stepPin1R, _dirPin1R, _stepPin2R, _dirPin2R;
    float angle1, angle2, angle1R, angle2R;
    bool selesai;
};

#endif
