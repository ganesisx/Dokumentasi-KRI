// #include <Arduino.h>
// #include "Arm.h"

// Arm::Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2) {
//     _stepPin1 = stepPin1;
//     _dirPin1 = dirPin1;
//     _stepPin2 = stepPin2;
//     _dirPin2 = dirPin2;    
    
//     stepper.setMaxSpeed(3000);
//     stepper.setAcceleration(3000);
//     stepper.moveTo(0);
//     stepper2.setMaxSpeed(400);
//     stepper2.setAcceleration(400);
//     stepper2.moveTo(0); 
// //   pinMode(_outCW, OUTPUT);  pinMode(_outCCW, OUTPUT);
// }

// void Arm::move(float angle1, float angle2){
//     stepper.moveTo(angle1);
//     stepper2.moveTo(angle2);
//     stepper.run();
//     stepper2.run();
// }

// void Arm::calculate(float dx, float dy){
//     const int L = 35;
//     angle1 = (32 * 3.75 * 57.29578 * (-atan(dx/dy) + acos((dx*dx + dy*dy)/(2*sqrt(dx*dx + dy*dy)*L)))) / 1.8;
//     angle2 = -7.5 * 100 - 7.5 * 57.29578 * (acos((dx*dx + dy*dy - 2*L*L)/(2*L*L))) / 1.8;;
// }

// void Arm::done(){
//     if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0){
//         selesai = true;
//     }
//     else{
//         selesai = false;
//     }
// }
#include <Arduino.h>
#include "Arm.h"

Arm::Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2)
    : stepper(AccelStepper::DRIVER, stepPin1, dirPin1),
      stepper2(AccelStepper::DRIVER, stepPin2, dirPin2) {
    _stepPin1 = stepPin1;
    _dirPin1 = dirPin1;
    _stepPin2 = stepPin2;
    _dirPin2 = dirPin2;

    stepper.setMaxSpeed(3000);
    stepper.setAcceleration(3000);
    stepper.moveTo(0);
    stepper2.setMaxSpeed(400);
    stepper2.setAcceleration(400);
    stepper2.moveTo(0);
}

void Arm::move() {
    stepper.moveTo(angle1);
    stepper2.moveTo(angle2);
    stepper.run();
    stepper2.run();
}

void Arm::calculate(float dx, float dy) {
    const int L = 35;
    angle1 = (32 * 3.75 * 57.29578 * (-atan(dx / dy) + acos((dx * dx + dy * dy) / (2 * sqrt(dx * dx + dy * dy) * L)))) / 1.8;
    angle2 = -3.75 * 100 + 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + 2 * L * L) / (2 * L * L))) / 1.8;;
}

void Arm::done() {
    selesai = (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0);
}
