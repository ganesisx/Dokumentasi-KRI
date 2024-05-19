#ifndef ARM_H
#define ARM_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>

class Arm {
public:
    Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2, uint8_t stepPin1R, uint8_t dirPin1R, uint8_t stepPin2R, uint8_t dirPin2R, uint8_t bawah, uint8_t atas, uint8_t bawahR, uint8_t atasR);
    
    void move(int signal, int signal2);
    // void moveR();
    void kalibrasi();
    void kalibrasiR();
    bool done();
    void calculate(float dx, float dy);
    void calculateR(float dx, float dy);
    void buang();
    bool selesai;
    int Rstate, Lstate;
    void servo_stop(Servo servo);
    Servo servoL, servoR;

private:
    AccelStepper stepper;
    AccelStepper stepper2;
    AccelStepper stepperR;
    AccelStepper stepper2R;
    uint8_t _stepPin1, _dirPin1, _stepPin2, _dirPin2, _stepPin1R, _dirPin1R, _stepPin2R, _dirPin2R, _bawah, _atas, _bawahR, _atasR;
    float angle1, angle2, angle1R, angle2R, newangle1, newangle2, newangle1R, newangle2R;
    int wait = (-32 * 50 * 3.75), wait2 = (-3.75 * 50), waitR = (32 * 50 * 3.75), wait2R = (3.75 * 50); 
    int sig1, sig2;
    unsigned long start_mill;
    unsigned long done_mill;

    int relayPinL = A0; int relayPinR = A1; int cek_logam = A6;
    void sucker_take(Servo servo, int relayPin);
    void sucker_put(Servo servo, int relayPin);
    void sucker_check_metal(Servo servo);
    
};

#endif