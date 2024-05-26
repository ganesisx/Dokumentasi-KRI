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
    void calculateBuang(float dx, float dy, float dxR, float dyR);
    void buang();
    bool selesai;
    int Rstate, Lstate;
    void servo_stop(Servo servo);
    Servo servoL, servoR;
    int sum_plastik = 0, sum_kertas = 0, sum_nonferro = 0, sum_ferro = 0, sum_daun = 1;
    float angle1daun, angle2daun, angle1plastik, angle2plastik, angle1ferro, angle2ferro, angle1nonferro, angle2nonferro, angle1kertas, angle2kertas;

private:
    AccelStepper stepper;
    AccelStepper stepper2;
    AccelStepper stepperR;
    AccelStepper stepper2R;
    uint8_t _stepPin1, _dirPin1, _stepPin2, _dirPin2, _stepPin1R, _dirPin1R, _stepPin2R, _dirPin2R, _bawah, _atas, _bawahR, _atasR;
    float angle1, angle2, angle1R, angle2R, newangle1, newangle2, newangle1R, newangle2R;
    int wait = (-1500), wait2 = (-187.5), waitR = (1500), wait2R = (187.5); 
    int sig1, sig2;
    unsigned long start_mill;
    unsigned long done_mill;


    int relayPinL = A0; int relayPinR = A1; int cek_logam = A6;
    int checkL, checkR;
    void sucker_takeR(Servo servo, int relayPin);
    void sucker_takeL(Servo servo, int relayPin);
    void sucker_put(Servo servo, int relayPin);
    void sucker_check_metal(Servo servo);
};

#endif