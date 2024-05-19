#include <Arduino.h>
#include "Arm.h"

Arm::Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2, uint8_t stepPin1R, uint8_t dirPin1R, uint8_t stepPin2R, uint8_t dirPin2R, uint8_t bawah, uint8_t atas, uint8_t bawahR, uint8_t atasR)
    : stepper(AccelStepper::DRIVER, stepPin1, dirPin1),
      stepper2(AccelStepper::DRIVER, stepPin2, dirPin2),
      stepperR(AccelStepper::DRIVER, stepPin1R, dirPin1R),
      stepper2R(AccelStepper::DRIVER, stepPin2R, dirPin2R) {
    
    pinMode(cek_logam, INPUT);
    pinMode(relayPinR, OUTPUT);
    pinMode(relayPinL, OUTPUT);    
    // servoL.attach(5); servoR.attach(4);
    
    _stepPin1 = stepPin1;
    _dirPin1 = dirPin1;
    _stepPin2 = stepPin2;
    _dirPin2 = dirPin2;

    stepper.setMaxSpeed(3000);
    stepper.setAcceleration(3000);
    stepper.moveTo(0);
    stepper2.setMaxSpeed(300);
    stepper2.setAcceleration(300);
    stepper2.moveTo(0);

    _stepPin1R = stepPin1R;
    _dirPin1R = dirPin1R;
    _stepPin2R = stepPin2R;
    _dirPin2R = dirPin2R;

    stepperR.setMaxSpeed(3000);
    stepperR.setAcceleration(3000);
    stepperR.moveTo(0);
    stepper2R.setMaxSpeed(400);
    stepper2R.setAcceleration(400);
    stepper2R.moveTo(0);

    _bawah = bawah;
    _atas = atas;
    pinMode(_bawah, INPUT); pinMode(_atas, INPUT);
    _bawahR = bawahR;
    _atasR = atasR;
    pinMode(_bawahR, INPUT); pinMode(_atasR, INPUT);    

    
    // servo_stop(servoL);
    // servo_stop(servoR);
}


void Arm::move(int signal, int signal2) {
  
  if (Rstate == 2 && Lstate == 2){
    stepper.moveTo(wait);  //if (stepper.currentPosition()>(-8400)){
    stepper2.moveTo(wait2);
    stepperR.moveTo(waitR); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(wait2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    if ((signal != 0) && (signal2 == 0 )){
      newangle1 = angle1;
      newangle2 = angle2;
      sig1 = signal;
      Lstate = 1; Rstate = 2;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      Lstate = 2; Rstate = 2;
      }
    else if ((signal2 != 0)){
      newangle1R = angle1R;
      newangle2R = angle2R;
      sig2 = signal2;
      Lstate = 2; Rstate = 1;
      }
    }
  }

  else if (Rstate == 2 && Lstate == 1){
    stepper.moveTo(newangle1);  //if (stepper.currentPosition()>(-8400)){
    stepper2.moveTo(newangle2);
    stepperR.moveTo(waitR); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(wait2R); 
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_take(servoL, relayPinL);
    if ((signal2 == 0 )){
      Lstate = 3; Rstate = 2;
      }
    else if(signal2 != 0){
      newangle1R = angle1R;
      newangle2R = angle2R;
      sig2 = signal2;
      Lstate = 3; Rstate = 1;
      }
    }
  }

  else if (Rstate == 2 && Lstate == 3){
    if (sig1 == 1){
      // stepper.moveTo(-14086);
      // stepper2.moveTo(-673);
      stepper.moveTo(-11000);
      stepper2.moveTo(-73);
    }
    else if (sig1 == 2){
      // stepper.moveTo(-16300);
      // stepper2.moveTo(-633);
      stepper.moveTo(-13667);
      stepper2.moveTo(-171);
    }
    else if (sig1 == 3){
      // stepper.moveTo(-11000);
      // //       stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      // stepper2.moveTo(-73);
      stepperR.moveTo(-9333.3);
      stepper2R.moveTo(-118.75);
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0){
        if (digitalRead(cek_logam) == LOW) {
          stepper.moveTo(-16300);
          stepper2.moveTo(-633);
        }
        else {
          stepper.moveTo(-11667);
          stepper2.moveTo(-166);
        }
      }
    }
    else if (sig1 == 4){
      stepper.moveTo(-11667);
      // stepper2.moveTo(-75);
      // if (stepper.currentPosition() < -10500){
      stepper2.moveTo(-166);
    }
    else if (sig1 == 5){
      // stepper.moveTo(-13667);
      // // stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      //   stepper2.moveTo(-171);
      stepperR.moveTo(-9333.3);
      stepper2R.moveTo(-118.75);
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW) {
          stepper.moveTo(-14086);
          stepper2.moveTo(-673);
        }
        else {
          stepper.moveTo(-11000);
          stepper2.moveTo(-73);
        }
      }
    }
    stepperR.moveTo(waitR); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(wait2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_put(servoL, relayPinL);
    if (signal != 0 && signal2 == 0 ){
      newangle1 = angle1;
      newangle2 = angle2;
      stepper2.moveTo(-75);
      sig1 = signal;
      Lstate = 1; Rstate = 2;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      Lstate = 2; Rstate = 2;
      }
    else if (signal2 != 0){
      newangle1R = angle1R;
      newangle2R = angle2R;
      sig2 = signal2;
      stepper2.moveTo(-75);
      Lstate = 2; Rstate = 1;
      }
    }
  }

  else if (Rstate == 1 && Lstate == 2){
    stepper.moveTo(wait); //if (stepper.currentPosition()>(-8400)){
    stepper2.moveTo(wait2);
    stepperR.moveTo(newangle1R); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(newangle2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_take(servoR, relayPinR);
    if ((signal != 0)){
      newangle1 = angle1;
      newangle2 = angle2;
      sig1 = signal;
      Lstate = 1; Rstate = 3;
      }
    else if(signal == 0){
      Lstate = 2; Rstate = 3;
      }
    } 
  }

  else if (Rstate == 1 && Lstate == 3){
    if (sig1 == 1){
      // stepper.moveTo(-14086);
      // stepper2.moveTo(-673);
      stepper.moveTo(-11000);
      stepper2.moveTo(-73);
    }
    else if (sig1 == 2){
      // stepper.moveTo(-16300);
      // stepper2.moveTo(-633);
      stepper.moveTo(-13667);
      stepper2.moveTo(-171);
    }
    else if (sig1 == 3){
      // stepper.moveTo(-11000);
      // //       stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      // stepper2.moveTo(-73);
      stepperR.moveTo(-9333.3);
      stepper2R.moveTo(-118.75);
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0){
        if (digitalRead(cek_logam) == LOW) {
          stepper.moveTo(-16300);
          stepper2.moveTo(-633);
        }
        else {
          stepper.moveTo(-11667);
          stepper2.moveTo(-166);
        }
      }
    }
    else if (sig1 == 4){
      stepper.moveTo(-11667);
      // stepper2.moveTo(-75);
      // if (stepper.currentPosition() < -10500){
      stepper2.moveTo(-166);
    }
    else if (sig1 == 5){
      // stepper.moveTo(-13667);
      // // stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      //   stepper2.moveTo(-171);
      stepperR.moveTo(-9333.3);
      stepper2R.moveTo(-118.75);
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW) {
          stepper.moveTo(-14086);
          stepper2.moveTo(-673);
        }
        else {
          stepper.moveTo(-11000);
          stepper2.moveTo(-73);
        }
      }
    }
    stepperR.moveTo(newangle1R); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(newangle2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_take(servoR, relayPinR);
    sucker_put(servoL, relayPinL);
    if ((signal != 0)){
      newangle1 = angle1;
      newangle2 = angle2;
      stepper2.moveTo(-75);
      sig1 = signal;
      Lstate = 1; Rstate = 3;
      }
    else if(signal == 0){
      stepper2.moveTo(-75);
      Lstate = 2; Rstate = 3;
      }
    }    
  }

  else if (Rstate == 3 && Lstate == 2){
    stepper.moveTo(wait); //if (stepper.currentPosition()>(-8400)){
    stepper2.moveTo(wait2);
    if (sig2 == 1){
      stepperR.moveTo(10000);
      stepper2R.moveTo(77);
      // stepperR.moveTo(13732);
      // stepper2R.moveTo(668);
      // stepperR.moveTo(13667);
      // // stepper2R.moveTo(75);
      // // if (stepperR.currentPosition() > 9500){
      // stepper2R.moveTo(198);
    }
    else if (sig2 == 2){

      // stepperR.moveTo(18533);
      // //       stepper2R.moveTo(75);
      // // if (stepperR.currentPosition() > 9500){
      // stepper2R.moveTo(165);
      stepperR.moveTo(13732);
      stepper2R.moveTo(668);
    }
    else if (sig2 == 3){
      stepperR.moveTo(9333);
      stepper2R.moveTo(118.75);
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW){
          // stepperR.moveTo(13667);
          // stepper2R.moveTo(198);
          stepperR.moveTo(18533);
          stepper2R.moveTo(165);
        }
        else {
        // stepperR.moveTo(13732);
        // stepper2R.moveTo(668);
          stepperR.moveTo(16308);
          stepper2R.moveTo(633);
        }
      }
    }
    else if (sig2 == 4){
      stepperR.moveTo(16308);
      stepper2R.moveTo(633);
    }
    else if (sig2 == 5){ // Sampah Plastik
      stepperR.moveTo(9333);
      stepper2R.moveTo(118.75);
      // start_mill = millis();
      // if (done_mill 
      // if
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW){
          stepperR.moveTo(13667);
          stepper2R.moveTo(198);
        }
        else {
          stepperR.moveTo(10000);
        //       stepper2R.moveTo(75);
        // if (stepperR.currentPosition() > 9500){
          stepper2R.moveTo(77);
        }
      }
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_put(servoR, relayPinR);   
    if ((signal == 0) && (signal2 != 0)){
      newangle1R = angle1R;
      newangle2R = angle2R;
      stepper2R.moveTo(75);
      sig2 = signal2;
      Lstate = 2; Rstate = 1;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      stepper2R.moveTo(75);
      Lstate = 2; Rstate = 2;
      }
    else if(signal != 0){
      newangle1 = angle1;
      newangle2 = angle2;
      sig1 = signal;
      stepper2R.moveTo(75);
      Lstate = 1; Rstate = 2;
      }
    }  
  }

  else if (Rstate == 3 && Lstate == 1){
    stepper.moveTo(newangle1); //if (stepper.currentPosition()>(-8400)){
    stepper2.moveTo(newangle2);
    if (sig2 == 1){
      stepperR.moveTo(10000);
      stepper2R.moveTo(77);
      // stepperR.moveTo(13732);
      // stepper2R.moveTo(668);
      // stepperR.moveTo(13667);
      // // stepper2R.moveTo(75);
      // // if (stepperR.currentPosition() > 9500){
      // stepper2R.moveTo(198);
    }
    else if (sig2 == 2){

      // stepperR.moveTo(18533);
      // //       stepper2R.moveTo(75);
      // // if (stepperR.currentPosition() > 9500){
      // stepper2R.moveTo(165);
      stepperR.moveTo(13732);
      stepper2R.moveTo(668);
    }
    else if (sig2 == 3){
      stepperR.moveTo(9333);
      stepper2R.moveTo(118.75);
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW){
          // stepperR.moveTo(13667);
          // stepper2R.moveTo(198);
          stepperR.moveTo(18533);
          stepper2R.moveTo(165);
        }
        else {
        // stepperR.moveTo(13732);
        // stepper2R.moveTo(668);
          stepperR.moveTo(16308);
          stepper2R.moveTo(633);
        }
      }
    }
    else if (sig2 == 4){
      stepperR.moveTo(16308);
      stepper2R.moveTo(633);
    }
    else if (sig2 == 5){ // Sampah Plastik
      stepperR.moveTo(9333);
      stepper2R.moveTo(118.75);
      // start_mill = millis();
      // if (done_mill 
      // if
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0) {
        if (digitalRead(cek_logam) == LOW){
          stepperR.moveTo(13667);
          stepper2R.moveTo(198);
        }
        else {
          stepperR.moveTo(10000);
        //       stepper2R.moveTo(75);
        // if (stepperR.currentPosition() > 9500){
          stepper2R.moveTo(77);
        }
      }
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_take(servoR, relayPinR);
    sucker_put(servoL, relayPinL);    
    if ((signal2 != 0)) {
      newangle1R = angle1R;
      newangle2R = angle2R;
      stepper2R.moveTo(75);
      sig2 = signal2;
      Lstate = 3; Rstate = 1;
      }
    else if(signal2 == 0 ){
      stepper2R.moveTo(75);
      Lstate = 3; Rstate = 2;
      }
    }    
  }
  stepperR.run();
  stepper2R.run();
  stepper.run();
  stepper2.run();
  if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
  selesai = true;
  }// Serial.println(String(Rstate) + " " + String(Lstate) + " " + String(angle1) + " " + String(angle2) + " " + String(angle1R) + " " + String(angle2R));}
  else{selesai = false;}
  // if (selesai){
  //   Serial.print(stepper.currentPosition()); Serial.print(stepper2.currentPosition()); Serial.print(stepperR.currentPosition()); Serial.println(stepper2R.currentPosition());
  // }
}

void Arm::kalibrasi(){
  
  int terkalibrasiL = 0, kalibrasibwhL = 0, kalibrasiatasL = 0;
  stepper.moveTo(-32 * 3.75 * 50);
  stepper2.moveTo(3.75 * 200);
  while(terkalibrasiL == 0){
    if (kalibrasibwhL == 0){
      if (stepper.distanceToGo() == 0){
        stepper.moveTo(32 * 3.75 *100);
      }
      stepper.run();
    }
    if (digitalRead(_bawah) == HIGH){
      kalibrasibwhL = 1;
      stepper.stop();
      stepper.setCurrentPosition(0);
      stepper.moveTo(stepper.currentPosition());
      stepper.setMaxSpeed(5000);
      stepper.setAcceleration(5000);
    }
    if (kalibrasiatasL == 0){
      // if (stepper.distanceToGo() == 0){
      stepper2.run();}    //}
    if (digitalRead(_atas) == HIGH){
      kalibrasiatasL = 1;
      stepper2.stop();
      stepper2.setCurrentPosition(0);
      stepper2.moveTo(stepper2.currentPosition());
      stepper2.setMaxSpeed(300);
      stepper2.setAcceleration(300);
    }   
    if (kalibrasiatasL == 1 && kalibrasibwhL == 1){
      terkalibrasiL = 1;
    }
  }
    while (stepper.distanceToGo() != 0 && stepper2.distanceToGo() != 0){
    stepper.run();
    stepper2.run();
    }
}

void Arm::kalibrasiR(){
  
  int terkalibrasiR = 0, kalibrasibwhR = 0, kalibrasiatasR = 0;
  stepperR.moveTo(32 * 3.75 * 50);
  stepper2R.moveTo(-3.75 * 200);
  while(terkalibrasiR == 0){
    if (kalibrasibwhR == 0){
      if (stepperR.distanceToGo() == 0){
        stepperR.moveTo(-32 * 3.75 *100);
      }
      stepperR.run();
    }
    if (digitalRead(_bawahR) == HIGH){
      kalibrasibwhR = 1;
      stepperR.stop();
      stepperR.setCurrentPosition(0);
      stepperR.moveTo(stepperR.currentPosition());
      stepperR.setMaxSpeed(4000);
      stepperR.setAcceleration(4000);
    }
    if (kalibrasiatasR == 0){
      // if (stepperR.distanceToGo() == 0){
      stepper2R.run();}
    // }
    if (digitalRead(_atasR) == HIGH){
      kalibrasiatasR = 1;
      // stepper2R.stop();
      stepper2R.setCurrentPosition(0);
      stepper2R.moveTo(stepper2R.currentPosition());
      stepper2R.setMaxSpeed(300);
      stepper2R.setAcceleration(300);
    }   
    if (kalibrasiatasR == 1 && kalibrasibwhR == 1){
      terkalibrasiR = 1;
    }
  }
  while (stepperR.distanceToGo() != 0 && stepper2R.distanceToGo() != 0){
    stepperR.run();
    stepper2R.run();
  }
  
}

void Arm::calculate(float dx, float dy) {
    const float L_atas = 35, L_bwh = 35;
    angle1 = -(32 * 3.75 * 57.29578 * (-atan(dx / dy) + acos((dx * dx + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(dx * dx + dy * dy) * L_bwh)))) / 1.8;
    angle2 = -3.75 * 200 + 3.75 * 3 + 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
}
void Arm::calculateR(float dx, float dy) {
    const float L_atasR = 35, L_bwhR = 35;
    angle1R = (-32 * 3.75  * 2 + 32 * 3.75 * 57.29578 * (atan(dx / dy) + acos((dx * dx + dy * dy + L_bwhR * L_bwhR - L_atasR * L_atasR) / (2 * sqrt(dx * dx + dy * dy) * L_bwhR)))) / 1.8;
    angle2R = 3.75 * 200 + 3.75 * 5 - 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwhR * L_bwhR + L_atasR * L_atasR) / (2 * L_atasR * L_bwhR))) / 1.8;
}

bool Arm::done() {
    selesai = (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0);
    return(selesai);
}

void Arm::sucker_take(Servo servo, int relayPin) {
  digitalWrite(relayPin, LOW); // Turn On Valve
  servo.write(0); delay (1000);
  servo.write(90);
  delay(100);
  servo.write(180); delay(1000);
  servo.write(90);
}

void Arm::sucker_put(Servo servo, int relayPin) {
  servo.write(0); delay (50);
  servo.write(90);
  delay(50);
  digitalWrite(relayPin, HIGH); // Turn On Valve
  delay(50);
  servo.write(180); delay(50);
  servo.write(90);
}

void Arm:: sucker_check_metal(Servo servo) {
  servo.write(0); delay (200);
  servo.write(90);
  delay(100);
  servo.write(180); delay(200);
  servo.write(90);
}

void Arm::servo_stop(Servo servo) {
  servo.write(0);
  delay(100000);
}
