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

    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(1000);
    stepper.moveTo(0);
    stepper2.setMaxSpeed(300);
    stepper2.setAcceleration(300);
    stepper2.moveTo(0);

    _stepPin1R = stepPin1R;
    _dirPin1R = dirPin1R;
    _stepPin2R = stepPin2R;
    _dirPin2R = dirPin2R;

    stepperR.setMaxSpeed(1000);
    stepperR.setAcceleration(1000);
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

    checkL = 0; checkR = 0;
    // servo_stop(servoL);
    // servo_stop(servoR);
}


void Arm::move(int signal, int signal2) {
if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    checkL = 0; checkR = 0;
    selesai = true;
  }// Serial.println(String(Rstate) + " " + String(Lstate) + " " + String(angle1) + " " + String(angle2) + " " + String(angle1R) + " " + String(angle2R));}
  // else if (newangle1 != angle1 || newangle2 != angle2 || newangle1R != angle1R || newangle2R != angle2R){selesai = false;}
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
      stepper.moveTo(-2750);
      stepper2.moveTo(-73);
    }
    else if (sig1 == 2){
      // stepper.moveTo(-16300);
      // stepper2.moveTo(-633);
      stepper.moveTo(-3417);
      stepper2.moveTo(-171);
    }
    else if (sig1 == 3){
      // stepper.moveTo(-11000);
      // //       stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      // stepper2.moveTo(-73);
      if (checkL == 0){
        stepper.moveTo(-2333.25);
        stepper2.moveTo(-118.75);
      }
      else if (checkL == 1){
        stepper.moveTo(-4075);
        stepper2.moveTo(-633);
      }
      else if (checkL == 2){
        stepper.moveTo(-2916.75);
        stepper2.moveTo(-166);
      }
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && checkL == 0){
        if (digitalRead(cek_logam) == LOW) {
          checkL = 1;
        }
        else {
          checkL = 2;
        }
      }
    }
    else if (sig1 == 4){
      stepper.moveTo(-2916.75);
      // stepper2.moveTo(-75);
      // if (stepper.currentPosition() < -10500){
      stepper2.moveTo(-166);
    }
    else if (sig1 == 5){
      // stepper.moveTo(-13667);
      // // stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      //   stepper2.moveTo(-171);
      if (checkL == 0){
        stepper.moveTo(-2333.25);
        stepper2.moveTo(-118.75);
      }
      else if (checkL == 1){
        stepper.moveTo(-3521.5);
        stepper2.moveTo(-673);
      }
      else if (checkL == 2){
        stepper.moveTo(-2750);
        stepper2.moveTo(-73);
      }
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && checkL == 0) {
        if (digitalRead(cek_logam) == LOW) {
          checkL = 1;
        }
        else {
          checkL = 2;
        }
      }
    }
    stepperR.moveTo(waitR); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(wait2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_put(servoL, relayPinL);
    if (sig1 == 1){
      sum_plastik += 1;
    }
    else if  (sig1 == 2){
      sum_daun += 1;
    }
    else if (sig1 == 3){
      if (checkL == 1){
        sum_nonferro += 1;
      }
      else if (checkL == 2){
        sum_kertas += 1;
      }
    }
    else if (sig1 == 4){
      sum_kertas += 1;
    }
    else if (sig1 == 5){
      if (checkL == 1){
        sum_ferro += 1;
      }
      else if (checkL == 2){
        sum_plastik += 1;
      }
    }
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
      stepper.moveTo(-2750);
      stepper2.moveTo(-73);
    }
    else if (sig1 == 2){
      // stepper.moveTo(-16300);
      // stepper2.moveTo(-633);
      stepper.moveTo(-3417);
      stepper2.moveTo(-171);
    }
    else if (sig1 == 3){
      // stepper.moveTo(-11000);
      // //       stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      // stepper2.moveTo(-73);
      if (checkL == 0){
        stepper.moveTo(-2333.25);
        stepper2.moveTo(-118.75);
      }
      else if (checkL == 1){
        stepper.moveTo(-4075);
        stepper2.moveTo(-633);
      }
      else if (checkL == 2){
        stepper.moveTo(-2916.75);
        stepper2.moveTo(-166);
      }
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && checkL == 0){
        if (digitalRead(cek_logam) == LOW) {
          checkL = 1;
        }
        else {
          checkL = 2;
        }
      }
    }
    else if (sig1 == 4){
      stepper.moveTo(-2916.75);
      // stepper2.moveTo(-75);
      // if (stepper.currentPosition() < -10500){
      stepper2.moveTo(-166);
    }
    else if (sig1 == 5){
      // stepper.moveTo(-13667);
      // // stepper2.moveTo(-75);
      // // if (stepper.currentPosition() < -10500){
      //   stepper2.moveTo(-171);
      if (checkL == 0){
        stepper.moveTo(-2333.25);
        stepper2.moveTo(-118.75);
      }
      else if (checkL == 1){
        stepper.moveTo(-3521.5);
        stepper2.moveTo(-673);
      }
      else if (checkL == 2){
        stepper.moveTo(-2750);
        stepper2.moveTo(-73);
      }
      if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && checkL ==0) {
        if (digitalRead(cek_logam) == LOW) {
          checkL = 1;
        }
        else {
          checkL = 2;
        }
      }
    }
    stepperR.moveTo(newangle1R); //if (stepperR.currentPosition()<8400){
    stepper2R.moveTo(newangle2R);
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_take(servoR, relayPinR);
    sucker_put(servoL, relayPinL);
        if (sig1 == 1){
      sum_plastik += 1;
    }
    else if  (sig1 == 2){
      sum_daun += 1;
    }
    else if (sig1 == 3){
      if (checkL == 1){
        sum_nonferro += 1;
      }
      else if (checkL == 2){
        sum_kertas += 1;
      }
    }
    else if (sig1 == 4){
      sum_kertas += 1;
    }
    else if (sig1 == 5){
      if (checkL == 1){
        sum_ferro += 1;
      }
      else if (checkL == 2){
        sum_plastik += 1;
      }
    }
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
      stepperR.moveTo(2500);
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
      stepperR.moveTo(3433);
      stepper2R.moveTo(668);
    }
    else if (sig2 == 3){
      if (checkR == 0){
        stepperR.moveTo(2333.25);
        stepper2R.moveTo(118.75);
      }
      else if (checkR == 1){
        stepperR.moveTo(4633.25);
        stepper2R.moveTo(165);
      }
      else if (checkR == 2){
        stepperR.moveTo(4077);
        stepper2R.moveTo(633);
      }
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0 && checkR == 0) {
        if (digitalRead(cek_logam) == LOW){
          checkR = 1;
          // stepperR.moveTo(13667);
          // stepper2R.moveTo(198);
        }
        else {
          checkR = 2;
        // stepperR.moveTo(13732);
        // stepper2R.moveTo(668);
        }
      }
    }
    else if (sig2 == 4){
      stepperR.moveTo(4077);
      stepper2R.moveTo(633);
    }
    else if (sig2 == 5){ // Sampah Plastik
      if (checkR == 0) {
        stepperR.moveTo(2333.25);
        stepper2R.moveTo(118.75);
      }
      else if (checkR == 1){
        stepperR.moveTo(3416.75);
        stepper2R.moveTo(198);
      }
      else if (checkR == 2){
          stepperR.moveTo(2500);
        //       stepper2R.moveTo(75);
        // if (stepperR.currentPosition() > 9500){
          stepper2R.moveTo(77);
      }
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0 && checkR == 0) {
        if (digitalRead(cek_logam) == LOW){
          checkR = 1;
        }
        else {
          checkR =2;
        }
      }
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
    sucker_put(servoR, relayPinR);  
    if (sig2 == 1){
      sum_plastik += 1;
    }
    else if  (sig2 == 2){
      sum_daun += 1;
    }
    else if (sig2 == 3){
      if (checkR == 1){
        sum_nonferro += 1;
      }
      else if (checkR == 2){
        sum_kertas += 1;
      }
    }
    else if (sig2 == 4){
      sum_kertas += 1;
    }
    else if (sig2 == 5){
      if (checkR == 1){
        sum_ferro += 1;
      }
      else if (checkR == 2){
        sum_plastik += 1;
      }
    } 
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
      stepperR.moveTo(2500);
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
      stepperR.moveTo(3433);
      stepper2R.moveTo(668);
    }
    else if (sig2 == 3){
      if (checkR == 0){
        stepperR.moveTo(2333.25);
        stepper2R.moveTo(118.75);
      }
      else if (checkR == 1){
        stepperR.moveTo(4633.25);
        stepper2R.moveTo(165);
      }
      else if (checkR == 2){
        stepperR.moveTo(4077);
        stepper2R.moveTo(633);
      }
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0 && checkR == 0) {
        if (digitalRead(cek_logam) == LOW){
          checkR = 1;
          // stepperR.moveTo(13667);
          // stepper2R.moveTo(198);
        }
        else {
          checkR = 2;
        // stepperR.moveTo(13732);
        // stepper2R.moveTo(668);
        }
      }
    }
    else if (sig2 == 4){
      stepperR.moveTo(4077);
      stepper2R.moveTo(633);
    }
    else if (sig2 == 5){ // Sampah Plastik
      if (checkR == 0) {
        stepperR.moveTo(2333.25);
        stepper2R.moveTo(118.75);
      }
      else if (checkR == 1){
        stepperR.moveTo(3416.75);
        stepper2R.moveTo(198);
      }
      else if (checkR == 2){
          stepperR.moveTo(2500);
        //       stepper2R.moveTo(75);
        // if (stepperR.currentPosition() > 9500){
          stepper2R.moveTo(77);
      }
      if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0 && checkR == 0) {
        if (digitalRead(cek_logam) == LOW){
          checkR = 1;
        }
        else {
          checkR =2;
        }
      }
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
      sucker_take(servoL, relayPinL);
      sucker_put(servoR, relayPinR); 
      if (sig2 == 1){
        sum_plastik += 1;
      }
      else if  (sig2 == 2){
        sum_daun += 1;
      }
      else if (sig2 == 3){
        if (checkR == 1){
          sum_nonferro += 1;
        }
        else if (checkR == 2){
          sum_kertas += 1;
        }
      }
      else if (sig2 == 4){
        sum_kertas += 1;
      }
      else if (sig2 == 5){
        if (checkR == 1){
          sum_ferro += 1;
        }
        else if (checkR == 2){
          sum_plastik += 1;
        }
      }    
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

}

void Arm::kalibrasi(){
  
  int terkalibrasiL = 0, kalibrasibwhL = 0, kalibrasiatasL = 0;
  stepper.moveTo(-8 * 3.75 * 50);
  stepper2.moveTo(3.75 * 200);
  int terkalibrasiR = 0, kalibrasibwhR = 0, kalibrasiatasR = 0;
  stepperR.moveTo(8 * 3.75 * 50);
  stepper2R.moveTo(-3.75 * 200);
  while(terkalibrasiL == 0 || terkalibrasiR == 0){
    if (kalibrasibwhL == 0){
      if (stepper.distanceToGo() == 0){
        stepper.moveTo(8 * 3.75 *100);
      }
      stepper.run();
    }
    if (digitalRead(_bawah) == HIGH){
      kalibrasibwhL = 1;
      stepper.stop();
      stepper.setCurrentPosition(0);
      stepper.moveTo(stepper.currentPosition());
      stepper.setMaxSpeed(3000);
      stepper.setAcceleration(3000);
    }
    if (kalibrasiatasL == 0){
      stepper2.run();}    //}
    if (digitalRead(_atas) == HIGH){
      kalibrasiatasL = 1;
      stepper2.setCurrentPosition(0);
      stepper2.moveTo(stepper2.currentPosition());
      stepper2.setMaxSpeed(300);
      stepper2.setAcceleration(300);
    }   
    if (kalibrasiatasL == 1 && kalibrasibwhL == 1){
      terkalibrasiL = 1;
    }
    if (kalibrasibwhR == 0){
      if (stepperR.distanceToGo() == 0){
        stepperR.moveTo(-8 * 3.75 *100);
      }
      stepperR.run();
    }
    if (digitalRead(_bawahR) == HIGH){
      kalibrasibwhR = 1;
      stepperR.setCurrentPosition(0);
      stepperR.moveTo(stepperR.currentPosition());
      stepperR.setMaxSpeed(3000);
      stepperR.setAcceleration(3000);
    }
    if (kalibrasiatasR == 0){
      stepper2R.run();}
    if (digitalRead(_atasR) == HIGH){
      kalibrasiatasR = 1;
      stepper2R.setCurrentPosition(0);
      stepper2R.moveTo(stepper2R.currentPosition());
      stepper2R.setMaxSpeed(300);
      stepper2R.setAcceleration(300);
    }   
    if (kalibrasiatasR == 1 && kalibrasibwhR == 1){
      terkalibrasiR = 1;
    }
  }
}

void Arm::buang(){
  if (Rstate == 3){
    if (sum_ferro != 0){
      stepperR.moveTo(angle1ferro); stepper2R.moveTo(angle2ferro);
    }
    else if (sum_nonferro != 0){
      stepperR.moveTo(angle1nonferro); stepper2R.moveTo(angle2nonferro);
    }
    if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
      sucker_put(servoR, relayPinR);
      if (sum_ferro != 0){
        sum_ferro -= 1;
      }
      else if (sum_nonferro != 0){
        sum_nonferro -= 1;
      }            
      Rstate = 4;
    }
  }
  else if (Rstate == 4){
    if (sum_ferro != 0){
      stepperR.moveTo(3416.75); stepper2R.moveTo(198);
    }
    else if (sum_nonferro != 0){
      stepperR.moveTo(4633.25); stepper2R.moveTo(165);
    }
    if (stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0){
      sucker_take(servoR, relayPinR);
      Rstate = 4;
    }
  }
  else if (Rstate == 5){
    stepperR.moveTo(0); stepper2R.moveTo(0);
  }

  if (Lstate == 3){  
    if (sum_daun != 0){
      stepper.moveTo(angle1daun); stepper2.moveTo(angle2daun);
    }
    else if (sum_kertas != 0){
      stepper.moveTo(angle1kertas); stepper2.moveTo(angle2kertas);
    }            
    else if (sum_plastik != 0){
      stepper.moveTo(angle1plastik); stepper2.moveTo(angle2plastik);
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0){
      sucker_put(servoL, relayPinL);
      if (sum_daun != 0){
        sum_daun -= 1;
      }
      else if (sum_kertas != 0){
        sum_kertas -= 1;
      }            
      else if (sum_plastik != 0){
        sum_plastik -= 1;
      }
      Lstate = 4;
    }
  }
  else if (Lstate == 4){  
    if (sum_daun != 0){
      stepper.moveTo(-3417); stepper2.moveTo(-171);
    }
    else if (sum_kertas != 0){
      stepper.moveTo(-2916.75); stepper2.moveTo(-166);
    }            
    else if (sum_plastik != 0){
      stepper.moveTo(-2750); stepper2.moveTo(-73);
    }
    if (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0){
      sucker_take(servoL, relayPinL);
      Lstate = 4;
    }
  }
  else if (Lstate == 5){
    stepper.moveTo(0); stepper2.moveTo(0);
  }

  if (sum_plastik == 0 && sum_kertas == 0 && sum_daun == 0){
    Lstate = 5;
  }
  if (sum_nonferro == 0 && sum_ferro == 0){
    Rstate = 5;
  }
  stepper.run(); stepper2.run(); stepperR.run(); stepper2.run();
}

void Arm::calculate(float dx, float dy) {
    const float L_atas = 35, L_bwh = 35;
    angle1 = -(8 * 3.75 * 57.29578 * (-atan(dx / dy) + acos((dx * dx + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(dx * dx + dy * dy) * L_bwh)))) / 1.8;
    angle2 = -3.75 * 200 + 3.75 * 3 + 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
}
void Arm::calculateR(float dx, float dy) {
    const float L_atasR = 35, L_bwhR = 35;
    angle1R = (-8 * 3.75  * 2 + 8 * 3.75 * 57.29578 * (atan(dx / dy) + acos((dx * dx + dy * dy + L_bwhR * L_bwhR - L_atasR * L_atasR) / (2 * sqrt(dx * dx + dy * dy) * L_bwhR)))) / 1.8;
    angle2R = 3.75 * 200 + 3.75 * 5 - 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwhR * L_bwhR + L_atasR * L_atasR) / (2 * L_atasR * L_bwhR))) / 1.8;
}
void Arm::calculateBuang(float dx, float dy, float dxR, float dyR) {
    const float L_atasR = 35, L_bwhR = 35;
    const float L_atas = 35, L_bwh = 35;
    float xdaun = dx - 26; float xkertas = dx - 13; float xplastik = dx;
    float xferro = dxR + 26; float xnonferro = dxR + 13;
    angle1daun = -(8 * 3.75 * 57.29578 * (-atan(xdaun / dy) + acos((xdaun * xdaun + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(xdaun * xdaun + dy * dy) * L_bwh)))) / 1.8;
    angle2daun = -3.75 * 200 + 3.75 * 3 + 3.75 * 57.29578 * (acos((-xdaun * xdaun - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
    angle1kertas = -(8 * 3.75 * 57.29578 * (-atan(xkertas / dy) + acos((xkertas * xkertas + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(xkertas * xkertas + dy * dy) * L_bwh)))) / 1.8;
    angle2kertas = -3.75 * 200 + 3.75 * 3 + 3.75 * 57.29578 * (acos((-xkertas * xkertas - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
    angle1plastik = -(8 * 3.75 * 57.29578 * (-atan(xplastik / dy) + acos((xplastik * xplastik + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(xplastik * xplastik + dy * dy) * L_bwh)))) / 1.8;
    angle2plastik = -3.75 * 200 + 3.75 * 3 + 3.75 * 57.29578 * (acos((-xplastik * xplastik - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
    angle1ferro = (-8 * 3.75  * 2 + 8 * 3.75 * 57.29578 * (atan(xferro / dyR) + acos((xferro * xferro + dyR * dyR + L_bwhR * L_bwhR - L_atasR * L_atasR) / (2 * sqrt(xferro * xferro + dyR * dyR) * L_bwhR)))) / 1.8;
    angle2ferro = 3.75 * 200 + 3.75 * 5 - 3.75 * 57.29578 * (acos((-xferro * xferro - dyR * dyR + L_bwhR * L_bwhR + L_atasR * L_atasR) / (2 * L_atasR * L_bwhR))) / 1.8;
    angle1nonferro = (-8 * 3.75  * 2 + 8 * 3.75 * 57.29578 * (atan(xnonferro / dyR) + acos((xnonferro * xnonferro + dyR * dyR + L_bwhR * L_bwhR - L_atasR * L_atasR) / (2 * sqrt(xnonferro * xnonferro + dyR * dyR) * L_bwhR)))) / 1.8;
    angle2nonferro = 3.75 * 200 + 3.75 * 5 - 3.75 * 57.29578 * (acos((-xnonferro * xnonferro - dyR * dyR + L_bwhR * L_bwhR + L_atasR * L_atasR) / (2 * L_atasR * L_bwhR))) / 1.8;
}

bool Arm::done() {
    selesai = (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0);
    return(selesai);
}

void Arm::sucker_take(Servo servo, int relayPin) {
  digitalWrite(relayPin, LOW); // Turn On Valve
  servo.write(0); delay (2000);
  servo.write(90);
  delay(100);
  servo.write(180); delay(2000);
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
