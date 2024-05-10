#include <Arduino.h>
#include "Arm.h"

Arm::Arm(uint8_t stepPin1, uint8_t dirPin1, uint8_t stepPin2, uint8_t dirPin2, uint8_t stepPin1R, uint8_t dirPin1R, uint8_t stepPin2R, uint8_t dirPin2R, uint8_t bawah, uint8_t atas, uint8_t bawahR, uint8_t atasR)
    : stepper(AccelStepper::DRIVER, stepPin1, dirPin1),
      stepper2(AccelStepper::DRIVER, stepPin2, dirPin2),
      stepperR(AccelStepper::DRIVER, stepPin1R, dirPin1R),
      stepper2R(AccelStepper::DRIVER, stepPin2R, dirPin2R) {
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

    _stepPin1R = stepPin1R;
    _dirPin1R = dirPin1R;
    _stepPin2R = stepPin2R;
    _dirPin2R = dirPin2R;

    stepperR.setMaxSpeed(3000);
    stepperR.setAcceleration(3000);
    stepperR.moveTo(0);
    stepper2R.setMaxSpeed(200);
    stepper2R.setAcceleration(200);
    stepper2R.moveTo(0);

    _bawah = bawah;
    _atas = atas;
    pinMode(_bawah, INPUT); pinMode(_atas, INPUT);
    _bawahR = bawahR;
    _atasR = atasR;
    pinMode(_bawahR, INPUT); pinMode(_atasR, INPUT);    
}


void Arm::move(int signal, int signal2) {
  
  if (Rstate == 2 && Lstate == 2){
    stepper.moveTo(-32 * 50 * 3.75);
    stepper2.moveTo(-3.75 * 50);
    stepperR.moveTo(32 * 50 * 3.75);
    stepper2R.moveTo(3.75 * 50);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    selesai = true;
    if ((signal != 0) && (signal2 == 0 )){
      newangle1 = angle1;
      newangle2 = angle2;
      selesai = false;
      Lstate = 1; Rstate = 2;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      selesai = false;
      Lstate = 2; Rstate = 2;
      }
    else if ((signal2 != 0)){
      newangle1R = angle1R;
      newangle2R = angle2R;
      selesai = false;
      Lstate = 2; Rstate = 1;
      }
    }
  }

  else if (Rstate == 2 && Lstate == 1){
    stepper.moveTo(newangle1);
    stepper2.moveTo(newangle2);
    stepperR.moveTo(32 * 50 * 3.75);
    stepper2R.moveTo(3.75 * 50);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if ((signal2 == 0 )){
      selesai = true;
      Lstate = "simpan"; Rstate = 2;
      }
    else if(signal2 != 0){
      newangle1R = angle1R;
      newangle2R = angle2R;
      selesai = true;
      Lstate = "simpan"; Rstate = 1;
      }
    }
  }

  else if (Rstate == 2 && Lstate == "simpan"){
    stepper.moveTo(32 * 50 * 3.75);
    stepper2.moveTo(-3.75 * 50);
    stepperR.moveTo(-32 * 50 * 3.75);
    stepper2R.moveTo(3.75 * 50);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if (signal != 0 && signal2 == 0 ){
      newangle1 = angle1;
      newangle2 = angle2;
      selesai = false;
      Lstate = 1; Rstate = 2;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      selesai = false;
      Lstate = 2; Rstate = 2;
      }
    else if (signal2 != 0){
      newangle1R = angle1R;
      newangle2R = angle2R;
      selesai = false;
      Lstate = 2; Rstate = 1;
      }
    }
  }

  else if (Rstate == 1 && Lstate == 2){
    stepper.moveTo(32 * 50 * 3.75);
    stepper2.moveTo(-3.75 * 50);
    stepperR.moveTo(newangle1R);
    stepper2R.moveTo(newangle2R);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if ((signal != 0)){
      newangle1 = angle1;
      newangle2 = angle2;
      selesai = true;
      Lstate = 1; Rstate = "simpan";
      }
    else if(signal == 0){
      selesai = true;
      Lstate = 2; Rstate = "simpan";
      }
    } 
  }

  else if (Rstate == 1 && Lstate == "simpan"){
    stepper.moveTo(32 * 50 * 3.75);
    stepper2.moveTo(-3.75 * 50);
    stepperR.moveTo(newangle1R);
    stepper2R.moveTo(newangle2R);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if ((signal != 0)){
      newangle1 = angle1;
      newangle2 = angle2;
      selesai = true;
      Lstate = 1; Rstate = "simpan";
      }
    else if(signal == 0){
      selesai = true;
      Lstate = 2; Rstate = "simpan";
      }
    }    
  }

  else if (Rstate == "simpan" && Lstate == 2){
    stepper.moveTo(32 * 50 * 3.75);
    stepper2.moveTo(-3.75 * 50);
    stepperR.moveTo(-32 * 50 * 3.75);
    stepper2R.moveTo(3.75 * 50);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if ((signal == 0) && (signal2 != 0)){
      newangle1R = angle1R;
      newangle2R = angle2R;
      selesai = false;
      Lstate = 2; Rstate = 1;
      }
    else if ((signal == 0) && (signal2 == 0 )){
      selesai = false;
      Lstate = 2; Rstate = 2;
      }
    else if(signal != 0){
      newangle1 = angle1;
      newangle2 = angle2;
      selesai = false;
      Lstate = 1; Rstate = 2;
      }
    }    
  }

  else if (Rstate == "simpan" && Lstate == 1){
    stepper.moveTo(newangle1);
    stepper2.moveTo(newangle2);
    stepperR.moveTo(-32 * 50 * 3.75);
    stepper2R.moveTo(3.75 * 50);
    if (stepper.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepperR.distanceToGo()==0&& stepper2R.distanceToGo()==0){
    if ((signal2 != 0)){
      newangle1R = angle1R;
      newangle2R = angle2R;
      selesai = true;
      Lstate = "simpan"; Rstate = 1;
      }
    else if(signal2 == 0 ){
      selesai = true;
      Lstate = "simpan"; Rstate = 2;
      }
    }    
  }
  stepperR.run();
  stepper2R.run();
  stepper.run();
  stepper2.run();
  // Serial.println(Rstate + " " + Lstate);

}

void Arm::moveR() {
    stepperR.moveTo(angle1R);
    stepper2R.moveTo(angle2R);
    stepperR.run();
    stepper2R.run();
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
      stepper.setMaxSpeed(3000);
      stepper.setAcceleration(3000);
    }
    if (kalibrasiatasL == 0){
      if (stepper.distanceToGo() == 0){
      stepper2.run();}    }
    if (digitalRead(_atas) == HIGH){
      kalibrasiatasL = 1;
      stepper2.stop();
      stepper2.setCurrentPosition(0);
      stepper2.moveTo(stepper2.currentPosition());
      stepper2.setMaxSpeed(400);
      stepper2.setAcceleration(400);
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
      stepperR.setMaxSpeed(3000);
      stepperR.setAcceleration(3000);
    }
    if (kalibrasiatasR == 0){
      if (stepperR.distanceToGo() == 0){
      stepper2R.run();}
    }
    if (digitalRead(_atasR) == HIGH){
      kalibrasiatasR = 1;
      // stepper2R.stop();
      stepper2R.setCurrentPosition(0);
      stepper2R.moveTo(stepper2R.currentPosition());
      stepper2R.setMaxSpeed(200);
      stepper2R.setAcceleration(200);
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
    const float L_atas = 50, L_bwh = 50;
    angle1 = -(32 * 3.75 * 57.29578 * (-atan(dx / dy) + acos((dx * dx + dy * dy + L_bwh * L_bwh - L_atas * L_atas) / (2 * sqrt(dx * dx + dy * dy) * L_bwh)))) / 1.8;
    angle2 = -3.75 * 200 + 3.75 * 7 + 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwh * L_bwh + L_atas * L_atas) / (2 * L_atas * L_bwh))) / 1.8;
}
void Arm::calculateR(float dx, float dy) {
    const float L_atasR = 50, L_bwhR = 50;
    angle1R = (32 * 3.75 * 57.29578 * (atan(dx / dy) + acos((dx * dx + dy * dy + L_bwhR * L_bwhR - L_atasR * L_atasR) / (2 * sqrt(dx * dx + dy * dy) * L_bwhR)))) / 1.8;
    angle2R = 3.75 * 200 - 3.75 * 7 - 3.75 * 57.29578 * (acos((-dx * dx - dy * dy + L_bwhR * L_bwhR + L_atasR * L_atasR) / (2 * L_atasR * L_bwhR))) / 1.8;
}

bool Arm::done() {
    selesai = (stepper.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepperR.distanceToGo() == 0 && stepper2R.distanceToGo() == 0);
    return(selesai);
}
