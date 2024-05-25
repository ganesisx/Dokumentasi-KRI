// Don't forget to put the lib files (both cpp and h) in the main folder

/***************************************************************************************
main function to parse data from serial and control the actuator. It also takes data f
rom sensors and send it to python script through serial.

Serial Setup:
  v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_type arm2_posX arm2_posY arm2_type
  Character v is flag for the right command
  Trash type in variable arm1_type or arm2_type represented as int as follows
  * 0 : 
  * 1 : 
  * 2 : 
  * 3 : 
  * 4 : 

There are two ways to debug this code,
   * OLED 128x64 Display
   * Serial Monitor
It is preferred to use the OLED Display since the serial is also being used by python
script

Current Version (Feb 11 2024): Not supporting serial write to python script

Copyright (c) 2024, Ganesis KRTMI ITB.
All rights reserved.

Debug input example: 
v 125 -125 125 -125 321.12345 -321.12345 1 321.12345 -321.12345 1 321.12345 -321.12345 1

***************************************************************************************/
#include "Driver.h"
#include "Encoder.h"
#include "Arm.h"
#include "Pid.h"
#include <Servo.h>


//======== OBject Declarations ========//
// Motor Driver 2 pin
Driver motor0(6,7);
Driver motor1(8,9);
Driver motor2(10,11);
Driver motor3(12,4);
// Driver motor0(999,999);
// Driver motor1(998,999);
// Driver motor2(910,911);
// Driver motor3(912,913);
// Encoder
#define ENCODER
Encoder encMotor0(63, 62, 330);
Encoder encMotor1(66, 67, 330);
Encoder encMotor2(64, 65, 330);
Encoder encMotor3(69, 68, 330);

const int PPR = 2000;
float pos = 0, enc_pulse = 0;
const int channelA = 2, channelB=3;
int pinA, pinB; //Condition of channelA and channel B
float r_wheel = 3.0; // whel radius
float jarak_checkpoint = 140.0;

// Encoder encMotor0(63, 64, 330);
// Encoder encMotor1(65, 66, 330);
// Encoder encMotor2(67, 68, 330);
// Encoder encMotor3(69, 70, 330);

//PID Maju
// kp, ki, dan kd
double kp1m[] = {
  4,  // Motor 0
  4,  // Motor 1
  3.75,  // Motor 2
  3.75  // Motor 3
};
double ki1m[] = {
  0.9,   // Motor 0
  0.9,   // Motor 1
  0.75,   // Motor 2
  0.75 // Motor 3
};
double kd[] = {
  0.0,   // Motor 0
  0.0,   // Motor 1
  0.0,   // Motor 2
  0.0     // Motor 3
};

// PID Kiri
double kp1L[] = {4, 2, 2, 4};
double ki1L[] = {0.9, 0.35, 0.35, 0.9};

// PID Kanan
double kp1R[] = {4, 3, 3.75, 3.75};
double ki1R[] = {0.9, 0.5, 0.75, 0.75};

// PID Mundur
double kp1B[] = {4, 3, 2, 3.75};
double ki1B[] = {0.9, 0.5, 0.35, 0.75};

// // PID Mundur
// double kp1B[] = {4, 4, 3.75, 3.75};
// double ki1B[] = {0.9, 0.9, 0.75, 0.75};

// PID
PID pid1m[] = {
  //PID maju
  PID(kp1m[0], ki1m[0], kd[0], 1000),  // Motor 0
  PID(kp1m[1], ki1m[1], kd[1], 1000),  // Motor 1
  PID(kp1m[2], ki1m[2], kd[2], 1000),  // Motor 2
  PID(kp1m[3], ki1m[3], kd[3], 1000)   // Motor 3
};

PID pid1L[] = {
  //PID kiri
  PID(kp1L[0], ki1L[0], kd[0], 1000),  // Motor 0
  PID(kp1L[1], ki1L[1], kd[1], 1000),  // Motor 1
  PID(kp1L[2], ki1L[2], kd[2], 1000),  // Motor 2
  PID(kp1L[3], ki1L[3], kd[3], 1000)   // Motor 3
};

PID pid1R[] = {
  //PID Kanan
  PID(kp1R[0], ki1R[0], kd[0], 1000),  // Motor 0
  PID(kp1R[1], ki1R[1], kd[1], 1000),  // Motor 1
  PID(kp1R[2], ki1R[2], kd[2], 1000),  // Motor 2
  PID(kp1R[3], ki1R[3], kd[3], 1000)   // Motor 3
};

PID pid1B[] = {
  //PID Mundur
  PID(kp1B[0], ki1B[0], kd[0], 1000),  // Motor 0
  PID(kp1B[1], ki1B[1], kd[1], 1000),  // Motor 1
  PID(kp1B[2], ki1B[2], kd[2], 1000),  // Motor 2
  PID(kp1B[3], ki1B[3], kd[3], 1000)   // Motor 3
};



//======== Variables ========//
const int trigkanan = 28;
const int echokanan = 24;
const int trigkiri = 47;
const int echokiri = 45;
// #define trigtengah A3
// #define echotengah A4
float distance_kanan;
float distance_kiri;
float distance_tengah;
float duration_kanan;
float duration_kiri;
float duration_tengah;
float set_jarak_kanan = 10;
float set_jarak_kiri = 10;
float set_jarak_tengah = 10;

unsigned long start;
unsigned long start_search;
unsigned long stop_search = 2000;
int count = 0;
int checkpoint = 0;

#define BUTTON_MAIN A2


// Motor speed setpoint
float spd0, spd1, spd2, spd3;

// Current Motor Speed
float currspd0, currspd1, currspd2, currspd3;
// PWM
double pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3;

// Motor Stepper driver pin
Arm arm(48, 46, 29, 31, 52, 50, 33, 35, 26, 53, 49, 51);

// // arm
// Servo servoL, servoR;

// Arm structs
struct {
  int type;   // Trash type
  float posX; // X coordinate target
  float posY; // Y coordinate target
} arm1, arm2;


//======== Serial Setup Between Microcontroller to SBC Using USB ========//
/* SERIAL SETUP FOR INCOMING DATA
v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_type arm2_posX arm2_posY arm2_type
character v is flag for the right command from sbc to arduino
NOTE: space is the delimiter
*/

const int data_length = 8;  // Variable to hold each serial data length
char data[data_length];     // Variable to hold arguments
int chr;                   // Variable to hold an input character
int state = -1;             // Variable to determine which parmeter is being read
short idx = 0;              // Variable of data array index
void parse_data();          // Parsing incoming data
void input_commands();      // Insert value to desired variable after parsing serial input
void clear_commands();      // Clear the current command parameters

//======== Debugging ========//
/* !!NOTE: Use this if you're not running the python scripts
Debugging Read Value using Serial Monitor
Comment the define and uncomment the undef if not used and vice versa
*/
// #define DEBUG
// #undef DEBUG

// #if defined(DEBUG) || defined(DISPLAY)
//   void print_commands();  // Function to print input values
// #endif

/* 
Debugging Read Value using OLED 128 x 64
Comment the define and uncomment the undef if not used and vice versa
*/
// #define DISPLAY
// #undef DISPLAY

// #ifdef DISPLAY
//   #include <SPI.h>
//   #include <Wire.h>
//   #include <Adafruit_GFX.h>
//   #include <Adafruit_SSD1306.h>

//   #define SCREEN_WIDTH 128 // OLED display width, in pixels
//   #define SCREEN_HEIGHT 64 // OLED display height, in pixels

//   // Declaration for SSD1306 display connected using I2C
//   #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//   #define SCREEN_ADDRESS 0x3C
//   Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//   void new_display(String text, int x, int y) {
//     display.clearDisplay();
//     display.setCursor(x,y);
//     display.println(text);
//     display.display();
//     display.clearDisplay();
//   }
// #endif
void handleInterrupt0() {
  // Serial.println("0");
  encMotor0.encode();
}

void handleInterrupt1() {
  // Serial.println("1");
  encMotor1.encode();
}

void handleInterrupt2() {
  // Serial.println("2");
  encMotor2.encode();
}

void handleInterrupt3() {
  // Serial.println("3");
  encMotor3.encode();
}

void ISR_ENCA() {
  pinA = digitalRead(channelA);
  pinB = digitalRead(channelB);

  if(pinA==LOW && pinB==LOW)    enc_pulse--; // CCW
  if(pinA==LOW && pinB==HIGH)   enc_pulse++; // CW
  if(pinA==HIGH && pinB==LOW)   enc_pulse++; // CW
  if(pinA==HIGH && pinB==HIGH)  enc_pulse--; // CCW
}
void ISR_ENCB() {
  pinA = digitalRead(channelA);
  pinB = digitalRead(channelB);

  if(pinA==LOW && pinB==LOW)    enc_pulse++; // CCW
  if(pinA==LOW && pinB==HIGH)   enc_pulse--; // CW
  if(pinA==HIGH && pinB==LOW)   enc_pulse--; // CW
  if(pinA==HIGH && pinB==HIGH)  enc_pulse++; // CCW
}
void tengah() {
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  Serial.print("pos =");
  Serial.println(pos);


//   if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri) {
//     spd0 = 16;
//     spd1 = -16;
//     spd2 = 16;
//     spd3 = -16;
//   }
//   else if(distance_kanan <= set_jarak_kanan && distance_kiri > set_jarak_kiri && distance_kiri < 50){
//     spd0 = 16;
//     spd1 = -16;
//     spd2 = 16;
//     spd3 = -16;
//   }
//   else if(distance_kanan > set_jarak_kanan && distance_kiri > set_jarak_kiri && distance_kiri < 50){
//     spd0 = 16;
//     spd1 = -16;
//     spd2 = 16;
//     spd3 = -16;
//   }
//   else if(distance_kanan > set_jarak_kanan && distance_kiri <= set_jarak_kiri){
//     spd0 = 16;
//     spd1 = -16;
//     spd2 = 16;
//     spd3 = -16;
//   }



  motor0.set_motor_speed((pwmMotor0));
  motor1.set_motor_speed((pwmMotor1));
  motor2.set_motor_speed((pwmMotor2));
  motor3.set_motor_speed((pwmMotor3));

  currspd0 = encMotor0.getPulses();
  currspd1 = encMotor1.getPulses();
  currspd2 = encMotor2.getPulses();
  currspd3 = encMotor3.getPulses();

  encMotor0.reset();
  encMotor1.reset();
  encMotor2.reset();
  encMotor3.reset();

  pwmMotor0 = pid1m[0].createpwm(spd0, currspd0);
  pwmMotor1 = pid1m[1].createpwm(spd1, currspd1);
  pwmMotor2 = pid1m[2].createpwm(spd2, currspd2);
  pwmMotor3 = pid1m[3].createpwm(spd3, currspd3);

  if (pos >= -3.25 && pos <= 3.25) {
    if (state == 3) state = 2;
    else if (state == 4) {state = 5; arm.Lstate = 4; arm.Rstate = 4;}
    delay(1000);

  }
}

void kiri() {
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();


  if (pos <=200 && pos >= -20) {
    spd0 = 25;
    spd1 = -40;
    spd2 = 25;
    spd3 = -40;

    
  motor0.set_motor_speed((pwmMotor0));
  motor1.set_motor_speed((pwmMotor1));
  motor2.set_motor_speed((pwmMotor2));
  motor3.set_motor_speed((pwmMotor3));

  currspd0 = encMotor0.getPulses();
  currspd1 = encMotor1.getPulses();
  currspd2 = encMotor2.getPulses();
  currspd3 = encMotor3.getPulses();

  encMotor0.reset();
  encMotor1.reset();
  encMotor2.reset();
  encMotor3.reset();

  pwmMotor0 = pid1B[0].createpwm(spd0, currspd0);
  pwmMotor1 = pid1B[1].createpwm(spd1, currspd1);
  pwmMotor2 = pid1B[2].createpwm(spd2, currspd2);
  pwmMotor3 = pid1B[3].createpwm(spd3, currspd3);
  }
  else if (pos < -20) {
    spd0 = 5;
    spd1 = -50;
    spd2 = 5;
    spd3 = -50;
    
  motor0.set_motor_speed((pwmMotor0));
  motor1.set_motor_speed((pwmMotor1));
  motor2.set_motor_speed((pwmMotor2));
  motor3.set_motor_speed((pwmMotor3));

  currspd0 = encMotor0.getPulses();
  currspd1 = encMotor1.getPulses();
  currspd2 = encMotor2.getPulses();
  currspd3 = encMotor3.getPulses();

  encMotor0.reset();
  encMotor1.reset();
  encMotor2.reset();
  encMotor3.reset();

  pwmMotor0 = pid1L[0].createpwm(spd0, currspd0);
  pwmMotor1 = pid1L[1].createpwm(spd1, currspd1);
  pwmMotor2 = pid1L[2].createpwm(spd2, currspd2);
  pwmMotor3 = pid1L[3].createpwm(spd3, currspd3);
  }

  // distance_kanan, distance_kiri = ultrasonic();

  // Serial.print("pos (kiri) =");
  // Serial.println(pos);

  // if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri) {
  //   spd0 = 7;
  //   spd1 = -10;
  //   spd2 = 7;
  //   spd3 = -10;
  // }
  // else if(distance_kanan <= set_jarak_kanan && distance_kiri > set_jarak_kiri && distance_kiri < 50){
  //   spd0 = 7;
  //   spd1 = -10;
  //   spd2 = 7;
  //   spd3 = -10;
  // }
  // else if(distance_kanan > set_jarak_kanan && distance_kiri > set_jarak_kiri && distance_kiri < 50){
  //   spd0 = 10;
  //   spd1 = -7;
  //   spd2 = 10;
  //   spd3 = -7;
  // }
  // else if(distance_kanan > set_jarak_kanan && distance_kiri <= set_jarak_kiri){
  //   spd0 = 10;
  //   spd1 = -8;
  //   spd2 = 8;
  //   spd3 = -10;
  // }
  // else if(distance_kiri > 50) state = 3;


}

void kanan() {
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (pos <= 70) {
    spd0 = -40;
    spd1 = 25;
    spd2 = -40;
    spd3 = 25;
  }
  else if(pos >70){
    spd0 = -50;
    spd1 = 5;
    spd2 = -50;
    spd3 = 5;
  }

  // spd0 = -12;
  // spd1 = 6;
  // spd2 = -12;
  // spd3 = 6;
  // if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri) {
  //   spd0 = -16;
  //   spd1 = 16;
  //   spd2 = -16;
  //   spd3 = 16;
  // }
  // else if(distance_kanan <= set_jarak_kanan && distance_kiri > set_jarak_kiri){
  //   spd0 = -16;
  //   spd1 = 16;
  //   spd2 = -16;
  //   spd3 = 16;
  // }
  // else if(distance_kanan > set_jarak_kanan && distance_kiri > set_jarak_kiri && distance_kanan > 50){
  //   spd0 = -16;
  //   spd1 = 16;
  //   spd2 = -16;
  //   spd3 = 16;
  // }
  // else if(distance_kanan > set_jarak_kanan && distance_kiri <= set_jarak_kiri && distance_kanan > 50){
  //   spd0 = -16;
  //   spd1 = 16;
  //   spd2 = -16;
  //   spd3 = 16;
  // }

  motor0.set_motor_speed((pwmMotor0));
  motor1.set_motor_speed((pwmMotor1));
  motor2.set_motor_speed((pwmMotor2));
  motor3.set_motor_speed((pwmMotor3));

  currspd0 = encMotor0.getPulses();
  currspd1 = encMotor1.getPulses();
  currspd2 = encMotor2.getPulses();
  currspd3 = encMotor3.getPulses();

  encMotor0.reset();
  encMotor1.reset();
  encMotor2.reset();
  encMotor3.reset();

  pwmMotor0 = pid1R[0].createpwm(spd0, currspd0);
  pwmMotor1 = pid1R[1].createpwm(spd1, currspd1);
  pwmMotor2 = pid1R[2].createpwm(spd2, currspd2);
  pwmMotor3 = pid1R[3].createpwm(spd3, currspd3);
}

void maju() {

  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri) {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    state = 0;
    
  }

  else{ 
    if (distance_kanan > set_jarak_kanan && distance_kiri > set_jarak_kiri) {
      spd0 = -50;
      spd1 = -50;
      spd2 = -50;
      spd3 = -50;
    }
    else if (distance_kanan < set_jarak_kanan && distance_kiri > set_jarak_kiri) {
      spd0 = -20;
      spd1 = -20;
      spd2 = 0;
      spd3 = 0;
    }
    else if (distance_kiri < set_jarak_kiri && distance_kanan > set_jarak_kanan) {
      spd0 = 0;
      spd1 = 0;
      spd2 = -20;
      spd3 = -20;
    }
    currspd0 = encMotor0.getPulses();
    currspd1 = encMotor1.getPulses();
    currspd2 = encMotor2.getPulses();
    currspd3 = encMotor3.getPulses();

    encMotor0.reset();
    encMotor1.reset();
    encMotor2.reset();
    encMotor3.reset();

    pwmMotor0 = pid1m[0].createpwm(spd0, currspd0);
    pwmMotor1 = pid1m[1].createpwm(spd1, currspd1);
    pwmMotor2 = pid1m[2].createpwm(spd2, currspd2);
    pwmMotor3 = pid1m[3].createpwm(spd3, currspd3);

    motor0.set_motor_speed((pwmMotor0));
    motor1.set_motor_speed((pwmMotor1));
    motor2.set_motor_speed((pwmMotor2));
    motor3.set_motor_speed((pwmMotor3));

  }
  
  // else{
  //   spd0 = -20;
  //   spd1 = -20;
  //   spd2 = -20;
  //   spd3 = -20;
  // }
}



void maju_kanan() {

  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kiri <= set_jarak_kiri) {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    state = 0;
    
  }

  else{ 
      spd0 = -50;
      spd1 = -50;
      spd2 = -50;
      spd3 = -50;
    }

  // if (distance_tengah <= set_jarak_kanan && distance_kiri <= set_jarak_kiri) {
  //   motor0.set_motor_speed((0));
  //   motor1.set_motor_speed((0));
  //   motor2.set_motor_speed((0));
  //   motor3.set_motor_speed((0));
  //   stop();
  //   state = 0;
    
  // }

  // else{ 
  //   if (distance_tengah > set_jarak_kanan && distance_kiri > set_jarak_kiri) {
  //     spd0 = -50;
  //     spd1 = -50;
  //     spd2 = -50;
  //     spd3 = -50;
  //   }
  //   else if (distance_tengah < set_jarak_kanan && distance_kiri > set_jarak_kiri) {
  //     spd0 = -20;
  //     spd1 = -20;
  //     spd2 = 0;
  //     spd3 = 0;
  //   }
  //   else if (distance_kiri < set_jarak_kiri && distance_tengah > set_jarak_kanan) {
  //     spd0 = 0;
  //     spd1 = 0;
  //     spd2 = -20;
  //     spd3 = -20;
  //   }
  // }



    currspd0 = encMotor0.getPulses();
    currspd1 = encMotor1.getPulses();
    currspd2 = encMotor2.getPulses();
    currspd3 = encMotor3.getPulses();

    encMotor0.reset();
    encMotor1.reset();
    encMotor2.reset();
    encMotor3.reset();

    pwmMotor0 = pid1m[0].createpwm(spd0, currspd0);
    pwmMotor1 = pid1m[1].createpwm(spd1, currspd1);
    pwmMotor2 = pid1m[2].createpwm(spd2, currspd2);
    pwmMotor3 = pid1m[3].createpwm(spd3, currspd3);

    motor0.set_motor_speed((pwmMotor0));
    motor1.set_motor_speed((pwmMotor1));
    motor2.set_motor_speed((pwmMotor2));
    motor3.set_motor_speed((pwmMotor3));

  
  // else{
  //   spd0 = -20;
  //   spd1 = -20;
  //   spd2 = -20;
  //   spd3 = -20;
  // }
}



void maju_kiri() {

  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kanan <= set_jarak_kanan) {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    state = 0;
    
  }

  else{ 
      spd0 = -50;
      spd1 = -50;
      spd2 = -50;
      spd3 = -50;
    
    }

  // if (distance_kanan <= set_jarak_kanan && distance_tengah <= set_jarak_kiri) {
  //   motor0.set_motor_speed((0));
  //   motor1.set_motor_speed((0));
  //   motor2.set_motor_speed((0));
  //   motor3.set_motor_speed((0));
  //   stop();
  //   state = 0;
    
  // }

  // else{ 
  //   if (distance_kanan > set_jarak_kanan && distance_tengah > set_jarak_kiri) {
  //     spd0 = -50;
  //     spd1 = -50;
  //     spd2 = -50;
  //     spd3 = -50;
  //   }
  //   else if (distance_kanan < set_jarak_kanan && distance_tengah > set_jarak_kiri) {
  //     spd0 = -20;
  //     spd1 = -20;
  //     spd2 = 0;
  //     spd3 = 0;
  //   }
  //   else if (distance_tengah < set_jarak_kiri && distance_kanan > set_jarak_kanan) {
  //     spd0 = 0;
  //     spd1 = 0;
  //     spd2 = -20;
  //     spd3 = -20;
  //   }
  // }
    currspd0 = encMotor0.getPulses();
    currspd1 = encMotor1.getPulses();
    currspd2 = encMotor2.getPulses();
    currspd3 = encMotor3.getPulses();

    encMotor0.reset();
    encMotor1.reset();
    encMotor2.reset();
    encMotor3.reset();

    pwmMotor0 = pid1m[0].createpwm(spd0, currspd0);
    pwmMotor1 = pid1m[1].createpwm(spd1, currspd1);
    pwmMotor2 = pid1m[2].createpwm(spd2, currspd2);
    pwmMotor3 = pid1m[3].createpwm(spd3, currspd3);

    motor0.set_motor_speed((pwmMotor0));
    motor1.set_motor_speed((pwmMotor1));
    motor2.set_motor_speed((pwmMotor2));
    motor3.set_motor_speed((pwmMotor3));

  
  // else{
  //   spd0 = -20;
  //   spd1 = -20;
  //   spd2 = -20;
  //   spd3 = -20;
  // }
}

void stop() {
  spd0 = 0;
  spd1 = 0;
  spd2 = 0;
  spd3 = 0;
  motor0.set_motor_speed((0));
  motor1.set_motor_speed((0));
  motor2.set_motor_speed((0));
  motor3.set_motor_speed((0));

  // currspd0 = encMotor0.getPulses();
  // currspd1 = encMotor1.getPulses();
  // currspd2 = encMotor2.getPulses();
  // currspd3 = encMotor3.getPulses();

  // encMotor0.reset();
  // encMotor1.reset();
  // encMotor2.reset();
  // encMotor3.reset();

  // pwmMotor0 = pid1m[0].createpwm(spd0, currspd0);
  // pwmMotor1 = pid1m[1].createpwm(spd1, currspd1);
  // pwmMotor2 = pid1m[2].createpwm(spd2, currspd2);
  // pwmMotor3 = pid1m[3].createpwm(spd3, currspd3);
}

void setup() {
  Serial.begin(500000);
  //Encoder motor
  attachPCINT(digitalPinToPCINT(63), handleInterrupt0, CHANGE);
  attachPCINT(digitalPinToPCINT(66), handleInterrupt1, CHANGE);
  attachPCINT(digitalPinToPCINT(64), handleInterrupt2, CHANGE);
  attachPCINT(digitalPinToPCINT(69), handleInterrupt3, CHANGE);
  //Extra Encoder
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelA), ISR_ENCA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB), ISR_ENCB, CHANGE);

  pinMode(43, OUTPUT); // pin enable stepper nema 17
  analogWrite(43, LOW);

  pinMode(trigkanan, OUTPUT);
  pinMode(echokanan, INPUT);
  pinMode(trigkiri, OUTPUT);
  pinMode(echokiri, INPUT);

  pinMode(BUTTON_MAIN, INPUT);
  arm.servoL.attach(13); arm.servoR.attach(5);
  arm.servoL.write(90); arm.servoR.write(90);
  // servoL.writeMicroseconds(2000); servoR.writeMicroseconds(1500);
  // arm.servo_stop(arm.servoL);
  // arm.servo_stop(arm.servoR);
  Serial.println("tes");
  // arm.servo_setup();
  Serial.setTimeout(10);
  arm.kalibrasi();
  arm.kalibrasi();
  arm.selesai = true;
  arm.Lstate = 2; arm.Rstate = 2;
  
  // while(!Serial) {} // Wait until serial connection is set

  // #ifdef DISPLAY
  //   #ifdef DEBUG
  //     Serial.println("Testing with both Serial Monitor and OLED Screen");
  //   #endif
  //   // initialize the OLED object
  //   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //     Serial.println(F("SSD1306 allocation failed"));
  //     for(;;); // Don't proceed, loop forever
  //   }

  //   // Clear the buffer.
  //   display.clearDisplay();

  //   // Display Text
  //   display.setTextSize(1);
  //   display.setTextColor(WHITE);
  //   display.setCursor(0,28);
  //   display.println("Waiting...");
  //   display.display();
  //   display.clearDisplay();
  // #endif

  // #ifdef DEBUG
  //   Serial.println("Testing Values Using Serial Montior");
  // #endif
  delay(5000);
}

float ultrasonic() {
      // Trigger ultrasonic sensor 1
  digitalWrite(trigkanan, LOW);
  delayMicroseconds(2);
  digitalWrite(trigkanan, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigkanan, LOW);
  duration_kanan = pulseIn(echokanan, HIGH);
  distance_kanan = duration_kanan * 0.034 / 2;

  // Trigger ultrasonic sensor 2
  digitalWrite(trigkiri, LOW);
  delayMicroseconds(2);
  digitalWrite(trigkiri, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigkiri, LOW);
  duration_kiri = pulseIn(echokiri, HIGH);
  distance_kiri = duration_kiri * 0.034 / 2;
  
  // // Trigger ultrasonic sensor 3 (tengah)
  // digitalWrite(trigtengah, LOW);
  // delayMicroseconds(2);
  // digitalWrite(trigtengah, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigtengah, LOW);
  // duration_tengah = pulseIn(echotengah, HIGH);
  distance_tengah = 10 * 0.034 / 2; // duration_tengah * 0.034 / 2;
  
  return distance_kanan, distance_kiri, distance_tengah;
}
void loop() {
  // Serial.print(distance_kiri); Serial.print('\t'); Serial.println(distance_kanan);
  if (digitalRead(BUTTON_MAIN) == LOW) {
    stop();
    distance_kanan, distance_kiri, distance_tengah = ultrasonic();
    // Serial.print(distance_kanan);Serial.print("   ");Serial.println(distance_kiri);
  }
  else{
      
  // Serial.print("pos= ");

  // Serial.print(state);
  // Serial.print("\t");
  // Serial.print(pos);
  // Serial.print("\t");
  // Serial.print(currspd0);
  // Serial.print("\t");
  // Serial.print(currspd1);
  // Serial.print("\t");
  // Serial.print(currspd2);
  // Serial.print("\t");
  // Serial.print(currspd3);
  // Serial.print("\t|");
  // Serial.print(spd0);
  // Serial.print("\t");
  // Serial.print(spd1);
  // Serial.print("\t");
  // Serial.print(spd2);
  // Serial.print("\t");
  // Serial.print(spd3);
  // Serial.print("\t|");
  // Serial.print("pwm");
  // Serial.print("\t");
  // Serial.print(pwmMotor0);
  // Serial.print("\t");
  // Serial.print(pwmMotor1);
  // Serial.print("\t");
  // Serial.print(pwmMotor2);
  // Serial.print("\t");
  // Serial.print(pwmMotor3);
  // Serial.print("\t|");
  // Serial.print(distance_kanan);
  // Serial.print("\t");
  // Serial.print(distance_kiri);
  // Serial.print("\t");
  // Serial.print(distance_tengah);
  // Serial.println("---------");
    // Position from Extra Encoder
    pos =  ((float)enc_pulse * 0.000125) * 6.2832 * r_wheel;
    // Serial.print(pos);
    // Serial.print("\t");
    // Serial.print(checkpoint);
    // Serial.print("\t");
    // Serial.println(state);

    // debug state
    // state = 2;

    // Serial.print(enc_pulse);
    if (state == -1) {
      maju();
    }
    // move arm
    else if (state == 0) {
      stop();
      // Read and parse incoming data
      if(Serial.available()>0 && arm.selesai) { //arm.selesai dihapus sementara
        Serial.println("state 0");
        chr = Serial.parseInt();
        if (chr == -999) {
          parse_data();
          
        }
      }
      arm.move(arm1.type, arm2.type);
    }

    else if (state == 1) { // jalan ke kanan
      count = 0;
      kanan();
      if (pos >= jarak_checkpoint) {
        state = 6;
        checkpoint += 1;
      }
    }

    else if (state == 2) { // jalan ke kiri
      count = 0;
      kiri();
      if (pos <= -110) {
        state = 7;
        checkpoint = 2;
      }
    }
    else if (state == 6) { // jalan ke kiri
      maju_kanan();
    }
    else if (state == 7) { // jalan ke kiri
      maju_kiri();
    }

    else if (state == 3) { // jalan ke  dari kanan
      count = 0;
      spd0 = 16;
      spd1 = -16;
      spd2 = 16;
      spd3 = -16;
      tengah();

    }
      else if (state == 4) { // jalan ke tengah dari kiri
      count = 0;
      kiri();
      if (pos <= 10 && pos >=-10) {
        state = 0;
        checkpoint = 2;
      }
      // spd0 = -16;
      // spd1 = 16;
      // spd2 = -16;
      // spd3 = 16;
      // tengah();

    }
    else if (state == 5) { // buang sampah
      stop();
      if (arm1.type != 6 && arm2.type != 6){
        parse_data();
        arm.calculateBuang(arm1.posX, arm1.posY, arm2.posX, arm2.posY);
      }
      else {
        arm.buang();
      }
      Serial.println("Buang Sampah");
      count = 0;
      
    }
    // start = millis();


    //     }
    //   }
    // }
    
    // arm.moveR();
    // if (arm.selesai){
    //   Serial.println("1");
    // }
    // Send data to sbc
  
}}

/*========== FUNCTIONS DEFINITIONS ==========*/
void parse_data() {
  if (Serial.available()>0 && arm.selesai ){ //arm.selesai dihapus sementara
  // while (Serial.available() > 0) {
  //   // Read the next character
  //   chr = Serial.read();

  //   #ifdef DEBUG
  //     Serial.println("Char Data: " + String(chr));
  //     Serial.println(chr,DEC);
  //   #endif

  //   if (state == -1) {
  //     if (chr == 'v') state = 0;
  //       else {
  //       #ifdef DEBUG
  //         Serial.println("Invalid Commands!");
  //       #endif 
  //       continue;
  //     }
  //   }
  //   else if (chr == 10 | chr == 13) { // newline
  //     data[idx] = NULL;
  //     input_commands();
  //     clear_commands();

  //     state = -1;
  //     idx = 0;
  //     #if defined(DEBUG) || defined(DISPLAY)
  //       print_commands();
  //     #endif
  //   }
  //   else if (chr == 32) { // Detect space
  //     data[idx] = NULL;
  //     input_commands();
  //     clear_commands();
  //     state++;
  //     idx = 0;
  //   }
  //   else {
  //     data[idx] = chr;
  //     idx++;
  //   }
  // chr = Serial.readString();
  spd0 = Serial.parseInt(); spd1 = Serial.parseInt(); spd2 = Serial.parseInt(); spd3 = Serial.parseInt();
  arm1.posX = Serial.parseFloat(); arm1.posY = Serial.parseFloat(); arm1.type = Serial.parseInt();
  arm2.posX = Serial.parseFloat(); arm2.posY = Serial.parseFloat(); arm2.type = Serial.parseInt();
  float skip = Serial.parseFloat();

  Serial.print("S"); Serial.print(", ");
  Serial.print(arm1.posX); Serial.print(","); Serial.print(arm1.posY); Serial.print(",");
  Serial.print(arm2.posX); Serial.print(","); Serial.println(arm2.posY);
  // }
  if(arm1.type != 0){
    if(arm1.posX != 0 || arm1.posY != 0) {
      arm.calculate(arm1.posX, arm1.posY);
    }
  }

  if(arm2.type != 0){
    if (arm2.posX != 0 || arm2.posY != 0) {
      arm.calculateR(arm2.posX, arm2.posY);
    }
  }
  if(arm1.type <= 0 && arm2.type <= 0) {
    count += 1;
    Serial.println(count);
    if (count == 2) {
      if (checkpoint == 0) {
        state = 1;
        Serial.println("Chechkpoint 0 selesai");
      }
      else if (checkpoint == 1) {
        state = 2;
        Serial.println("Chechkpoint 1 selesai");
      }
      else if (checkpoint == 2) {
        state = 4;
        Serial.println("Chechkpoint 2 selesai");
      }
    }
  }

  // if (arm.done()){
  //   //  Serial.println("Char Data: " + String(chr));
  //   delay(500);
  //   Serial.print("POS ");
    arm.selesai = false;
    }
}

void input_commands() {
  switch (state) {
    /* ============= MOTOR TARGET SPEED ============= */
    case 1:
      spd0 = atoi(data);
      break;
    case 2:
      spd1 = atoi(data);
      break;
    case 3:
      spd2 = atoi(data);
      break;
    case 4:
      spd3 = atoi(data);
      break;
    
    /* ============= ARM1 ============= */
    case 5:
      arm1.posX = atof(data);
      break;
    case 6:
      arm1.posY = atof(data);
      break;
    case 7:
      arm1.type = atoi(data);
      break;

    /* ============= ARM2 ============= */
    case 8:
      arm2.posX = atof(data);
      break;
    case 9:
      arm2.posY = atof(data);
      break;
    case 10:
      arm2.type = atoi(data);
      break;
    /* ============= SWEEP ARM ============= */

    default:
      clear_commands();
      break;
  }
}



void clear_commands() {
  memset(data, 0, data_length);
}

// #if defined(DEBUG) || defined (DISPLAY)
//   void print_commands() {
//     #ifdef DEBUG
//       Serial.println("This goes");
//       char data[8];
//       String outString = "PWM_0 = " + String(spd0) +"\n";
//       outString += "PWM_1 = " + String(spd1) +"\n";
//       outString += "PWM_2 = " + String(spd2) +"\n";
//       outString += "PWM_3 = " + String(spd3) +"\n";

//       outString += "arm1_posX = " + String(m_arm1.posX) +"\n";
//       outString += "arm1_posY = " + String(m_arm1.posY) +"\n";
//       outString += "arm1_grip = " + String(m_arm1.grip) +"\n";

//       outString += "arm2_posX = " + String(m_arm2.posX) +"\n";
//       outString += "arm2_posY = " + String(m_arm2.posY) +"\n";
//       outString += "arm2_grip = " + String(m_arm2.grip) +"\n";

//       Serial.println(outString);
//     #endif

//     #ifdef DISPLAY 
//       // Uncomment the parameter needs to be tested. 
//       // If more than one, add more new_display function with different cursor
//       // and add more output string variable


//       String outDisp = "PWM_0 = " + String(spd0);
//       String outDisp = "PWM_0 = " + String(spd0);
//       String outDisp = "PWM_0 = " + String(spd0);
//       String outDisp = "PWM_0 = " + String(spd0);

//       String outDisp = "arm1_posX = " + String(m_arm1.posX);
//       String outDisp = "arm1_posY = " + String(m_arm1.posY);
//       String outDisp = "arm1_grip = " + String(m_arm1.grip);

//       String outDisp = "arm2_posX = " + String(m_arm2.posX);
//       String outDisp = "arm2_posY = " + String(m_arm2.posY);
//       String outDisp = "arm2_grip = " + String(m_arm2.grip);

//       new_display(outDisp, 0, 1);
//     #endif
//   }
// #endif
