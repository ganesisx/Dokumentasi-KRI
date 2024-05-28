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
Driver motor0(6, 7);
Driver motor1(8, 9);
Driver motor2(10, 11);
Driver motor3(12, 4);
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
const int channelA = 2, channelB = 3;
int pinA, pinB;      // Condition of channelA and channel B
float r_wheel = 3.0; // whel radius
float jarak_checkpoint = 110.0;

// Encoder encMotor0(63, 64, 330);
// Encoder encMotor1(65, 66, 330);
// Encoder encMotor2(67, 68, 330);
// Encoder encMotor3(69, 70, 330);

// PID Maju

// kp, ki, dan kd

double kp1m[] = {

    2.9, // Motor 0
    2, // Motor 1
    2.3, // Motor 2
    2.3 // Motor 3
    // 3.9, // Motor 0

    // 3.9, // Motor 1

    // 3.75, // Motor 2

    // 3.75 // Motor 3

};

double ki1m[] = {

    0.46, // Motor 0
    0.3, // Motor 1
    0.3, // Motor 2
    0.3 // Motor 3
    // 0.8, // Motor 0

    // 0.8, // Motor 1

    // 0.75, // Motor 2

    // 0.75 // Motor 3

};

double kd[] = {

    0.0, // Motor 0

    0.0, // Motor 1

    0.0, // Motor 2

    0.0 // Motor 3

};

// PID Kiri

double kp1L[] = {3, 2.5, 2.5, 3};

double ki1L[] = {0.5, 0.35, 0.35, 0.55};

// PID Kanan

double kp1R[] = {4, 3, 3, 3.75};

double ki1R[] = {0.9, 0.5, 0.5, 0.75};

// PID Mundur

double kp1B[] = {2, 2, 2, 2};

double ki1B[] = {0.3, 0.25, 0.25, 0.55};

// // PID Mundur

// double kp1B[] = {4, 4, 3.75, 3.75};

// double ki1B[] = {0.9, 0.9, 0.75, 0.75};

// PID

PID pid1m[] = {

    // PID maju

    PID(kp1m[0], ki1m[0], kd[0], 1000), // Motor 0

    PID(kp1m[1], ki1m[1], kd[1], 1000), // Motor 1

    PID(kp1m[2], ki1m[2], kd[2], 1000), // Motor 2

    PID(kp1m[3], ki1m[3], kd[3], 1000) // Motor 3

};

PID pid1L[] = {

    // PID kiri

    PID(kp1L[0], ki1L[0], kd[0], 1000), // Motor 0

    PID(kp1L[1], ki1L[1], kd[1], 1000), // Motor 1

    PID(kp1L[2], ki1L[2], kd[2], 1000), // Motor 2

    PID(kp1L[3], ki1L[3], kd[3], 1000) // Motor 3

};

PID pid1R[] = {

    // PID Kanan

    PID(kp1R[0], ki1R[0], kd[0], 1000), // Motor 0

    PID(kp1R[1], ki1R[1], kd[1], 1000), // Motor 1

    PID(kp1R[2], ki1R[2], kd[2], 1000), // Motor 2

    PID(kp1R[3], ki1R[3], kd[3], 1000) // Motor 3

};

PID pid1B[] = {

    // PID Mundur

    PID(kp1B[0], ki1B[0], kd[0], 1000), // Motor 0

    PID(kp1B[1], ki1B[1], kd[1], 1000), // Motor 1

    PID(kp1B[2], ki1B[2], kd[2], 1000), // Motor 2

    PID(kp1B[3], ki1B[3], kd[3], 1000) // Motor 3

};

// PID
// PID pid1m[] = {
//   //PID maju
//   PID(kp1m[0], ki1m[0], kd[0], 1000),  // Motor 0
//   PID(kp1m[1], ki1m[1], kd[1], 1000),  // Motor 1
//   PID(kp1m[2], ki1m[2], kd[2], 1000),  // Motor 2
//   PID(kp1m[3], ki1m[3], kd[3], 1000)   // Motor 3
// };

//======== Variables ========//
const int trigkanan = 28;
const int echokanan = 24;
const int trigkiri = 47;
const int echokiri = 45;
const int trigtengah = 30;
const int echotengah = 32;
float distance_kanan;
float distance_kiri;
float distance_tengah;
float duration_kanan;
float duration_kiri;
float duration_tengah;
float set_jarak_kanan = 6;
float set_jarak_kiri = 6;
float set_jarak_tengah = 6;

// IR
int pin_ir1 = 34;
int pin_ir2 = 36;
int pin_ir3 = A7;
int pin_ir4 = A4;
int pin_ir5 = A3;

int count = 0;
int checkpoint = 0;
int nyala = 0;
#define BUTTON_MAIN A2
unsigned long start_time, check_time;
// enum {MULAI, MAJU_TENGAH, KIRI, KANAN, MAJU_KANAN, MAJU_KIRI, KANAN_TENGAH, KALKULASI, BUANG, AMBIL};

String state = "mulai";

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
struct
{
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

const int data_length = 8; // Variable to hold each serial data length
char data[data_length];    // Variable to hold arguments
int chr;                   // Variable to hold an input character
// int state = -1;             // Variable to determine which parmeter is being read
short idx = 0;         // Variable of data array index
void parse_data();     // Parsing incoming data
void input_commands(); // Insert value to desired variable after parsing serial input
void clear_commands(); // Clear the current command parameters

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
void handleInterrupt0()
{
  // Serial.println("0");
  encMotor0.encode();
}

void handleInterrupt1()
{
  // Serial.println("1");
  encMotor1.encode();
}

void handleInterrupt2()
{
  // Serial.println("2");
  encMotor2.encode();
}

void handleInterrupt3()
{
  // Serial.println("3");
  encMotor3.encode();
}

void ISR_ENCA()
{
  pinA = digitalRead(channelA);
  pinB = digitalRead(channelB);

  if (pinA == LOW && pinB == LOW)
    enc_pulse--; // CCW
  if (pinA == LOW && pinB == HIGH)
    enc_pulse++; // CW
  if (pinA == HIGH && pinB == LOW)
    enc_pulse++; // CW
  if (pinA == HIGH && pinB == HIGH)
    enc_pulse--; // CCW
}
void ISR_ENCB()
{
  pinA = digitalRead(channelA);
  pinB = digitalRead(channelB);

  if (pinA == LOW && pinB == LOW)
    enc_pulse++; // CCW
  if (pinA == LOW && pinB == HIGH)
    enc_pulse--; // CW
  if (pinA == HIGH && pinB == LOW)
    enc_pulse--; // CW
  if (pinA == HIGH && pinB == HIGH)
    enc_pulse++; // CCW
}

void kiri()
{
// Serial.print(state);
//   Serial.print("\t");
//   Serial.print(pos);
//   Serial.print("\t");
//   Serial.print(currspd0);
//   Serial.print("\t");
//   Serial.print(currspd1);
//   Serial.print("\t");
//   Serial.print(currspd2);
//   Serial.print("\t");
//   Serial.print(currspd3);
//   Serial.print("\t|");
//   Serial.print(spd0);
//   Serial.print("\t");
//   Serial.print(spd1);
//   Serial.print("\t");
//   Serial.print(spd2);
//   Serial.print("\t");
//   Serial.print(spd3);
//   Serial.print("\t|");
//   Serial.print("pwm");
//   Serial.print("\t");
//   Serial.print(pwmMotor0);
//   Serial.print("\t");
//   Serial.print(pwmMotor1);
//   Serial.print("\t");
//   Serial.print(pwmMotor2);
//   Serial.print("\t");
//   Serial.print(pwmMotor3);
//   Serial.print("\t|");
//   Serial.print(distance_kanan);
//   Serial.print("\t");
//   Serial.print(distance_kiri);
//   Serial.print("\t");
//   Serial.print(distance_tengah);
//   Serial.println("---------");
// Serial.print(state);
//   Serial.print("\t");
//   Serial.print(pos);
//   Serial.print("\t");
//   Serial.print(currspd0);
//   Serial.print("\t");
//   Serial.print(currspd1);
//   Serial.print("\t");
//   Serial.print(currspd2);
//   Serial.print("\t");
//   Serial.print(currspd3);
//   Serial.print("\t|");
//   Serial.print(spd0);
//   Serial.print("\t");
//   Serial.print(spd1);
//   Serial.print("\t");
//   Serial.print(spd2);
//   Serial.print("\t");
//   Serial.print(spd3);
//   Serial.print("\t|");
//   Serial.print("pwm");
//   Serial.print("\t");
//   Serial.print(pwmMotor0);
//   Serial.print("\t");
//   Serial.print(pwmMotor1);
//   Serial.print("\t");
//   Serial.print(pwmMotor2);
//   Serial.print("\t");
//   Serial.print(pwmMotor3);
//   Serial.print("\t|");
//   Serial.print(distance_kanan);
//   Serial.print("\t");
//   Serial.print(distance_kiri);
//   Serial.print("\t");
//   Serial.print(distance_tengah);
//   Serial.println("---------");

  // distance_kanan, distance_kiri = ultrasonic();
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  // Serial.print("pos (kiri) =");
  // Serial.println(pos);

  if (pos <= 200 && pos >= -90)
  {

    spd0 = 25;
    spd1 = -40;
    spd2 = 25;
    spd3 = -40;

    // motor0.set_motor_speed((pwmMotor0));
    // motor1.set_motor_speed((pwmMotor1));
    // motor2.set_motor_speed((pwmMotor2));
    // motor3.set_motor_speed((pwmMotor3));

    motor0.set_motor_speed((180));
    motor1.set_motor_speed((-150));
    motor2.set_motor_speed((180));
    motor3.set_motor_speed((-200));

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

  // else if (pos < -20)
  // {
  //   spd0 = 5;
  //   spd1 = -50;
  //   spd2 = 5;
  //   spd3 = -50;

  //   // motor0.set_motor_speed((pwmMotor0));
  //   // motor1.set_motor_speed((-40));
  //   // motor2.set_motor_speed((pwmMotor2));
  //   // motor3.set_motor_speed((pwmMotor3));

  //   motor0.set_motor_speed((150));
  //   motor1.set_motor_speed((-150));
  //   motor2.set_motor_speed((150));
  //   motor3.set_motor_speed((-150));


  //   currspd0 = encMotor0.getPulses();
  //   currspd1 = encMotor1.getPulses();
  //   currspd2 = encMotor2.getPulses();
  //   currspd3 = encMotor3.getPulses();

  //   encMotor0.reset();
  //   encMotor1.reset();
  //   encMotor2.reset();
  //   encMotor3.reset();

  //   pwmMotor0 = pid1L[0].createpwm(spd0, currspd0);
  //   pwmMotor1 = pid1L[1].createpwm(spd1, currspd1);
  //   pwmMotor2 = pid1L[2].createpwm(spd2, currspd2);
  //   pwmMotor3 = pid1L[3].createpwm(spd3, currspd3);
  // }

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


void kiri_setengah()
{ 
  // distance_kanan, distance_kiri = ultrasonic();
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  // Serial.print("pos (kiri) =");
  // Serial.println(pos);
  // Serial.print("\t");
  // Serial.print(distance_kanan);
  // Serial.print("\t");
  // Serial.print(distance_kiri);
  // Serial.print("\t");
  // Serial.println(distance_tengah);

  // if (pos <= 200 && pos >= -90)
  // {

    if (distance_kanan > 15) 
    {
      spd3 -= 1;
    }
    else 
    {
      spd3 += 1;
    }
    if ( distance_kiri > 15) 
    {
      spd1 -= 2;
    }
    else 
    {
      spd1 += 2;
    }
  //   spd0 = 22;
  //   spd1 = -35;
  //   spd2 = 11;
  //   spd3 = -70;

    // motor0.set_motor_speed((pwmMotor0));
    // motor1.set_motor_speed((pwmMotor1));
    // motor2.set_motor_speed((pwmMotor2));
    // motor3.set_motor_speed((pwmMotor3));

      motor0.set_motor_speed((180));
    motor1.set_motor_speed((-220));
    motor2.set_motor_speed((160));
    motor3.set_motor_speed((-230));


  //   // motor0.set_motor_speed((180));
  //   // motor1.set_motor_speed((-150));
  //   // motor2.set_motor_speed((180));
  //   // motor3.set_motor_speed((-200));

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
  // // }
}

void kanan()
{
  // distance_kanan, distance_kiri = ultrasonic();
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();
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
  

  if (pos <= 100)
  {
    // Serial.println("aaaa");
    spd0 = -20;
    spd1 = 15;
    spd2 = -20;
    spd3 = 15;
  }
  else if (pos > 100)
  {
    // Serial.println("bbbb");
    spd0 = -50;
    spd1 = 5;
    spd2 = -50;
    spd3 = 5;
  }

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

void maju()
{ 
  // distance_kanan, distance_kiri = ultrasonic();
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri)
  {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    // kanan_tengah();
    if (state == "mau_kalkulasi")
    {
      state = "kalkulasi";
    }
    else
    {
      state = "ambil";
    }
    return;
  }

  else
  {
    if (distance_kanan > set_jarak_kanan && distance_tengah > set_jarak_tengah)
    {
      if (state == "mau_ambil" || state == "mau_kalkulasi")
      {
        spd0 = -48;
        spd1 = -48;
        spd2 = -50;
        spd3 = -50;
      }
      else
      {
        // spd0 = -40;
        // spd1 = -35;
        // spd2 = -40;
        // spd3 = -35;
        spd0 = -60;
        spd1 = 0;
        spd2 = -50;
        spd3 = 0;
      }
    }

    else if (distance_kanan < set_jarak_kanan && distance_kiri > set_jarak_kiri)
    {
      spd0 = -30;
      spd1 = -30;
      spd2 = 0;
      spd3 = 0;
    }

    else if (distance_kiri < set_jarak_kiri && distance_kanan > set_jarak_kanan)
    {
      spd0 = 0;
      spd1 = 0;
      spd2 = -30;
      spd3 = -30;
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
}


void dempetin()
{ 
  // distance_kanan, distance_kiri = ultrasonic();
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kanan <= set_jarak_kanan && distance_kiri <= set_jarak_kiri)
  {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    // kanan_tengah();
    if (state == "mau_kalkulasi")
    {
      state = "kalkulasi";
    }
    else
    {
      state = "ambil";
    }
    return;
  }
  else
  {

    if (distance_kanan > set_jarak_kanan && distance_kiri > set_jarak_kiri)
    {
      spd0 = -40;
      spd1 = -40;
      spd2 = -40;
      spd3 = -40;
    }

    else if (distance_kanan < set_jarak_kanan && distance_kiri > set_jarak_kiri)
    {
      spd0 = -25;
      spd1 = -25;
      spd2 = 0;
      spd3 = 0;
    }

    else if (distance_kiri < set_jarak_kiri && distance_kanan > set_jarak_kanan)
    {
      spd0 = 0;
      spd1 = 0;
      spd2 = -25;
      spd3 = -25;
    }

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

    motor0.set_motor_speed((pwmMotor0));
    motor1.set_motor_speed((pwmMotor1));
    motor2.set_motor_speed((pwmMotor2));
    motor3.set_motor_speed((pwmMotor3));
  }
}

void stop()
{
  spd0 = 0;
  spd1 = 0;
  spd2 = 0;
  spd3 = 0;
  motor0.set_motor_speed((0));
  motor1.set_motor_speed((0));
  motor2.set_motor_speed((0));
  motor3.set_motor_speed((0));
}

void maju_kanan()
{

  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_tengah <= set_jarak_kanan && distance_kiri <= set_jarak_kiri)
  {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    state = "ambil";
    count = 0;
    stop();
  }

  else
  {
    if (distance_tengah > set_jarak_kanan && distance_kiri > set_jarak_kiri)
    {
      spd0 = -50;
      spd1 = -50;
      spd2 = -50;
      spd3 = -50;
    }
    else if (distance_tengah < set_jarak_kanan && distance_kiri > set_jarak_kiri)
    {
      spd0 = -20;
      spd1 = -20;
      spd2 = 0;
      spd3 = 0;
    }
    else if (distance_kiri < set_jarak_kiri && distance_tengah > set_jarak_kanan)
    {
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

void maju_kiri()
{

  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  if (distance_kanan <= set_jarak_kanan && distance_tengah <= set_jarak_kiri)
  {
    motor0.set_motor_speed((0));
    motor1.set_motor_speed((0));
    motor2.set_motor_speed((0));
    motor3.set_motor_speed((0));
    stop();
    state = "ambil";
  }

  else
  {
    if (distance_kanan > set_jarak_kanan && distance_tengah > set_jarak_kiri)
    {
      spd0 = -50;
      spd1 = -50;
      spd2 = -50;
      spd3 = -50;
    }
    else if (distance_kanan < set_jarak_kanan && distance_tengah > set_jarak_kiri)
    {
      spd0 = -20;
      spd1 = -20;
      spd2 = 0;
      spd3 = 0;
    }
    else if (distance_tengah < set_jarak_kiri && distance_kanan > set_jarak_kanan)
    {
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

    pwmMotor0 = pid1L[0].createpwm(spd0, currspd0);
    pwmMotor1 = pid1L[1].createpwm(spd1, currspd1);
    pwmMotor2 = pid1L[2].createpwm(spd2, currspd2);
    pwmMotor3 = pid1L[3].createpwm(spd3, currspd3);

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

void setup()
{
  Serial.begin(500000);
  // Encoder motor
  attachPCINT(digitalPinToPCINT(63), handleInterrupt0, CHANGE);
  attachPCINT(digitalPinToPCINT(66), handleInterrupt1, CHANGE);
  attachPCINT(digitalPinToPCINT(64), handleInterrupt2, CHANGE);
  attachPCINT(digitalPinToPCINT(69), handleInterrupt3, CHANGE);
  // Extra Encoder
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
  pinMode(trigtengah, OUTPUT);
  pinMode(echotengah, INPUT);

  pinMode(pin_ir1, INPUT);
  pinMode(pin_ir2, INPUT);
  pinMode(pin_ir3, INPUT);
  pinMode(pin_ir4, INPUT);
  pinMode(pin_ir5, INPUT);

  pinMode(BUTTON_MAIN, INPUT);
  arm.servoL.attach(13);
  arm.servoR.attach(5);
  arm.servoL.write(90);
  arm.servoR.write(90);

  Serial.setTimeout(10);
  arm.kalibrasi();
  arm.kalibrasi();
  arm.selesai = true;
  arm.Lstate = 2;
  arm.Rstate = 2;
}

float ultrasonic()
{
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

  // Trigger ultrasonic sensor tengah
  digitalWrite(trigtengah, LOW);
  delayMicroseconds(2);
  digitalWrite(trigtengah, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigtengah, LOW);
  duration_tengah = pulseIn(echotengah, HIGH);
  distance_tengah = duration_tengah * 0.034 / 2;

  return distance_kanan, distance_kiri, distance_kiri;
}

// Function to reset the Arduino
void resetArduino()
{
  asm volatile("  jmp 0");
}

void loop()
{
  if (digitalRead(BUTTON_MAIN) == LOW)
  {
    if (nyala == 1)
    {
      resetArduino();
      nyala = 0;
      stop();
    }
    stop();
    return;
  }
  

  start_time = millis();
  nyala = 1;
  // Position from Extra Encoder
  pos = ((float)enc_pulse * 0.000125) * 6.2832 * r_wheel;
  // Serial.print(pos);
  // Serial.print("\t|");
  // Serial.print(checkpoint);
  // Serial.print("\t|");
  // // Serial.print("Q ");
  distance_kanan, distance_kiri, distance_tengah = ultrasonic();

  // Serial.print(distance_kanan);
  // Serial.print("\t|");
  // Serial.println(distance_kiri);

  // Serial.print(enc_pulse);

  // state = "kanan";
  while (state == "ambil")
  {

    if (digitalRead(BUTTON_MAIN) == LOW)
    {
      if (nyala == 1)
      {
        resetArduino();
        nyala = 0;
        stop();
      }
      stop();
      continue;
    }

    nyala = 1;
    check_time = millis();
    stop();
    // cek waktu
    // Serial.println(check_time - start_time);
    // Read and parse incoming data

    arm.move(arm1.type, arm2.type);
    if (check_time - start_time >= 30000)
    {
      if (checkpoint == 0)
      {
        spd0 = 22;
        spd1 = -35;
        spd2 = 11;
        spd3 = -70;
        state = "kanan";
        // Serial.println("Chechkpoint 0 selesai");
        arm.stepper.setMaxSpeed(800);
        arm.stepper.setAcceleration(800);
        arm.stepper2.setMaxSpeed(200);
        arm.stepper2.setAcceleration(200);
        arm.stepperR.setMaxSpeed(800);
        arm.stepperR.setAcceleration(800);
        arm.stepper2R.setMaxSpeed(200);
        arm.stepper2R.setAcceleration(200);
        arm.kalibrasi();
        checkpoint = 1;
        break;
      }
      else if (checkpoint == 1)
      {
        Serial.println("tes1");
        state = "kalkulasi";
        // Serial.println("Chechkpoint 1 selesai");
        arm.stepper.setMaxSpeed(800);
        arm.stepper.setAcceleration(800);
        arm.stepper2.setMaxSpeed(200);
        arm.stepper2.setAcceleration(200);
        arm.stepperR.setMaxSpeed(800);
        arm.stepperR.setAcceleration(800);
        arm.stepper2R.setMaxSpeed(200);
        arm.stepper2R.setAcceleration(200);
        arm.kalibrasi();
        checkpoint = 1;
        break;
        Serial.println("tes2");
      }
      else if (checkpoint == 2)
      {
        state = "kanan_tengah";
        // Serial.println("Chechkpoint 2 selesai");
        arm.kalibrasi();
        break;
      }
    }
    else if (Serial.available() > 0 && arm.selesai)
    { // arm.selesai dihapus sementara
      // Serial.println("state AMBIL");
      chr = Serial.parseInt();
      if (chr == -999)
      {
        parse_data();
      }
    }
  }

  if (state == "mulai")
  {
    maju();
  }
  // move arm
  else if (state == "kanan")
  { // jalan ke dari kanan ke tengah
    count = 0;
    // kanan();
    kiri_setengah();
    int ir1 = digitalRead(pin_ir1);
    int ir2 = digitalRead(pin_ir2);
    int ir3 = digitalRead(pin_ir3);
    int ir4 = digitalRead(pin_ir4);
    int ir5 = digitalRead(pin_ir5);
    // state = "kalkulasi";
    if (pos<=70 && (ir4 == 0 || ir5 == 0))
    // if (ir3 == 0)
    {
      // state = "kalkulasi";
      count = 0;
      // state = "mau_ambil";
      checkpoint = 1;
      while (!(distance_kiri <= set_jarak_kiri && distance_kanan <= set_jarak_kanan)) {
        dempetin();
      // maju();
      }
      state = "ambil";
    }
    
    // if (pos >= 120)
    // {
    //   state = "maju_kanan";
    //   checkpoint = 1;
    // }
  }

  else if (state == "kiri")
  { // jalan ke kiri
    count = 0;
    kiri_setengah();
    if (pos <= -95)
    {
      // state = "mau_ambil";
      checkpoint = 2;
      while (!(distance_kiri <= set_jarak_kiri && distance_kanan <= set_jarak_kanan)) {
        dempetin();

      // maju();
      }
      state = "ambil";
      // maju();
    }
  }

  // }
  else if (state == "kanan_tengah")
  { // jalan ke tengah dari kiri ---- KANAN_TENGAH
    // checkpoint = 23;
    // kanan_tengah();
    count = 0;
    int ir1 = digitalRead(pin_ir1);
    int ir2 = digitalRead(pin_ir2);
    int ir3 = digitalRead(pin_ir3);
    int ir4 = digitalRead(pin_ir4);
    int ir5 = digitalRead(pin_ir5);
    // state = "kalkulasi";
    if (pos >= -50 && (ir1 == 0 || ir2 == 0))
    {
      state = "mau_kalkulasi";
      while (!(distance_kiri <= set_jarak_kiri && distance_kanan <= set_jarak_kanan)) {
        dempetin();
      // checkpoint = 1;
      // maju();
      }
      // delay(2000);
      state = "kalkulasi";
      // maju();
    }
    kanan();
  }

  else if (state == "kalkulasi")
  { // kalkulasi posisi tong sampah
    stop();
    // Serial.print("Q ");
    // Serial.println("buang");

    arm.Lstate = 4;
    arm.Rstate = 4;
    arm.calculateBuang(16, 56, -18, 56);
    state = "buang";
  }

  else if (state == "buang")
  { // buang sampah
    while (arm.Rstate != 5 || arm.Lstate != 5){
      arm.buang();
    }
    count = 0;
    if (arm.Rstate == 5 && arm.Lstate == 5 && checkpoint == 1) {
      arm.kalibrasi();
      state = "kiri";
      arm.Lstate = 2;
      arm.Rstate = 2;
    }
  }

  else if (state == "maju_kanan")
  { // jalan ke kiri
    maju();
    count = 0;
    state = "ambil";
  }

  else if (state == "maju_kiri")
  { // jalan ke kiri
    maju();
    count = 0;
    state = "ambil";
  }
}

/*========== FUNCTIONS DEFINITIONS ==========*/
void parse_data()
{
  if (Serial.available() > 0 && arm.selesai)
  { // arm.selesai dihapus sementara

    spd0 = Serial.parseInt();
    spd1 = Serial.parseInt();
    spd2 = Serial.parseInt();
    spd3 = Serial.parseInt();
    arm1.posX = Serial.parseFloat();
    arm1.posY = Serial.parseFloat();
    arm1.type = Serial.parseInt();
    arm2.posX = Serial.parseFloat();
    arm2.posY = Serial.parseFloat();
    arm2.type = Serial.parseInt();
    float skip = Serial.parseFloat();

    // Serial.print("S"); Serial.print(", ");
    // Serial.print(arm1.posX); Serial.print(","); Serial.print(arm1.posY); Serial.print(",");
    // Serial.print(arm2.posX); Serial.print(","); Serial.println(arm2.posY);
    // }
// Jika menggunakan komvis buat nyari tong, uncomment
    // if (state == "kalkulasi")
    // {
    //   // arm.calculateBuang(arm1.posX, arm1.posY, arm2.posX, arm2.posY);
    //   arm.calculateBuang(23.84, 50, -9.15, 59.97);
    //   state = "buang";
    }
    // if (arm1.type == 6 && arm2.type == 6) {
    //   // arm.calculateBuang(arm1.posX, arm1.posY, arm2.posX, arm2.posY);
    //   arm.calculateBuang(23.84, 50, -9.15, 59.97);
    //   state = 8;
    // }

    if (arm2.type != 0 && arm2.type != 6)
    {
      if (arm2.posX != 0 || arm2.posY != 0)
      {
        arm.calculateR(arm2.posX, arm2.posY);
      }
    }
    else if (arm1.type != 0 && arm1.type != 6)
    {
      if (arm1.posX != 0 || arm1.posY != 0)
      {
        arm.calculate(arm1.posX, arm1.posY);
      }
    }

    else if (arm1.type <= 0 && arm2.type <= 0)
    {
      count += 1;
      // Serial.println(count);
      if (count >= 10)
      {
        if (checkpoint == 0 && arm.Lstate == 2 && arm.Rstate == 2)
        {
          spd0 = 22;
          spd1 = -35;
          spd2 = 11;
          spd3 = -70;
          state = "kanan";
          // Serial.println("Chechkpoint 0 selesai");
                  arm.stepper.setMaxSpeed(800);
        arm.stepper.setAcceleration(800);
        arm.stepper2.setMaxSpeed(200);
        arm.stepper2.setAcceleration(200);
        arm.stepperR.setMaxSpeed(800);
        arm.stepperR.setAcceleration(800);
        arm.stepper2R.setMaxSpeed(200);
        arm.stepper2R.setAcceleration(200);
          arm.kalibrasi();
          count = 0;
        }
        else if (checkpoint == 1 && arm.Lstate == 2 && arm.Rstate == 2)
        {
          state = "kalkulasi";
          // Serial.println("Chechkpoint 1 selesai");
                  arm.stepper.setMaxSpeed(800);
        arm.stepper.setAcceleration(800);
        arm.stepper2.setMaxSpeed(200);
        arm.stepper2.setAcceleration(200);
        arm.stepperR.setMaxSpeed(800);
        arm.stepperR.setAcceleration(800);
        arm.stepper2R.setMaxSpeed(200);
        arm.stepper2R.setAcceleration(200);
          arm.kalibrasi();
          count = 0;
        }
        else if (checkpoint == 2 && arm.Lstate == 2 && arm.Rstate == 2)
        {
          state = "kanan_tengah";
          // Serial.println("Chechkpoint 2 selesai");
                  arm.stepper.setMaxSpeed(800);
        arm.stepper.setAcceleration(800);
        arm.stepper2.setMaxSpeed(200);
        arm.stepper2.setAcceleration(200);
        arm.stepperR.setMaxSpeed(800);
        arm.stepperR.setAcceleration(800);
        arm.stepper2R.setMaxSpeed(200);
        arm.stepper2R.setAcceleration(200);
          arm.kalibrasi();
          count = 0;
        }
        count = 0;
      }
    }
    arm.selesai = false;
}

