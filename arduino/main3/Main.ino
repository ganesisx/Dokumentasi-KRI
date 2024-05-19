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
v 125 -125 125 -125 321.12345 -321.12345 1 321.12345 -321.12345 1

***************************************************************************************/
#include "Driver.h"
#include "Encoder.h"
#include "Pid.h"
// #include "stepper.h"

//======== OBject Declarations ========//
// Motor Driver 2 pin
Driver motor0(6, 7);
Driver motor1(8, 9);
Driver motor2(10, 11);
Driver motor3(12, 13);

// Encoder
#define ENCODER
Encoder encMotor0(63, 62, 330);
Encoder encMotor1(66, 67, 330);
Encoder encMotor2(64, 65, 330);
Encoder encMotor3(69, 68, 330);
// Encoder addition to know position
float pos = 0, enc_pulse = 0;
const int channelA = 2, channelB=3;
int pinA, pinB; //Condition of channelA and channel B

// Encoder encMotor0(63, 64, 330);
// Encoder encMotor1(65, 66, 330);
// Encoder encMotor2(67, 68, 330);
// Encoder encMotor3(69, 70, 330);

//Kiri
// kp, ki, dan kd
double kp1m[] = {
  0.5,  // Motor 0
  0.7,  // Motor 1
  0.4,  // Motor 2
  0.7  // Motor 3
};

double ki1m[] = {
  0.3,   // Motor 0
  0.6,   // Motor 1
  0.3,   // Motor 2
  0.2  // Motor 3
};
double kd[] = {
  0.0,   // Motor 0
  0.0,   // Motor 1
  0.0,   // Motor 2
  0.0     // Motor 3
};



// PID
PID pid1m[] = {
  //PID maju
  PID(kp1m[0], ki1m[0], kd[0], 1000),  // Motor 0
  PID(kp1m[1], ki1m[1], kd[1], 1000),  // Motor 1
  PID(kp1m[2], ki1m[2], kd[2], 1000),  // Motor 2
  PID(kp1m[3], ki1m[3], kd[3], 1000)   // Motor 3
};



// Stepper

//======== Variables ========//
// Arm structs
struct {
  int type;    // Trash type
  float posX;  // X coordinate target
  float posY;  // Y coordinate target
} arm1, arm2;

// Motor speed setpoint
float spd0, spd1, spd2, spd3;

// Current Motor Speed
float currspd0, currspd1, currspd2, currspd3;
// PWM
double pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3;

// Trash

//======== Serial Setup Between Microcontroller to SBC Using USB ========//
/* SERIAL SETUP FOR INCOMING DATA
v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_type arm2_posX arm2_posY arm2_type
character v is flag for the right command from sbc to arduino
NOTE: space is the delimiter
*/
const int data_length = 8;  // Variable to hold each serial data length
char data[data_length];     // Variable to hold arguments
char chr;                   // Variable to hold an input character
int state = -1;             // Variable to determine which parmeter is being read
short idx = 0;              // Variable of data array index
void parse_data();          // Parsing incoming data
void input_commands();      // Insert value to desired variable after parsing serial input
void clear_commands();      // Clear the current command parameters

/* SERIAL


//======== Debugging ========//
/* !!NOTE: Use this if you're not running the python scripts
Debugging Read Value using Serial Monitor
Comment the define and uncomment the undef if not used and vice versa
*/
// #define DEBUG
#undef DEBUG

#if defined(DEBUG) || defined(DISPLAY)
void print_commands();  // Function to print input values
#endif

/*
Debugging Read Value using OLED 128 x 64
Comment the define and uncomment the undef if not used and vice versa
*/
// #define DISPLAY
#undef DISPLAY

#ifdef DISPLAY
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void new_display(String text, int x, int y) {
  display.clearDisplay();
  display.setCursor(x, y);
  display.println(text);
  display.display();
  display.clearDisplay();
}
#endif


void setup() {
  Serial.begin(115200);
  attachPCINT(digitalPinToPCINT(63), handleInterrupt0, CHANGE);
  attachPCINT(digitalPinToPCINT(66), handleInterrupt1, CHANGE);
  attachPCINT(digitalPinToPCINT(64), handleInterrupt2, CHANGE);
  attachPCINT(digitalPinToPCINT(69), handleInterrupt3, CHANGE);
  //Extra Encoder
  attachInterrupt(digitalPinToInterrupt(channelA), ISR_ENCA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB), ISR_ENCB, CHANGE);
  delay(5000);
  while (!Serial) {}  // Wait until serial connection is set

#ifdef DISPLAY
#ifdef DEBUG
  Serial.println("Testing with both Serial Monitor and OLED Screen");
#endif
  // initialize the OLED object
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Clear the buffer.
  display.clearDisplay();

  // Display Text
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 28);
  display.println("Waiting...");
  display.display();
  display.clearDisplay();
#endif

#ifdef DEBUG
  Serial.println("Testing Values Using Serial Montior");
#endif
}

void loop() {
  // Read and parse incoming data
  // parse_data();
  spd0 = 2;
  spd1 = -5;
  spd2 = 2;
  spd3 = -4;

// Move motor
#ifdef NO_ENCODER
  motor0.set_motor_speed(spd0);
  motor1.set_motor_speed(spd1);
  motor2.set_motor_speed(spd2);
  motor3.set_motor_speed(spd3);
#endif
  // currspd0 = fabs(encMotor0.getPulses());
  // currspd1 = fabs(encMotor1.getPulses());
  // currspd2 = fabs(encMotor2.getPulses());
  // currspd3 = fabs(encMotor3.getPulses());
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

  Serial.print(currspd0);
  Serial.print(" ");
  Serial.print(currspd1);
  Serial.print(" ");
  Serial.print(currspd2);
  Serial.print(" ");
  Serial.print(currspd3);
  Serial.print(" ");
  Serial.print("pwm");
  Serial.print(" ");
  Serial.print(pwmMotor0);
  Serial.print(" ");
  Serial.print(pwmMotor1);
  Serial.print(" ");
  Serial.print(pwmMotor2);
  Serial.print(" ");
  Serial.print(pwmMotor3);
  Serial.println("---------");

  // int chanA = digitalRead(69);
  // int chanB = digitalRead(68);
  // Serial.println(chanA);
  // Serial.println("A/B");
  // Serial.println(chanB);

  motor0.set_motor_speed((pwmMotor0));
  motor1.set_motor_speed((pwmMotor1));
  motor2.set_motor_speed((pwmMotor2));
  motor3.set_motor_speed((pwmMotor3));

  // motor0.set_motor_speed((0));
  // motor1.set_motor_speed((0));
  // motor2.set_motor_speed((0));
  // motor3.set_motor_speed((0));
  // Move Arm

  // Send data to sbc
}

/*========== FUNCTIONS DEFINITIONS ==========*/
void parse_data() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

#ifdef DEBUG
    Serial.println("Char Data: " + String(chr));
    Serial.println(chr, DEC);
#endif

    if (state == -1) {
      if (chr == 'v') state = 0;
      else {
#ifdef DEBUG
        Serial.println("Invalid Commands!");
#endif
        continue;
      }
    } else if (chr == 10 | chr == 13) {  // newline
      data[idx] = NULL;
      input_commands();
      clear_commands();

      state = -1;
      idx = 0;
#if defined(DEBUG) || defined(DISPLAY)
      print_commands();
#endif
    } else if (chr == 32) {  // Detect space
      data[idx] = NULL;
      input_commands();
      clear_commands();
      state++;
      idx = 0;
    } else {
      data[idx] = chr;
      idx++;
    }
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

}
void ISR_ENCB() {

}

#if defined(DEBUG) || defined(DISPLAY)
void print_commands() {
#ifdef DEBUG
  Serial.println("This goes");
  char data[8];
  String outString = "PWM_0 = " + String(spd0) + "\n";
  outString += "PWM_1 = " + String(spd1) + "\n";
  outString += "PWM_2 = " + String(spd2) + "\n";
  outString += "PWM_3 = " + String(spd3) + "\n";

  outString += "arm1_posX = " + String(m_arm1.posX) + "\n";
  outString += "arm1_posY = " + String(m_arm1.posY) + "\n";
  outString += "arm1_grip = " + String(m_arm1.grip) + "\n";

  outString += "arm2_posX = " + String(m_arm2.posX) + "\n";
  outString += "arm2_posY = " + String(m_arm2.posY) + "\n";
  outString += "arm2_grip = " + String(m_arm2.grip) + "\n";

  Serial.println(outString);
#endif

#ifdef DISPLAY
  // Uncomment the parameter needs to be tested.
  // If more than one, add more new_display function with different cursor
  // and add more output string variable


  String outDisp = "PWM_0 = " + String(spd0);
  String outDisp = "PWM_0 = " + String(spd0);
  String outDisp = "PWM_0 = " + String(spd0);
  String outDisp = "PWM_0 = " + String(spd0);

  String outDisp = "arm1_posX = " + String(m_arm1.posX);
  String outDisp = "arm1_posY = " + String(m_arm1.posY);
  String outDisp = "arm1_grip = " + String(m_arm1.grip);

  String outDisp = "arm2_posX = " + String(m_arm2.posX);
  String outDisp = "arm2_posY = " + String(m_arm2.posY);
  String outDisp = "arm2_grip = " + String(m_arm2.grip);

  new_display(outDisp, 0, 1);
#endif
}
#endif
