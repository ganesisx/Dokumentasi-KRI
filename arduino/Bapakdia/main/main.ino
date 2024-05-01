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


//======== OBject Declarations ========//
// Motor Driver 2 pin
Driver motor0(6,7);
Driver motor1(8,9);
Driver motor2(10,11);
Driver motor3(12,13);
Arm arm(48, 46, 29, 31, 52, 50, 33, 35);


// Encoder
#define NO_ENCODER

// arm

//======== Variables ========//
// Arm structs
;struct {
  int type;   // Trash type
  float posX; // X coordinate target
  float posY; // Y coordinate target
} arm1, arm2;

// Motor speed setpoint
int spd0, spd1, spd2, spd3;

// Trash 

//======== Serial Setup Between Microcontroller to SBC Using USB ========//
/* SERIAL SETUP FOR INCOMING DATA
v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_type arm2_posX arm2_posY arm2_type
character v is flag for the right command from sbc to arduino
NOTE: space is the delimiter
*/
const int data_length = 8;  // Variable to hold each serial data length
char data[data_length];     // Variable to hold arguments
String chr;                   // Variable to hold an input character
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

  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels

  // Declaration for SSD1306 display connected using I2C
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  void new_display(String text, int x, int y) {
    display.clearDisplay();
    display.setCursor(x,y);
    display.println(text);
    display.display();
    display.clearDisplay();
  }
#endif


void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  Serial.begin(500000);
  while(!Serial) {} // Wait until serial connection is set

  #ifdef DISPLAY
    #ifdef DEBUG
      Serial.println("Testing with both Serial Monitor and OLED Screen");
    #endif
    // initialize the OLED object
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    // Clear the buffer.
    display.clearDisplay();

    // Display Text
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,28);
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
  parse_data();


  // Move motor
  #ifdef NO_ENCODER
    motor0.set_motor_speed(spd0);
    motor1.set_motor_speed(spd1);
    motor2.set_motor_speed(spd2);
    motor3.set_motor_speed(spd3);
  #else
    currspd0 = encMotor0.getPulses()/2;
    currspd1 = encMotor1.getPulses()/2;
    currspd2 = encMotor2.getPulses()/2;
    currspd3 = encMotor3.getPulses()/2;

    encMotor0.reset();
    encMotor1.reset();
    encMotor2.reset();
    encMotor3.reset();

    pwmMotor0 = pid1m[0].createpwm(spd0,currspd0);
    pwmMotor1 = pid1m[1].createpwm(spd1,currspd1);
    pwmMotor2 = pid1m[2].createpwm(spd2,currspd2);
    pwmMotor3 = pid1m[3].createpwm(spd3,currspd3);

    motor0.set_motor_speed(pwmMotor0);
    motor1.set_motor_speed(pwmMotor1);
    motor2.set_motor_speed(pwmMotor2);
    motor3.set_motor_speed(pwmMotor3);
  #endif


  // Move Arm
  arm.move();
  arm.moveR();
  // Send data to sbc
}

/*========== FUNCTIONS DEFINITIONS ==========*/
void parse_data() {
  if (Serial.available()>0 && arm.done()){
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
  chr = Serial.readString(); spd0 = Serial.parseInt(); spd1 = Serial.parseInt(); spd2 = Serial.parseInt(); spd3 = Serial.parseInt();
  arm1.posX = Serial.parseFloat(); arm1.posY = Serial.parseFloat(); arm1.type = Serial.parseInt();
  arm2.posX = Serial.parseFloat(); arm2.posY = Serial.parseFloat(); arm2.type = Serial.parseInt();
  float skip = Serial.parseFloat();
  // }
  if(arm1.posX != 0 && arm1.posY != 0 && arm2.posX != 0 && arm2.posY != 0) {
    Serial.print("POS ");
    arm.calculate(arm1.posX, arm1.posY);
    // Serial.print(arm1.posX);
    // Serial.print(",");
    // Serial.print(arm1.posY);
    // Serial.print(",");
    // // arm.calculateR(arm2.posX, arm2.posY);
    // Serial.print(arm2.posX);
    // Serial.print(",");
    // Serial.println(arm2.posY);
    }
  if (arm.done()){
     Serial.println("Char Data: " + String(chr));
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

#if defined(DEBUG) || defined (DISPLAY)
  void print_commands() {
    #ifdef DEBUG
      Serial.println("This goes");
      char data[8];
      String outString = "PWM_0 = " + String(spd0) +"\n";
      outString += "PWM_1 = " + String(spd1) +"\n";
      outString += "PWM_2 = " + String(spd2) +"\n";
      outString += "PWM_3 = " + String(spd3) +"\n";

      outString += "arm1_posX = " + String(m_arm1.posX) +"\n";
      outString += "arm1_posY = " + String(m_arm1.posY) +"\n";
      outString += "arm1_grip = " + String(m_arm1.grip) +"\n";

      outString += "arm2_posX = " + String(m_arm2.posX) +"\n";
      outString += "arm2_posY = " + String(m_arm2.posY) +"\n";
      outString += "arm2_grip = " + String(m_arm2.grip) +"\n";

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
