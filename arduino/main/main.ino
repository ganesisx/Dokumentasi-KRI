// Don't forget to put the lib files (both cpp and h) in the main folder

/***************************************************************************************
main function to parse data from serial and control the actuator. It also takes data f
rom sensors and send it to python script through serial.

Serial Setup:
    v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_grip arm2_posX arm2_posY arm2_grip sweep_posX sweep_posY sweep_grip
Character v is flag for the right command

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

/* 
Debugging Read Value using OLED 128 x 64
Comment the define and uncomment the undef if not used and vice versa
*/
#define DISPLAY
// #undef DISPLAY

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

/* 
Debugging Read Value using Serial Monitor
Comment the define and uncomment the undef if not used and vice versa
NOTE: Use this when you're not running the python scripts
*/
// #define DEBUG
#undef DEBUG

#if defined(DEBUG) || defined(DISPLAY)
  void print_commands();  // Function to print input values
#endif

struct {
  int grip;
  float posX;
  float posY;
} m_arm1, m_arm2, sweep_arm;

int spd0, spd1, spd2, spd3;


/* SERIAL SETUP
v pwm0 pwm1 pwm2 pwm3 arm1_posX arm1_posY arm1_grip arm2_posX arm2_posY arm2_grip sweep_posX sweep_posY sweep_grip
character v is flag for the right command
NOTE: space is the delimiter
*/
const int data_length = 8;  // Variable to hold each serial data length
char data[data_length];     // Variable to hold arguments
char chr;                   // Variable to hold an input character
int state = -1;             // Variable to determine which parmeter is being read
short idx = 0;              // Variable of data array index
void input_commands();      // Insert value to desired variable after parsing serial input
void clear_commands();      // Clear the current command parameters


void setup() {
  Serial.begin(115200);
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

  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    #ifdef DEBUG
      Serial.println("Char Data: " + String(chr));
      Serial.println(chr,DEC);
    #endif

    if (state == -1) {
      if (chr == 'v') state = 0;
        else {
        #ifdef DEBUG
          Serial.println("Invalid Commands!");
        #endif 
        continue;
      }
    }
    else if (chr == 10 | chr == 13) { // newline
      data[idx] = NULL;
      input_commands();
      clear_commands();

      state = -1;
      idx = 0;
      #if defined(DEBUG) || defined(DISPLAY)
        print_commands();
      #endif
    }
    else if (chr == 32) {
      data[idx] = NULL;
      input_commands();
      clear_commands();
      state++;
      idx = 0;
    }
    else {
      data[idx] = chr;
      idx++;
    }
  }
  
}

/*========== FUNCTIONS DEFINITIONS ==========*/
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
      m_arm1.posX = atof(data);
      break;
    case 6:
      m_arm1.posY = atof(data);
      break;
    case 7:
      m_arm1.grip = atoi(data);
      break;

    /* ============= ARM2 ============= */
    case 8:
      m_arm2.posX = atof(data);
      break;
    case 9:
      m_arm2.posY = atof(data);
      break;
    case 10:
      m_arm2.grip = atoi(data);
      break;
    /* ============= SWEEP ARM ============= */
    case 11:
      sweep_arm.posX = atof(data);
      break;
    case 12:
      sweep_arm.posY = atof(data);
      break;
    case 13:
      sweep_arm.grip = atoi(data);
      break;

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

      // outString += "arm2_posX = " + String(m_arm2.posX) +"\n";
      // outString += "arm2_posY = " + String(m_arm2.posY) +"\n";
      // outString += "arm2_grip = " + String(m_arm2.grip) +"\n";

      Serial.println(outString);
    #endif

    #ifdef DISPLAY 
      // Uncomment the parameter needs to be tested. 
      // If more than one, add more new_display function with different cursor
      // and add more output string variable


      // String outDisp = "PWM_0 = " + String(spd0);
      // String outDisp = "PWM_0 = " + String(spd0);
      // String outDisp = "PWM_0 = " + String(spd0);
      // String outDisp = "PWM_0 = " + String(spd0);

      // String outDisp = "arm1_posX = " + String(m_arm1.posX);
      // String outDisp = "arm1_posY = " + String(m_arm1.posY);
      // String outDisp = "arm1_grip = " + String(m_arm1.grip);

      // String outDisp = "arm2_posX = " + String(m_arm2.posX);
      // String outDisp = "arm2_posY = " + String(m_arm2.posY);
      // String outDisp = "arm2_grip = " + String(m_arm2.grip);

      // String outDisp = "s_arm_posX = " + String(sweep_arm.posX);
      // String outDisp = "s_arm_posY = " + String(sweep_arm.posY);
      String outDisp = "s_arm_grip = " + String(sweep_arm.grip);

      new_display(outDisp, 0, 1);
    #endif
  }
#endif