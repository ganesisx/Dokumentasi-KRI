

      /*************          ******************
       *  L2      *        *    R2      *
      **************        ******************
     *    L1    *          *    R1    *
    **************          ******************
    *  UP    ******************    TRIANGLE    *
    *LEFT RIGHT* SELECT START * SQUARE  CIRCLE *
    * DOWN   ******************     CROSS    *
    ************** L3 *    * R3 ******************
           ******    *****/ 

// Urutan 16 Bit
    /***************************************************************************************************************
    * UP * DOWN * LEFT * RIGHT * R1 * L1 * START * SELECT * R3 * L3 * R2 * L2 * SQUARE * CIRCLE * TRIANGLE * CROSS *
    ****************************************************************************************************************
bit ke-   15  14     13     12     11   10    09       08     07   06   05   04     03       02        01        00 */

// Add library PS3BT.h dan usbhub.h pada Arduino IDE

#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

void setup()
{
  Serial.begin(9600);
#if !defined(__MIPSEL__) 
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  //Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

unsigned long prevTime, countTime;
// MSB merupakan 8 bit paling depan yaitu bit ke-15 hingga bit ke-8
// LSB merupakan 8 bit paling belakang yaitu bit ke-7 hingga bit ke-0
uint8_t MSB=0, LSB=0, analog_LeftX=127, analog_LeftY=127, analog_RightX, analog_RightY, cond;
char test[1];
int mode;
void loop()
{ 
  Usb.Task();
  //Debounce
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (millis()-prevTime >= 25){
      prevTime = millis();
      if (PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
        analog_LeftY = PS3.getAnalogHat(LeftHatY);
        analog_RightY = PS3.getAnalogHat(RightHatY);
      }
      if (PS3.getButtonPress(UP)) {
        MSB |= 128;
      }
      else if (!PS3.getButtonPress(UP)){
        MSB &= 127;
      }
      if (PS3.getButtonPress(DOWN)) {
        MSB |= 64;
      }
      else if (!PS3.getButtonPress(DOWN)){
        MSB &= 191;
      }
      if (PS3.getButtonPress(LEFT)) {
        MSB |= 32;
      }
      else if (!PS3.getButtonPress(LEFT)) {
        MSB &= 223;
      }
      if (PS3.getButtonPress(RIGHT)) {
        MSB |= 16;
      }
      else if (!PS3.getButtonPress(RIGHT)) {
        MSB &= 239;
      } 
      if (PS3.getButtonPress(R1)) {
        MSB |= 8;
      }
      else if (!PS3.getButtonPress(R1)) {
        MSB &= 247;
      }
      if (PS3.getButtonPress(L1)) {
        MSB |= 4;
      }
      else if (!PS3.getButtonPress(L1)) {
        MSB &= 251;
      }
      if (PS3.getButtonPress(START)) {
        if (millis()-countTime >= 250){
          countTime = millis();
          MSB |= 2;
        }
      }
      else if (!PS3.getButtonPress(START)) {
        MSB &= 253;
      }
      if (PS3.getButtonPress(SELECT)) {
          MSB |= 1;
//          PS3.setLedOff();
//          PS3.setLedOn(LED2);
      }
      else if (!PS3.getButtonPress(SELECT)) {
        MSB &= 254;
      }
      if (PS3.getButtonPress(R3)) {
        LSB |= 128;
      }
      else if (!PS3.getButtonPress(R3)) {
        LSB &= 127;
      }
      if (PS3.getButtonPress(L3)) {
        if (millis()-countTime >= 250){
          countTime = millis();
          LSB |= 64;
        }
      }
      else if (!PS3.getButtonPress(L3)) {
        LSB &= 191;
      }
      if (PS3.getButtonPress(R2)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 32;
        }
      }
      else if (!PS3.getButtonPress(R2)) {
        LSB &= 223;
      }
      if (PS3.getButtonPress(L2)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 16;
        }
      }
      else if (!PS3.getButtonPress(L2)) {
        LSB &= 239;
      }
      if (PS3.getButtonPress(SQUARE)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 8;
          }
      }
      else if (!PS3.getButtonPress(SQUARE)) {
        LSB &= 247;
      }
      if (PS3.getButtonPress(CIRCLE)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 4;
          }
      }  
      else if (!PS3.getButtonPress(CIRCLE)) {
        LSB &= 251;
      }
      if (PS3.getButtonPress(TRIANGLE)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 2;
          }
      }
      else if (!PS3.getButtonPress(TRIANGLE)) {
        LSB &= 253;
      }
      if (PS3.getButtonPress(CROSS)) {
        if (millis()-countTime >= 200){
          countTime = millis();
          LSB |= 1;
          }
      }
      else if (!PS3.getButtonPress(CROSS)) {
        LSB &= 254;
      }
      //UART
      Serial.write(MSB);
      Serial.write(LSB);
      Serial.write(analog_RightY);
      Serial.write(analog_LeftY);
    }
  }
  analog_RightY = 127;
  analog_LeftY = 127;
}
