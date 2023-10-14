      /*************          ******************
       *  L2      *        *    R2      *
      **************        ******************
     *    L1    *          *    R1    *
    **************          ******************
    *  UP    ******************    TRIANGLE    *
    LEFT RIGHT SELECT START * SQUARE  CIR6CLE *
    * DOWN   ******************     CROSS    *
    ************** L3 *    * R3 ******************
           ******    *****/ 

// Urutan 16 Bit
    /***************************************************************************************************************
    * UP * DOWN * LEFT * RIGHT * R1 * L1 * PSButton * SELECT * R3 * L3 * R2 * L2 * SQUARE * CIRCLE * TRIANGLE * CROSS *
    ****************************************************************************************************************
bit ke-   15  14     13     12     11   10    09       08     07   06   05   04     03       02        01        00 */

#include <PS4Controller.h>
#define DEBUG 0

unsigned long prevTime, countTime;
// MSB merupakan 8 bit paling depan yaitu bit ke-15 hingga bit ke-8
// LSB merupakan 8 bit paling belakang yaitu bit ke-7 hingga bit ke-0
uint8_t MSB=0, LSB=0;
int LeftX, LeftY, RightX, RightY;


void choice() {
  /*if (PS4.LStickX() >= 10 || PS4.LStickX() <= -10) {LeftX = PS4.LStickX();}
  if (PS4.LStickY() >= 10 || PS4.LStickY() <= -10) {LeftY = PS4.LStickY();}
  
  if (PS4.RStickX() >= 10 || PS4.RStickX() <= -10) {RightX = PS4.RStickX();}
  if (PS4.RStickY() >= 10 || PS4.RStickY() <= -10) {RightY = PS4.RStickY();}*/

  LeftX = PS4.LStickX()+128; LeftY = PS4.LStickY()+128;
  RightX = PS4.RStickX()+128; RightY = PS4.RStickY()+128;

  if (PS4.Up()) {
    MSB |= 128;
  } else if (!PS4.Up()) {
    MSB &= 127;
  }

  if (PS4.Down()) {
    MSB |=64;
  } else if (!PS4.Down()) {
    MSB &= 191;
  }

  if (PS4.Left()) {
    MSB |= 32;
  } else if (!PS4.Left()) {
    MSB &= 223;
  }

  if (PS4.Right()) {
    MSB |= 16;
  } else if (!PS4.Right()){
    MSB &= 239;
  }

  if (PS4.R1()) {
    MSB |= 8;
  } else if (!PS4.R1()) {
    MSB &= 247;
  }

  if (PS4.L1()) {
    MSB |= 4;
  } else if (!PS4.L1()) {
    MSB &= 251;
  }

  if (PS4.PSButton()) {
    if (millis() - countTime >= 250) {
      countTime = millis();
      MSB |= 2;
    }
  } else if (!PS4.PSButton()) {
    MSB &= 253;
  }

  if (PS4.Options()) {
    MSB |= 1;
  } else if (!PS4.Options()) {
    MSB &= 254;
  }

  if (PS4.R3()) {
    LSB |= 128;
  } else if (!PS4.R3()) {
    LSB &= 127;
  }

  if (PS4.L3()) {
    LSB |= 64;
  } else if (!PS4.L3()) {
    LSB &= 191;
  }

  if (PS4.R2()) {
    LSB |= 32;
  } else if (!PS4.R2()) {
    LSB &= 223;
  }

  if (PS4.L2()) {
    LSB |= 16;  
  } else if (!PS4.L2()) {
    LSB &= 239;
  }

  if (PS4.Square()) {
    LSB |= 8;
  } else if (!PS4.Square()) {
    LSB &= 247;
  }

  if (PS4.Circle()) {
    LSB |= 4;
  } else if (!PS4.Circle()) {
    LSB &= 251;
  }

  if (PS4.Triangle()) {
    LSB |= 2;
  } else if (!PS4.Triangle()) {
    LSB &= 253;
  }

  if (PS4.Cross()) {
    LSB |= 1;  
  } else if (!PS4.Cross()) {
    LSB &= 254;
  }
}

void setup() {
  Serial.begin(9600);
  //Serial2.begin(9600,SERIAL_8N1,16,17);
  PS4.attach(choice);
  //PS4.begin("08:b6:1f:28:e7:32");
  PS4.begin("b0:05:94:b9:d2:b0");
  
}

void loop() {
  /*
  if (PS4.Battery() > 3) {
    PS4.setLed(223, 255, 0);
    PS4.sendToController();
  }
  else {
    PS4.setLed(223, 0, 0);
    PS4.sendToController();
  }*/
  
  if (millis() - prevTime >= 25) {
    prevTime = millis();

    if (DEBUG) {
    char msg[200];
    sprintf(msg, "%4d, %4d, %4d, %4d, %4d, %4d, %3d", LeftX, LeftY, RightX, RightY, MSB, LSB, PS4.Battery());
    Serial.println(msg);
    }

    if (!DEBUG) {
      Serial.write(MSB);
      Serial.write(LSB);
      Serial.write(RightY);
      Serial.write(LeftY);
      Serial.write(RightX);
      Serial.write(LeftX);
    }
  }

}
