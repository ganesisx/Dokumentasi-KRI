#include <Ps3Controller.h>

void setup()
{
    Serial.begin(115200);
    Ps3.begin("11:00:22:99:33:88");
    Serial.println("Ready.");
}

unsigned long prevTime, countTime;
uint8_t MSB=0, LSB=0;

void loop()
{

    if(Ps3.isConnected()){
      if (millis()-prevTime >= 25){
        prevTime = millis();
        //MSB
        //up
        if( Ps3.data.button.up ){
            MSB |= 128;
        }
        else if( Ps3.data.button.up ){
            MSB &= 127;
        }
        //down
        if( Ps3.data.button.down ){
            MSB |= 64;
        }
        else if( Ps3.data.button.down ){
            MSB &= 191;
        }
        //left
        if( Ps3.data.button.left ){
            MSB |= 32;
        }
        else if( Ps3.data.button.left ){
            MSB &= 223;
        }
        //right
        if( Ps3.data.button.right ){
            MSB |= 16;
        }
        else if( Ps3.data.button.right ){
            MSB &= 239;
        }
        //r1
        if( Ps3.data.button.r1 ){
            MSB |= 8;
        }
        else if( Ps3.data.button.r1 ){
            MSB &= 247;
        }
        //l1
        if( Ps3.data.button.l1 ){
            MSB |= 4;
        }
        else if( Ps3.data.button.l1 ){
            MSB &= 251;
        }
        //start
        if( Ps3.data.button.start ){
            if (millis()-countTime >= 250){
              countTime = millis();
              MSB |= 2;
              //PS3.setLedOff();
              //PS3.setLedOn(LED1);
            }
        }
        else if( Ps3.data.button.start ){
            MSB &= 253;
        }
        //select
        if( Ps3.data.button.select ){
            if (millis()-countTime >= 250){
              countTime = millis();
              MSB |= 1;
              //PS3.setLedOff();
              //PS3.setLedOn(LED2);
            }
        }
        else if( Ps3.data.button.select ){
            MSB &= 254;
        }
        //LSB
        //r3
        if( Ps3.data.button.r3 ){
            LSB |= 128;
        }
        else if( Ps3.data.button.r3 ){
            LSB &= 127;
        }
        //l3
        if( Ps3.data.button.l3 ){
            LSB |= 64;
        }
        else if( Ps3.data.button.l3 ){
            LSB &= 191;
        }
        //r2
        if( Ps3.data.button.r2 ){
            if (millis()-countTime >= 200){
              countTime = millis();
              LSB |= 32;
            }
        }
        else if( Ps3.data.button.r2 ){
            LSB &= 223;
        }
        //l2
        if( Ps3.data.button.l2 ){
            if (millis()-countTime >= 200){
              countTime = millis();
              LSB |= 16;
            }
        }
        else if( Ps3.data.button.l2 ){
            LSB &= 239;
        }
        //square
        if( Ps3.data.button.square ){
            if (millis()-countTime >= 200){
              countTime = millis();
              LSB |= 8;
            }
        }
        else if( Ps3.data.button.square ){
            LSB &= 247;
        }
        //circle
        if( Ps3.data.button.circle ){
            if (millis()-countTime >= 200){
              countTime = millis();
              LSB |= 4;
            }
        }
        else if( Ps3.data.button.circle ){
            LSB &= 251;
        }
        //triangle
        if( Ps3.data.button.triangle ){
            if (millis()-countTime >= 200){
              countTime = millis();
              LSB |= 2;
            }
        }
        else if( Ps3.data.button.triangle ){
            LSB &= 253;
        }
        //cross
        if( Ps3.data.button.cross ){
            if (millis()-countTime >= 200){
             countTime = millis();
             LSB &= 254;
            }
        }
        else if( Ps3.data.button.cross ){
            LSB &= 254;
        }
        //ps
        if( Ps3.data.button.ps ){
            Serial.println("ps");
        }
        //analog
        if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
        Serial.print("Moved the left stick:");
        Serial.print(" x="); Serial.print(Ps3.data.analog.stick.lx, DEC);
        Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ly, DEC);
        Serial.println();
        }
        if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
        Serial.print("Moved the right stick:");
        Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
        Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
        Serial.println();
        }
      }        
    }
}
