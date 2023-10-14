#include "mbed.h"
#include "AX12A.h"

BufferedSerial pc(USBTX, USBRX, 57600);

AX12A motor2(PA_0, PA_1, PA_8, 1000000); 

int main()
{
    char status[1] = {0};
    while(1){
        //motor2.toggleLED(1);
        //pc.write(status, 1);
        //ThisThread::sleep_for(200ms);
        //motor2.toggleLED(0);
        //ThisThread::sleep_for(200ms);
        //printf("1.%s\n", status);
        motor2.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 2);
        motor2.setAngleLimit(0, ADDRESS_CCW_ANGLE_LIMIT, 2);
        motor2.setSpeed(300, 2);
        //motor2.move_to(0);
        // ThisThread::sleep_for(2000ms);
        // motor2.setSpeed(1500);
        // ThisThread::sleep_for(4000ms);
        // motor2.setSpeed(0);
        // motor2.setID(3);
    }
}