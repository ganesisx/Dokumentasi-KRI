#include "mbed.h"
#include "Driver.h"
#include "PID.h"
#include "encoderKRAI.h"

double kp1 = 0.00022414;
double ki1 = 0.000067105;
double kd1 = 0.00018717;

double kp2 = 0.0002375;
double ki2 = 0.000076444;
double kd2 = 0.0001577;

double kp3 = 0.00027617;
double ki3 = 0.000084701;
double kd3 = 0.00020134;

double kp4 = 0.003004;
double ki4 = 0.000078;
double kd4 = 0.018623;

encoderKRAI encMotor1(PC_12, PC_11, 330);
encoderKRAI encMotor2(PA_6, PA_7, 330);
encoderKRAI encMotor3(PB_12, PB_1, 330);
encoderKRAI encMotor4(PC_2, PC_3, 330);

Driver motor1(PA_13, PA_14, PA_15);
Driver motor2(PC_8, PC_6, PC_9);
Driver motor3(PB_13, PB_14, PB_15);
Driver motor4(PC_10, PC_13, PB_7);

PID pid1(kp1,ki1,kd1,1000);
PID pid2(kp2,ki2,kd2,1000);
PID pid3(kp3,ki3,kd3,1000);
PID pid4(kp4,ki4,kd4,1000);

int pulses1, pulses2, pulses3,pulses4;
int spd1, spd2, spd3, spd4;
double tuning1, tuning2, tuning3, tuning4;
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);

Thread thread;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &pc;
}

void resetBacaan(){
    while(1){
        spd1 = pulses1;
        spd2 = pulses2;
        spd3 = pulses3;
        spd4 = pulses4;
        
        tuning1 = pid1.createpwm(28,spd1);
        tuning2 = pid2.createpwm(28,spd2);
        tuning3 = pid3.createpwm(28,spd3);
        tuning4 = pid4.createpwm(28,spd4);
        
        printf("%d %f\n", spd3, tuning3);
        
        encMotor1.reset();
        encMotor2.reset();
        encMotor3.reset();
        encMotor4.reset();
        
        ThisThread::sleep_for(100ms);   
    }
 }
 
int main(){
    thread.start(resetBacaan);
    while (1){
        pulses1 = encMotor1.getPulses();
        pulses2 = encMotor2.getPulses();
        pulses3 = encMotor3.getPulses();
        pulses4 = encMotor4.getPulses();
        
        /*if (pulses1 == 25){
            printf("raising Time 1 = %d\n", us_ticker_read()/1000);
            }
        if (pulses2 == 25){
            printf("raising Time 2 = %d\n", us_ticker_read()/1000);
            }
        if (pulses3 == 25){
            printf("raising Time 3 = %d\n", us_ticker_read()/1000);
            }
        if (pulses4 == 25){
            printf("raising Time 4 = %d\n", us_ticker_read()/1000);
            }*/
            
        motor1.Motion(tuning1);
        motor2.Motion(tuning2);
        motor3.Motion(tuning3);
        motor4.Motion(tuning4);
    }
}
