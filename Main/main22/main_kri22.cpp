#include "mbed.h"
#include "rtos.h"
#include "stm32f4xx_hal.h"
#include "Driver.h"
#include "PID.h"
#include "AX12A.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include <cmath>
#include "SharpIR.h"




#define SDA PB_3
#define SCL PB_10
#define PI 3.14
#define sqrt2 1.4142

//--------------kp, ki untuk maju (joystick)------------------------
double kp1m[] = {
    0.0082, // Motor 0
    0.0087, // Motor 1
    0.0093, // Motor 2
    0.0088, // Motor 3
};
double ki1m[] = {
    0.0008100, // Motor 0
    0.0007750, // Motor 1
    0.0005500, // Motor 2
    0.00077939, // Motor 3
    //0.0008190, // Motor 0
    //0.000819, // Motor 1
    //0.0008178, // Motor 2
    //0.0008639, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk mundur (joystick)----------------------
// double kp1mu[] = {
//     0.0050054, // Motor 0
//     0.0050665, // Motor 1
//     0.005, // Motor 2
//     0.0041655, // Motor 3
// };
double kp1mu[] = {
    0.00022, // Motor 0
    0.00099, // Motor 1
    0.00086, // Motor 2
    0.00087, // Motor 3
};

double ki1mu[] = {
    0.00007900, // Motor 0
    0.0000975, // Motor 1
    0.00009, // Motor 2
    0.000087939, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kanan (joystick)----------------------
double kp1k[] = {
    0.0052, // Motor 0
    0.009, // Motor 1
    0.007, // Motor 2
    0.01, // Motor 3
};

double ki1k[] = {
    0.0007900, // Motor 0
    0.0007750, // Motor 1
    0.0001500, // Motor 2
    0.00057939, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kiri (joystick)----------------------
double kp1ki[] = {
    0.0052, // Motor 0
    0.009, // Motor 1
    0.007, // Motor 2
    0.01, // Motor 3
};

double ki1ki[] = {
    0.0007900, // Motor 0
    0.0007750, // Motor 1
    0.0001500, // Motor 2
    0.00057939, // Motor 3
};
//------------------------------------------------------------------

//------------kp, ki untuk mode kecepatan lambat--------------------
double kp2[] = {
    0.0089015, // Motor 0
    0.00564050, // Motor 1
    0.00533830, // Motor 2
    0.00584845, // Motor 3
};

double ki2[] = {
    0.000922, // Motor 0
    0.000929, // Motor 1
    0.000851, // Motor 2
    0.000991, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk maju (joystick)------------------------
double kp1boost[] = {
    0.00363054, // Motor 0
    0.0035815, // Motor 1
    0.00343045, // Motor 2
    0.0037375, // Motor 3
};
double ki1boost[] = {
    0.0008190, // Motor 0
    0.000819, // Motor 1
    0.0007878, // Motor 2
    0.0008639, // Motor 3
};
//------------------------------------------------------------------

double kd[] = {
    0.001, // Motor 0
    0.001, // Motor 1
    0.001, // Motor 2
    0.0005, // Motor 3
};

PID pid1m[] = { //PID maju
    PID(kp1m[0], ki1m[0], kd[0], 1000), // Motor 0
    PID(kp1m[1], ki1m[1], kd[1], 1000), // Motor 1
    PID(kp1m[2], ki1m[2], kd[2], 1000), // Motor 2
    PID(kp1m[3], ki1m[3], kd[3], 1000)  // Motor 3
};

PID pid1mu[] = { //PID mundur
    PID(kp1mu[0], ki1mu[0], kd[0], 1000), // Motor 0
    PID(kp1mu[1], ki1mu[1], kd[1], 1000), // Motor 1
    PID(kp1mu[2], ki1mu[2], kd[2], 1000), // Motor 2
    PID(kp1mu[3], ki1mu[3], kd[3], 1000)  // Motor 3
};

PID pid1k[] = { // PID kanan
    PID(kp1k[0], ki1k[0], kd[0], 1000), // Motor 0
    PID(kp1k[1], ki1k[1], kd[1], 1000), // Motor 1
    PID(kp1k[2], ki1k[2], kd[2], 1000), // Motor 2
    PID(kp1k[3], ki1k[3], kd[3], 1000)  // Motor 3
};

PID pid1ki[] = { // PID kiri
    PID(kp1ki[0], ki1ki[0], kd[0], 1000), // Motor 0
    PID(kp1ki[1], ki1ki[1], kd[1], 1000), // Motor 1
    PID(kp1ki[2], ki1ki[2], kd[2], 1000), // Motor 2
    PID(kp1ki[3], ki1ki[3], kd[3], 1000)  // Motor 3
};

PID pid2[] = {  // PID gerak lambat
    PID(kp2[0], ki2[0], kd[0], 1000), // Motor 0
    PID(kp2[1], ki2[1], kd[1], 1000), // Motor 1
    PID(kp2[2], ki2[2], kd[2], 1000), // Motor 2
    PID(kp2[3], ki2[3], kd[3], 1000)  // Motor 3
};

PID pid_boost[] = {  // PID boost
    PID(kp1boost[0], ki1boost[0], kd[0], 1000), // Motor 0
    PID(kp1boost[1], ki1boost[1], kd[1], 1000), // Motor 1
    PID(kp1boost[2], ki1boost[2], kd[2], 1000), // Motor 2
    PID(kp1boost[3], ki1boost[3], kd[3], 1000)  // Motor 3
};

PID pidsdt(0.00005,0.0008524,0,1000);  // PID sudut

encoderKRAI encMotor[]={
    encoderKRAI(PC_10, PC_11, 330),       // Motor 0
    encoderKRAI(PA_7, PA_6, 330),        // Motor 1
    encoderKRAI(PC_2, PC_3, 330),     // Motor 2
    encoderKRAI(PB_1, PB_12, 330)      // Motor 3
};

Driver motor[]={
    Driver(PA_13, PA_14, PA_15),    // Motor 0
    Driver(PC_6, PC_8, PC_9),      // Motor 1
    Driver(PC_13, PH_1, PB_7),    // Motor 2
    Driver(PB_14, PB_13, PB_15)    // Motor 3


    // Driver(PA_13, PA_14, PA_15),       // Motor 0
    // Driver(PC_6, PC_8, PC_9),      // Motor 1
    // Driver(PC_13, PC_12, PB_7),    // Motor 2
    // Driver(PB_14, PB_13, PB_15)    // Motor 3
};

/*  Konfigurasi motor (Mecanum)
     ___             ___            y
    | 0 |___________| 1 |           ^
    |___|           |___|           |
        |           |               |---> x
        |           |                  
     ___|           |___
    | 2 |___________| 3 |
    |___|           |___|
*/

/* Omniwheel x-Configuration
    ___             ___            y
    | 2 |___________| 0 |           ^
    |___|           |___|           |
        |           |               |---> x
        |           |                  
     ___|           |___
    | 3 |___________| 1 |
    |___|           |___|
*/

/* Omniwheel 3 wheels x-Configuration
                   ____          y
     ___ __________\ 0 \         ^
    / 2/|           |\__\        |
   /__/ |           |            |---> x
        |           |                  
        |           |
        |___________| 
                / 1/
               /__/
*/

SharpIR sensor(PB_0, 430);

//Objek serial
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 9600);
//Kalau joystick bermasalah, bisa diubah jadi unbuffered serial dengan beberapa penyesuaian
//BufferedSerial joystick(PA_9, PA_10, 9600);

//Serial replacement for Joystick
UART_HandleTypeDef uart;

// CMPS12
CMPS12_KRAI cmps(PB_9, PB_8, 0xc0);

//Objek serial servo
AX12A servo(PA_0, PA_1, PA_8, 1000000);

//----------------------------------- Global Variabel-----------------------------------
uint16_t controlSig = 0; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
uint8_t receivedData[7]; // Buffer to store raw joystick data
//Speed variables
int pulse[4] = {0};
double omega[4] = {0};
double pwmMotor[4] = {0};
    
float vx = 0, vy = 0, beta = 0; //vx,vy, dan beta untuk eksekusi dan perhitungan
int xymode, spdmode = 1; //Mode kecepatan, default = 40 cm/s

//char MSB[1]={0}, LSB[1]={0}, right[1]={0}, left[1] = {0};
float base = 0;
char MSB = 0, LSB = 0;
int RightY=0, LeftY=0; // inisialisasi analog joystick sumbu Y
int RightX=0, LeftX=0; // inisialisasi analog joystick sumbu Y
    

//void getJoyIn();            //Mendapatkan input joystick
void initJoystick();        //Mendapatkan input joystick
void decideCase();          //Ubah kondisi sesuai kasus input joystick
void taskComputeSpeed();    


int spd[4];
float rWheel = 3;   //Jari jari roda
float midX = 7.17;  //Jarak dari tengah robot ke roda, sumbu x (dalm cm)
//float midY = 7.17;  //Jarak dari tengah robot ke roda, sumbu y (dalam cm)
float pos = 10.14;    //Jarak pusat robot ke tengah roda (dalam cm)

//--------------------------------------Variable--------------------------------------
unsigned long spdChangeFlag = 0;
unsigned long computeSpeedFlag = 0;
unsigned long boosterChangeFlag = 0;
unsigned long gripChangeFlag = 0;
//int a = 0, aset = 0, a_prev = 0;
//int a_pivot = 0, aset_pivot = 0, a_prev_pivot = 0;
bool odom1 = false, odom2 = false, boost_forward = false;
bool odom1_pivot = false, odom2_pivot = false;

int theta = 0, initial_theta = 0;
float theta_joystick = 0;
float vxMode = 100, vyMode = 200, betaMode = 10; //Setpoint vx, vy, dan kecepatan putar
float vxtable[] = {27, 33}; //Dalam cm/s
float vytable[] = {27, 33}; //Dalam cm/s
float betatable[] = {0.05, 0.07}; //Dalam derajat/s atau rad/s ??? Lupa

//Servo variables
float roll[] = {810,490};   // {Paralel lantai, tegak lurus lantai}     ID: 11
float yaw[] = {590,690};    // {Grip di tempat, tarik dari tempat}      ID: 10
float Jepit_Lepastable[] = {270,550}; //jepit, lepas                   ID: 13
float Naik_Turuntable[] = {0, 720}; //bawah, atas                     ID: 12

short flag_grip_from_base = 0;
// 0 : posisi lain
// 1 : udh posisi siap grip
// 2 : udh selesai ambil
short flag_grip_ungrip = 0;
// 1 : Lagi Grip
// 2 : Lepas Grip
short flag_yaw = 0;
short flag_naikturun = 0;

float distance = 0;

// float CW_CCWtable[] = {204, 510, 817}; //segitiga, circle, cross

bool up = true, lepas = false, circle = true, cross = false;
//------------------------------------------------------------------------------------


int main(){
    //servo.setID(12, 2);
    boosterChangeFlag = us_ticker_read()/1000;
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 10);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 10);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 11);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 11);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 12);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 12);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 13);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 13);
    servo.setSpeed(900, 10);
    servo.setSpeed(900, 11);
    servo.setSpeed(900, 12);
    servo.setSpeed(900, 13);
    servo.move_to(roll[0], 11);
    ThisThread::sleep_for(70ms);
    servo.move_to(yaw[0], 10);
    ThisThread::sleep_for(70ms);
    servo.move_to(Jepit_Lepastable[0], 13);
    ThisThread::sleep_for(70ms);
    servo.move_to(0, 12);
    ThisThread::sleep_for(70ms);

    // servo.move_to(Jepit_Lepastable[0], 10);
    // ThisThread::sleep_for(70ms); 
    // servo.move_to(Naik_Turuntable[0], 3);
    // ThisThread::sleep_for(70ms); 
    // servo.move_to(CW_CCWtable[1], 2);

    spdChangeFlag = us_ticker_read()/1000;
    gripChangeFlag = us_ticker_read()/1000;
    initial_theta = cmps.getAngle()/10;
    controlSig = 0;
    printf("Started\n");
    initJoystick();

    while (1){
        
        //cmps.calibrate();
        //Get joystick input
        // if (joystick.readable()){
        //     joystick.read(MSB,1);
        //     joystick.read(LSB,1);
        //     controlSig = ((int)*MSB<<8|(int)*LSB);
        //     joystick.read(right,1);
        //     analog_RightY = (int)*right - 128;
        //     joystick.read(left,1 );
        //     analog_LeftY = (int)*left - 128;   
        //     joystick.read(right,1);
        //     analog_RightX = (int)*right - 128;
        //     joystick.read(left,1);
        //     analog_LeftX = (int)*left - 128;
        // }
        // else {
        //     //controlSig = 0;
        // }

        if (__HAL_UART_GET_FLAG(&uart, UART_FLAG_RXNE) != RESET) {
            
            for (int i = 0; i < sizeof(receivedData); i++) {
                HAL_UART_Receive(&uart, &receivedData[i], 1, HAL_MAX_DELAY);
                //printf("%d\t",receivedData[i]);
                
            }
            

            // Process received data
            char MSB = receivedData[1];
            char LSB = receivedData[2];
            char RightY = receivedData[3];
            char LeftY = receivedData[4];
            char RightX = receivedData[5];
            char LeftX = receivedData[6];

            controlSig = MSB << 8 | LSB;

            // Do something with the received data
            // ...

            // Print received data for testing
            // printf("MSB: %d\t LSB: %d\t RightY: %d\t LeftY: %d\t RightX: %d\t LeftX: %d\t Final: %d\r\n",
            // MSB, LSB, RightY, LeftY, RightX, LeftX, controlSig);
        }

        if (sensor.distance() < 5){
            if (flag_grip_ungrip != 1) {
                flag_grip_ungrip = 1;
                servo.move_to(Jepit_Lepastable[0], 13);
            }
        }
        
        //Tentukan aksi yang dilakukan
        decideCase();
        theta = (cmps.getAngle()/10 - initial_theta);

        if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
            computeSpeedFlag = us_ticker_read()/1000;
            taskComputeSpeed();
            printf("%d,%d,%d,%d\n", spd[0], -1*spd[1], spd[2], -1*spd[3]);
            //printf("%d\n", xymode)
            //printf("%d\n", controlSig);
            //printf("Analog X: %d\tAnalog Y: %d\n", analog_LeftX - 127,analog_LeftY - 127);
            //printf("Analog: %4d %4d\n", analog_LeftX, analog_LeftY);
            //printf("vx = %4d | vy = %4d\n", (int)vx, (int)vy);
            //printf("%d %d %d\n", controlSig, (int)vx, (int)vy);
        }

    }
}


// Functions
void taskComputeSpeed(){
    if (!boost_forward){    
        for (int i = 0; i<4; i++){
            spd[i] = encMotor[i].getPulses()/2;
        }
        for (int i = 0; i<4; i++){     
            encMotor[i].reset();
        }
        //Testing direction
        // omega[0] = 8;
        // omega[1] = 8;
        // omega[2] = 8;
        // omega[3] = 8;

        // +_config Omniwheel
        //omega[0] = ((1)*(vy-beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[2] = ((1)*(-vx+beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[3] = ((1)*(-vy-beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[1] = ((1)*(vx+beta*midX)/rWheel)/6.28*330*0.035;
  
        // + config OmniWheel + CMPS
        //omega[0] = ((1)*(vx * cos(PI/2 + theta) + vy * sin(PI/2 + theta) - beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[2] = ((1)*(vx * cos(PI + theta) + vy * sin(PI + theta) + beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[3] = ((1)*(vx * cos(3 * PI/2 + theta) + vy * sin(3 * PI/2 + theta) -beta*midX)/rWheel)/6.28*330*0.035; 
        //omega[1] = ((1)*(vx * cos(theta) + vy * sin(theta) +beta*midX)/rWheel)/6.28*330*0.035;
        
        // x_config Omniwheel (Equation with no regards to robots heading)
        omega[0] = ((1)*(vx/sqrt2 + vy/sqrt2 + beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        omega[1] = ((1)*(vx/sqrt2 - vy/sqrt2 + beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        omega[3] = ((1)*(-vx/sqrt2 - vy/sqrt2 + beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        omega[2] = ((1)*(-vx/sqrt2 + vy/sqrt2 + beta*pos/sqrt2)/rWheel)/6.28*330*0.035;
        
        // x_config Omniwheel (Complete Equation)
        //omega[0] = (beta*(pos * cos(PI/4 + theta) * sin(3*PI/4) - pos * sin(PI/4 + theta) * cos(3*PI/4)) + vx * cos(3*PI/4 + theta) + vy * sin(3*PI/4 + theta))/rWheel/6.28*330*0.035;
        //omega[2] = (beta*(pos * cos(3*PI/4 + theta) * sin(PI/4) - pos * sin(3*PI/4 + theta) * cos(PI/4)) + vx * cos(PI/4 + theta) + vy * sin(PI/4 + theta))/rWheel/6.28*330*0.035;
        //omega[3] = (beta*(pos * cos(5*PI/4 + theta) * sin(7*PI/4) - pos * sin(5*PI/4 + theta) * cos(7*PI/4)) + vx * cos(7*PI/4 + theta) + vy * sin(7*PI/4 + theta))/rWheel/6.28*330*0.035;
        //omega[1] = (beta*(pos * cos(7*PI/4 + theta) * sin(5*PI/4) - pos * sin(7*PI/4 + theta) * cos(5*PI/4)) + vx * cos(5*PI/4 + theta) + vy * sin(5*PI/4 + theta))/rWheel/6.28*330*0.035;

        // 4 Mecanum Wheel
        //omega[0] = ((vy-vx-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640
        //omega[1] = ((vy+vx+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
        //omega[2] = ((vy+vx-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
        //omega[3] = ((vy-vx+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640

        // x_config Omniwheel 3 wheels
        // omega[0] = ((1)*(vx + 0 - beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        // omega[1] = ((1)*(-vx/2 - vy*sin(PI/3) - beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        // omega[3] = ((1)*(-vx/2 + vy*sin(PI/3) - beta*pos/sqrt2)/rWheel)/6.28*330*0.035; 
        // omega[2] = 0;
    
        if (vy!=0 && vx!=0 && beta ==0){ 
            for (int i = 0; i < 4; i++){
                if (omega[i] > 23) {
                    omega[i] = 23; 
                }
                else if (omega[i] < -23){
                    omega[i] = -23;
                }
            }
        }
        
        for (int i = 0; i<4; i++){
            if (spdmode){ // mode gerak cepat
                if (xymode==0){ // pid maju
                    pwmMotor[i] = pid1m[i].createpwm(omega[i], spd[i]);
                }
                else if (xymode==1) { // pid kanan
                    pwmMotor[i] = pid1k[i].createpwm(omega[i], spd[i]);
                }
                else if (xymode==2) { // pid mundur 
                    pwmMotor[i] = pid1mu[i].createpwm(omega[i], spd[i]);
                }
                else { // pid kiri
                    pwmMotor[i] = pid1ki[i].createpwm(omega[i], spd[i]);
                }
            }
            else { // mode gerak lambat
                pwmMotor[i] = pid2[i].createpwm(omega[i], spd[i]);
            }
        }
        for (int i = 0; i<4; i++){
            motor[i].Motion(pwmMotor[i]);
        }
        xymode = 0;
    }
    else {
        spdChangeFlag = us_ticker_read()/1000;
        computeSpeedFlag = us_ticker_read()/1000;
        while ((us_ticker_read()/1000)-spdChangeFlag <= 800){
            if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
                computeSpeedFlag = us_ticker_read()/1000;
                for (int i = 0; i<4; i++){
                    spd[i] = encMotor[i].getPulses()/2;
                }
                for (int i = 0; i<4; i++){     
                    encMotor[i].reset();
                }
                
                omega[0] = (80.0/rWheel)/6.28*330*0.035; //640
                omega[1] = 0;//(80.0/rWheel)/6.28*330*0.035; //1050
                omega[2] = 0;//(80.0/rWheel)/6.28*330*0.035; //1050
                omega[3] = (80.0/rWheel)/6.28*330*0.035; //640
                
                for (int i = 0; i<4; i++){
                    pwmMotor[i] = pid_boost[i].createpwm(omega[i], spd[i]);
                }
                for (int i = 0; i<4; i++){
                    motor[i].Motion(pwmMotor[i]);
                }
            }
        }
        boost_forward = false;
    }
}

 
void decideCase(){

    //theta_joystick = atan2(analog_LeftY,analog_LeftX);

    
    
    
    if ((controlSig & 32768) == 32768){ //Up
        vy = vyMode;
        xymode = 0;
    }
    else if ((controlSig & 16384) == 16384){ //Down
        vy = -vyMode;
        xymode = 2;
    }

    if ((controlSig & 8192) == 8192){ //Left
        if(xymode != 2){
            vx = -vxMode;
            xymode = 3;
        }
    }
    else if ((controlSig & 4096) == 4096){ //Right
        if(xymode != 2){
            vx = vxMode;
            xymode = 1;
        }
    }

    // Rotation (Yaw)
    if ((controlSig & 2048) == 2048){ //R1
        beta = -10; // omniwheel
    }
    else if ((controlSig & 1024) == 1024){ //L1
        beta = 10; // omniwheel
    }
    else if ((controlSig & 32) == 32) { //R2
        beta = -2; // omniwheel
    }
    else if ((controlSig & 16) == 16) { //L2
        beta = 2; // omniwheel
    }
    else{
        beta = 0;
    }

    if ((controlSig & 512) == 512){ //Start
        initial_theta = cmps.getAngle()/10; 
    }

    if (controlSig == 0) {
        vx = 0;
        vy = 0;
    }

    if (LeftY >= 100 && LeftY <= 200 && LeftX >= 100 && LeftX <= 200 && controlSig == 0) {
        vx = 0; vy = 0;
    }

    
    // else if (LeftY < 113 || LeftY > 143 || LeftX < 113 || LeftX > 143) {
    //     int magnitude = sqrt(pow(LeftX - 128, LeftY - 128));
    //     vx = vxMode * (LeftX - 128) / magnitude;
    //     vy = vyMode * (LeftY - 128) / magnitude;
    // }
    

    



    /*if (analog_RightY == 255 ){ //180 degree to right
        a = cmps.getAngle()/10;
        if (!odom1 && !odom2){f
            aset = a+82; a_prev = a;
            if (aset>360){
                odom2 = true;
                aset = aset%360;
            }
            else {
                odom1 = true;
                aset = aset%360;
            }
            vx = -(vxMode); vy = 0;
            beta = -(vxMode+80)/(10.0+12.5);
        }
        else if (odom1){
            if(a>=aset || a<a_prev){
                beta = 0;
                vy = vyMode; vx = 0;
            }
            else{
                vx = -(vxMode); vy = 0;
                beta = -(vxMode+80)/(10.0+12.5);
            }
        }
        else if (odom2){
            if(a>=aset && a<a_prev){
                beta = 0;
                vy = vyMode; vx = 0;
            }
            else{
                vx = -(vxMode); vy = 0;
                beta = -(vxMode+80)/(10.0+12.5);
            }
        }
        xymode = 0;
    }
    else if (analog_LeftY == 255 ){ //180 degree to left
        a = cmps.getAngle()/10;
        if (!odom1 && !odom2){
            aset = (a-82)%360; a_prev = a;
            if (aset<0){
                aset = 360 + aset;
                odom2 = true;
            }
            else {
                odom1 = true;
            }
            vx = (vxMode); vy = 0;
            beta = (vxMode + 80)/(10.0+12.5); 
        }
        else if (odom1){
            if(a<=aset || a>a_prev){
                beta = 0;
                vy = vyMode; vx = 0;
            }
            else{
                vx = (vxMode); vy = 0;
                beta = (vxMode + 80)/(10.0+12.5);
            }
        }
        else if (odom2){
            if(a<=aset && a>a_prev){
                beta = 0;
                vy = vyMode; vx = 0;
            }
            else{
                vx = (vxMode); vy = 0;
                beta = (vxMode + 80)/(10.0+12.5);
            }
        }
        xymode = 0;
    }
    else if (analog_RightY == 0){ //pivot to right
        beta = -betaMode;
    }
    else if (analog_LeftY == 0){ //pivot to left
        beta = betaMode;
    }
    else {
        beta = 0; odom1 = false; odom2 = false; aset = a = a_prev;
    }*/
    /*
    if(analog_LeftY == 127 && analog_RightY == 127) {
        if ((controlSig & 256) == 256){ //L3
            a_pivot = cmps.getAngle()/10;
            if (!odom1_pivot && !odom2_pivot){
                aset_pivot = a_pivot+136; a_prev_pivot = a_pivot;
                if (aset_pivot>360){
                    odom2_pivot = true;
                    aset_pivot = aset_pivot%360;
                }
                else {
                    odom1_pivot = true;
                    aset_pivot = aset_pivot%360;
                }
                vx = (vxMode+80);
                beta = -1*(vxMode + 80)/(10.0+12.5);
            }
            else if (odom1_pivot){
                if(a_pivot>=aset_pivot || a_pivot<a_prev_pivot){
                    beta = 0;vx = 0;
                }
                else{
                    vx = (vxMode+80);
                    beta = -1*(vxMode + 80)/(10.0+12.5);
                }
            }
            else if (odom2_pivot){
                if(a_pivot>=aset_pivot && a_pivot<a_prev_pivot){
                    beta = 0; vx = 0;
                }
                else{
                    vx = (vxMode+80);
                    beta = -1*(vxMode + 80)/(10.0+12.5);
                }
            }
            xymode = 0;
        }
        else{
            if (controlSig < 4095){
                vx = 0;
                beta = 0;
            }
            else{
                beta = 0;
            }
            odom1_pivot = false; odom2_pivot = false; aset_pivot = a_pivot = a_prev_pivot;
        }
    }*/
    /*
    if ((controlSig & 512) == 512){ //Start
        if ((us_ticker_read()/1000)-boosterChangeFlag >= 9500){
            boosterChangeFlag = us_ticker_read()/1000;
            boost_forward = true;
        }
    }*/
    
    
    //if ((controlSig & 64) == 64){ //L3
    //    if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
    //        spdChangeFlag = us_ticker_read()/1000;
    //        //Cycle mode kecepatan 
    //        if (spdmode == 1){
    //            spdmode--;
    //            vxMode = vxtable[spdmode];
    //            vyMode = vytable[spdmode];
    //            betaMode = betatable[spdmode];
    //        }
    //        else if (spdmode == 0){
    //            spdmode++;
    //            vxMode = vxtable[spdmode];
    //            vyMode = vytable[spdmode];
    //            betaMode = betatable[spdmode];
    //        }
    //    }
    //}

    // if ((controlSig & 32) == 32){ //R2
    //     if ((us_ticker_read()/1000)-spdChangeFlag >= 320){
    //         spdChangeFlag = us_ticker_read()/1000;
    //         if(!lepas && (up == true || circle == false)){
    //             servo.move_to(Jepit_Lepastable[1], 1);
    //             lepas = true;
    //         }
    //         else{
    //             servo.move_to(Jepit_Lepastable[0], 1);
    //             lepas = false;
    //         }
    //     }
    // }

    if ((controlSig & 128) == 128){ //R3
        servo.setTorque_Limit(1023,10);
        servo.setTorque_Limit(1023,11);
        servo.setTorque_Limit(1023,12);
        servo.setTorque_Limit(1023,13);
    }

    if ((controlSig & 4) == 4){ //Circle
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
            spdChangeFlag = us_ticker_read()/1000;
            // if(!up){
            //     servo.move_to(yaw[0], 10);
            //     up = true;
            // }
            // else if (!circle){
            //     servo.move_to(yaw[1], 10);
            //     up = false;
            // }
            // else if (circle && !lepas){
            //     servo.move_to(Naik_Turuntable[1], 10);
            //     up = false;
            // }
            if (flag_yaw != 1) {
                flag_yaw = 1;
                servo.move_to(yaw[0], 10);
            }
            else if (flag_yaw != 2) {
                flag_yaw = 2;
                servo.move_to(yaw[1], 10);
            }
        }
    }

    if ((controlSig & 1) == 1){ //Cross
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
            spdChangeFlag = us_ticker_read()/1000;
            // servo.move_to(roll[1], 1);
            // circle = true; cross = false;
            if (flag_naikturun != 1) {
                flag_naikturun = 1;
                servo.move_to(Naik_Turuntable[0], 12);
            }
            else if (flag_naikturun == 1) {
                flag_naikturun = 2;
                servo.move_to(Naik_Turuntable[1], 12);
            }
        }
    }
    else if ((controlSig & 2) == 2){ //Triangle
        // if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && ((up == true)&&(cross == false || lepas == false))){
        //     spdChangeFlag = us_ticker_read()/1000;
        //     servo.move_to(roll[0], 2);
        //     circle = false; cross = false;
        // }
        if ((us_ticker_read()/1000) - spdChangeFlag >= 200) {
            spdChangeFlag = us_ticker_read()/1000;
            if (flag_grip_ungrip != 1) {
                flag_grip_ungrip = 1;
                servo.move_to(Jepit_Lepastable[0], 13);
            }
            else if (flag_grip_ungrip == 1) {
                flag_grip_ungrip = 2;
                servo.move_to(Jepit_Lepastable[1], 13);
            }
        }
    }  
    else if ((controlSig & 8) == 8){ //Square
        // if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && (up == true && lepas == false)){
        //     spdChangeFlag = us_ticker_read()/1000;
        //     servo.move_to(roll[1], 2);
        //     circle = false; cross = true;
        // }
        if ((us_ticker_read()/1000) - spdChangeFlag >= 250) {
            gripChangeFlag = us_ticker_read()/1000;
            spdChangeFlag = us_ticker_read()/1000;
            if (flag_grip_from_base != 1) {
                flag_grip_from_base = 1;
                servo.move_to(roll[1], 11);
            }
            else if (flag_grip_from_base == 1) {
                flag_grip_from_base = 2;
                servo.move_to(roll[0], 11);
            }
        
        // if ((us_ticker_read()/1000) - gripChangeFlag >= 200) {
        //     gripChangeFlag = us_ticker_read()/1000;
        //     if (flag_grip_from_base != 1) {
        //             //Posisi gripper ke atas
        //             servo.move_to(Naik_Turuntable[1], 12);
        //             flag_naikturun = 2;
        //             flag_grip_from_base = 1;
        //             if ((us_ticker_read()/1000) - gripChangeFlag >= 200 && flag_grip_from_base == 1) {
        //                 gripChangeFlag = us_ticker_read()/1000;
        //                 //Posisi gripper tegak lurus lantai
        //                 servo.move_to(roll[1], 11);
        //                 // Buka gripper
        //                 servo.move_to(Jepit_Lepastable[1], 13);
        //                 flag_grip_ungrip = 2;
        //                 flag_grip_from_base = 1;
        //             }
        //     }

        //     else if (flag_grip_from_base == 1) {
        //         //gripChangeFlag = us_ticker_read()/1000;
        //         // Tutup Gripper
        //         servo.move_to(Jepit_Lepastable[0], 13);
        //         // Angkat Koin
        //         servo.move_to(yaw[1], 10);
        //         // Posisi Gripper Paralel Lantai
        //         servo.move_to(roll[0], 11);
        //         // Gripper Turun
        //         servo.move_to(Naik_Turuntable[0], 12);
        //         flag_naikturun = 1;
        //         if ((us_ticker_read()/1000) - gripChangeFlag >= 200 && flag_grip_from_base == 1) {
        //             gripChangeFlag = us_ticker_read()/1000;
        //             // Posisi Koin lurus ke depan
        //             servo.move_to(yaw[0], 10);
        //             flag_yaw = 1;
        //             flag_grip_from_base = 2;
        //         }
        //     }
        }
    }
}

void initJoystick() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
  
    GPIO_InitTypeDef gpioInit;
    gpioInit.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
  
    uart.Instance = USART1;
    uart.Init.BaudRate = 9600;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
  
    if (HAL_UART_Init(&uart) != HAL_OK) {
        vx = 0; vy = 0;
    }
}
