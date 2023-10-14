#include "mbed.h"
#include "rtos.h"
#include "Driver.h"
#include "PID.h"
#include "AX12A.h"
#include "encoderKRAI.h"

#define SDA PB_3
#define SCL PB_10
#define PI 3.14

double kp1m[] = {
    0.00370054, // Motor 0
    0.0013665, // Motor 1
    0.00353045, // Motor 2
    0.0040655, // Motor 3
};
double ki1m[] = {
    0.0009900, // Motor 0
    0.00425, // Motor 1
    0.002048, // Motor 2
    0.0009939, // Motor 3
};

double kp1mu[] = {
    0.010054, // Motor 0
    0.0000073665, // Motor 1
    0.0005, // Motor 2
    0.0040655, // Motor 3
};

double ki1mu[] = {
    0.0009900, // Motor 0
    0.00000725, // Motor 1
    0.000002, // Motor 2
    0.0009939, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kiri kanan (joystick)------------------
double kp1k[] = {
    0.0590014, // Motor 0
    0.0490740, // Motor 1
    0.0527960, // Motor 2
    0.0595955, // Motor 3
};

double ki1k[] = {
    0.001081, // Motor 0
    0.0002024, // Motor 1
    0.000156, // Motor 2
    0.0002039, // Motor 3
};
//------------------------------------------------------------------

//------------kp, ki untuk mode kecepatan lambat--------------------
double kp2[] = {
    0.008015, // Motor 0
    0.0064050, // Motor 1
    0.0063830, // Motor 2
    0.0084845, // Motor 3
};

double ki2[] = {
    0.0000922, // Motor 0
    0.0000929, // Motor 1
    0.0000851, // Motor 2
    0.0000953, // Motor 3
};
//------------------------------------------------------------------
double kd[] = {
    0.00, // Motor 0
    0.00, // Motor 1
    0.00, // Motor 2
    0.00, // Motor 3
};


PID pid1m[] = { //PID maju mundur
    PID(kp1m[0], ki1m[0], kd[0], 1000), // Motor 0
    PID(kp1m[1], ki1m[1], kd[1], 1000), // Motor 1
    PID(kp1m[2], ki1m[2], kd[2], 1000), // Motor 2
    PID(kp1m[3], ki1m[3], kd[3], 1000)  // Motor 3
};

PID pid1mu[] = { //PID maju mundur
    PID(kp1mu[0], ki1mu[0], kd[0], 1000), // Motor 0
    PID(kp1mu[1], ki1mu[1], kd[1], 1000), // Motor 1
    PID(kp1mu[2], ki1mu[2], kd[2], 1000), // Motor 2
    PID(kp1mu[3], ki1mu[3], kd[3], 1000)  // Motor 3
};

PID pid1k[] = { // PID kiri kanan
    PID(kp1k[0], ki1k[0], kd[0], 1000), // Motor 0
    PID(kp1k[1], ki1k[1], kd[1], 1000), // Motor 1
    PID(kp1k[2], ki1k[2], kd[2], 1000), // Motor 2
    PID(kp1k[3], ki1k[3], kd[3], 1000)  // Motor 3
};

PID pid2[] = {  // PID gerak lambat
    PID(kp2[0], ki2[0], kd[0], 1000), // Motor 0
    PID(kp2[1], ki2[1], kd[1], 1000), // Motor 1
    PID(kp2[2], ki2[2], kd[2], 1000), // Motor 2
    PID(kp2[3], ki2[3], kd[3], 1000)  // Motor 3
};

PID pidsdt(0.085705,0.0008524,0,1000);  // PID sudut

encoderKRAI encMotor[]={
    encoderKRAI(PC_11, PC_10, 330),       // Motor 0
    encoderKRAI(PA_7, PA_6, 330),        // Motor 1
    encoderKRAI(PC_3, PC_2, 330),     // Motor 2
    encoderKRAI(PB_1, PB_12, 330)      // Motor 3
};

Driver motor[]={
    Driver(PA_13, PA_14, PA_15),       // Motor 0
    Driver(PC_6, PC_8, PC_9),      // Motor 1
    Driver(PC_12, PC_13, PB_7),    // Motor 2
    Driver(PB_13, PB_14, PB_15)    // Motor 3
};

/*  Konfigurasi motor :
     ___             ___            y
    | 0 |___________| 1 |           ^
    |___|           |___|           |
        |           |               |---> x
        |           |                  
     ___|           |___
    | 2 |___________| 3 |
    |___|           |___|
*/

//Objek serial
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 9600);
//Kalau joystick bermasalah, bisa diubah jadi unbuffered serial dengan beberapa penyesuaian
BufferedSerial joystick(PA_9, PA_10, 9600);
//Serial ESP32
//BufferedSerial esp(PB_10, PC_5, 1000000);

//Objek serial servo
AX12A servo(PA_0, PA_1, PA_8, 1000000);

//----------------------------------- Global Variabel-----------------------------------
uint16_t controlSig = 0; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
//Speed variables
int pulse[4] = {0};
double omega[4] = {0};
double pwmMotor[4] = {0};
float belog=0; // kecepatan rotasi analog

int analog_LeftX=127, analog_LeftY=127, 
    analog_RightX=127, analog_RightY=127; // inisialisasi analog joystick
    
float vx = 0, vy = 0, beta = 0; //vx,vy, dan beta untuk eksekusi dan perhitungan
int xymode, spdmode = 1; //Mode kecepatan, default = 40 cm/s

float   sudut = 0,          // sudut robot menurut cmps12
        angle_Left = 0,     // sudut analog kiri
        angle_Right = 0;    // sudut analog kanan

char MSB[1]={0}, LSB[1]={0}, temp[1]={0};
float base = 0;

void getJoyIn();            //Mendapatkan input joystick
float getAngle(int X, int Y);
int getSpeed(int X, int Y);
void decideCase();          //Ubah kondisi sesuai kasus input joystick
void taskComputeSpeed();   


int spd[4];
float rWheel = 3;   //Jari jari roda
float midX = 10;    //Jarak dari tengah robot ke roda, sumbu x
float midY = 12.5;  //Jarak dari tengah robot ke roda, sumbu y
float   vytot = 0, // kecepatan maju mundur total
        vxtot = 0, // kecepatan kiri kanan total
        vtemp = 0, // magnitude kecepatan analog
        betot = 0; // kecepatan rotasi total

//--------------------------------------Variable--------------------------------------
unsigned long spdChangeFlag = 0;
float pivot=50.0;

float vxMode = 78, vyMode = 78, betaMode = 2.8; //Setpoint vx, vy, dan kecepatan putar
float vxtable[] = {40, 78}; //Dalam cm/s
float vytable[] = {40, 78}; //Dalam cm/s
float betatable[] = {0.9, 2.8}; //Dalam derajat/s atau rad/s ??? Lupa

//Servo variables
float Naik_Turuntable[] = {850, 390}; //dalam bit (0-1023)
float CW_CCWtable[] = {204, 510, 817}; //dalam bit (0-1023)
float Jepit_Lepastable[] = {812, 510};
bool up = true, jpt = false, circle = true;
//------------------------------------------------------------------------------------

unsigned long computeSpeedFlag;

int main(){
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 1);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 1);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 2);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 2);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 3);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 3);
    servo.setSpeed(600, 2);
    servo.setSpeed(600, 3);
    servo.setSpeed(600, 1);
    servo.move_to(Jepit_Lepastable[0], 1);
    ThisThread::sleep_for(70ms); 
    servo.move_to(Naik_Turuntable[0], 3);
    ThisThread::sleep_for(70ms); 
    servo.move_to(CW_CCWtable[1], 2);

    while (1){
        //Get joystick input
        if (joystick.readable()){
            joystick.read(MSB,1);
            joystick.read(LSB,1);
            controlSig = ((int)*MSB<<8|(int)*LSB);   
            joystick.read(temp,1);
            analog_LeftX = (int)*temp;
            joystick.read(temp,1);
            analog_LeftY = (int)*temp;
        }
        
        //Tentukan aksi yang dilakukan
        decideCase();

        if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
            computeSpeedFlag = us_ticker_read()/1000;
            angle_Left = getAngle(analog_LeftX, analog_LeftY);
            taskComputeSpeed();
            //printf("%d %d %d %d\n", spd[0], spd[1], spd[2], spd[3]);
        }
    }
}

// Functions
void taskComputeSpeed(){
     angle_Left = getAngle(analog_LeftX, analog_LeftY);

        vtemp = getSpeed(analog_LeftX, analog_LeftY);
        vytot = vy + vtemp*cos(angle_Left);
        vxtot = vx + vtemp*sin(angle_Left);
        
        for (int i = 0; i<4; i++){
            spd[i] = encMotor[i].getPulses()/2;
        }
        
        omega[0] = ((vytot-vxtot-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640
        omega[1] = ((vytot+vxtot+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
        omega[2] = ((vytot+vxtot-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
        omega[3] = ((vytot-vxtot+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640
        
        for (int i = 0; i<4; i++){
            if (spdmode){ // mode gerak cepat
                if (xymode==0){ // pid maju mundur
                    pwmMotor[i] = pid1m[i].createpwm(omega[i], spd[i]);
                }
                else if (xymode==1) { //pid ke kanan dan ke kiri
                    pwmMotor[i] = pid1k[i].createpwm(omega[i], spd[i]);
                }
                else {
                    pwmMotor[i] = pid1mu[i].createpwm(omega[i], spd[i]);
                }
            }
            else { // mode gerak lambat
                pwmMotor[i] = pid2[i].createpwm(omega[i], spd[i]);
            }
            
            //batas kecepatan
            if (pwmMotor[i] > 0.8){
                pwmMotor[i] = 0.8;
            }
            else if (pwmMotor[i] < -0.8){
                pwmMotor[i] = -0.8;
            }
        }
        for (int i = 0; i<4; i++){
            motor[i].Motion(pwmMotor[i]);
        }

        for (int i = 0; i<4; i++){     
            encMotor[i].reset();
        }
    
 }
 
 void decideCase(){
    if ((controlSig & 32768) == 32768){ //Up
         vy = vyMode;
         xymode = 0;
         }
     else if ((controlSig & 16384) == 16384){ //Down
         vy = -vyMode;
         xymode = 2;
         }
     else{
         vy = 0;
         xymode = 0;
         }
     if ((controlSig & 8192) == 8192){ //Left
         vx = vxMode;
         xymode = 1;
         }
     else if ((controlSig & 4096) == 4096){ //Right
         vx = -vxMode;
         xymode = 1;
         }
     else{
         vx = 0;
         xymode = 1;
         }
     if ((controlSig & 2048) == 2048){ //R1
        if(vy != 0){
            beta = -(betatable[0]+0.1);
        }
        else {
            beta = -betaMode;
        }
        xymode = 0;
        }
     else if ((controlSig & 1024) == 1024){ //L1
        if(vy != 0){
            beta = betatable[0]+0.1;
        }
        else {
            beta = betaMode;
        }
        xymode = 0;
        }
     else{
         beta = 0;
         xymode = 0;
         }
     
     if ((controlSig & 512) == 512){ //Start
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                //Cycle kecepatan +
                if (spdmode < 1) spdmode++;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
         }
     else if ((controlSig & 256) == 256){ //Select
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                //Cycle kecepatan -
                if (spdmode > 0) spdmode--;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
         }
     if ((controlSig & 32) == 32){ //R2
         if ((us_ticker_read()/1000)-spdChangeFlag >= 320){
                spdChangeFlag = us_ticker_read()/1000;
                if(!jpt && (up == true || circle == false)){
                    servo.move_to(Jepit_Lepastable[1], 1);
                    jpt = true;
                }
                else{
                    servo.move_to(Jepit_Lepastable[0], 1);
                    jpt = false;
                }
            }
         }
     if ((controlSig & 16) == 16){ //L2
         servo.setTorque_Limit(1023,1);
         servo.setTorque_Limit(1023,2);
         servo.setTorque_Limit(1023,3);
     }
     if ((controlSig & 8) == 8){ //Square
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
            spdChangeFlag = us_ticker_read()/1000;
            if(!up){
                servo.move_to(Naik_Turuntable[0], 3);
                up = true;
            }
            else{
                servo.move_to(Naik_Turuntable[1], 3);
                up = false;
            }
         }
     }
     if ((controlSig & 4) == 4){ //Circle
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && (up == true || jpt == false)){
                spdChangeFlag = us_ticker_read()/1000;
                servo.move_to(CW_CCWtable[1], 2);
                circle = true;
            }
         }
    else if ((controlSig & 2) == 2){ //Triangle
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && (up == true || jpt == false)){
            spdChangeFlag = us_ticker_read()/1000;
            servo.move_to(CW_CCWtable[0], 2);
            circle = false;
         }
    }
        
    else if ((controlSig & 1) == 1){ //Cross
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && (up == true || jpt == false)){
            spdChangeFlag = us_ticker_read()/1000;
            servo.move_to(CW_CCWtable[2], 2);
            circle = false;
         }
    }
}

float getAngle(int X, int Y){
    //--------------------------------------Variable--------------------------------------
    float angle;
    //------------------------------------------------------------------------------------
    
    //--------------------------------------Algoritma--------------------------------------
    if (X==127 & Y==127){
        angle=0;
    } 
    else if (X<=255 && Y==0){
        angle = PI/4.0-(float(X)/255.0)*PI/2.0;
    }
    else if(X==0 && Y<=255){
        angle = PI/4.0+(float(Y)/255.0)*PI/2.0;
    }
    else if(X==255 && Y<=255){
        angle = PI/4.0-(PI/2.0+(float(Y)/255.0)*PI/2.0);
    }
    else if (X<=255 && Y==255){
        angle = PI/4.0+(PI/2.0+(float(X)/255.0)*PI/2.0);
    }
    return angle; 
}

int getSpeed(int X, int Y){
    int v;
    if (X==127 & Y==127){
        v=0;
    }
    else{
        v=40;
    }
    return v;
}

void computeOmega(){
    //--------------------------------------Variable--------------------------------------
    //Robot parameters
    float rWheel = 3;   //Jari jari roda
    float midX = 10;    //Jarak dari tengah robot ke roda, sumbu x
    float midY = 12.5;  //Jarak dari tengah robot ke roda, sumbu y
    float   vytot, // kecepatan maju mundur total
            vxtot, // kecepatan kiri kanan total
            vtemp, // magnitude kecepatan analog
            betot; // kecepatan rotasi total
    //------------------------------------------------------------------------------------
    
    //--------------------------------------Algoritma--------------------------------------
    vtemp = getSpeed(analog_LeftX, analog_LeftY);
    vytot = vy + vtemp*cos(angle_Left);
    vxtot = vx + vtemp*sin(angle_Left);
    betot = beta + belog;
    
    //Yang di laporan, acuan sumbu x dan y nya terbalik.
    //Logika sederhana, kalau robot mau maju nanti omega 2 dan 3 nya akan melawan (kalau ikutin rumus laporan).
    omega[0] = ((vytot-vxtot-(betot*(midX+midY)))/rWheel)/6.28*330*7/200; //640
    omega[1] = ((vytot+vxtot+(betot*(midX+midY)))/rWheel)/6.28*500*7/200; //1050
    omega[2] = ((vytot+vxtot-(betot*(midX+midY)))/rWheel)/6.28*500*7/200; //1050
    omega[3] = ((vytot-vxtot+(betot*(midX+midY)))/rWheel)/6.28*330*7/200; //640

 }
