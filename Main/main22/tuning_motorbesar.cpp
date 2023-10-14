#include "mbed.h"
#include "rtos.h"
#include "Driver.h"
#include "PID.h"
#include "AX12A.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"


#define SDA PB_3
#define SCL PB_10
#define PI 3.14

//--------------kp, ki untuk maju (joystick)------------------------
double kp1m[] = {
    0.00363054, // Motor 0
    0.0035815, // Motor 1
    0.00343045, // Motor 2
    0.0037375, // Motor 3
};
double ki1m[] = {
    0.0008190, // Motor 0
    0.000819, // Motor 1
    0.0007878, // Motor 2
    0.0008639, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk mundur (joystick)----------------------
double kp1mu[] = {
    0.0050054, // Motor 0
    0.0053665, // Motor 1
    0.005, // Motor 2
    0.0044655, // Motor 3
};

double ki1mu[] = {
    0.0007900, // Motor 0
    0.000725, // Motor 1
    0.0007, // Motor 2
    0.00097939, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kanan (joystick)----------------------
double kp1k[] = {
    0.0050054, // Motor 0
    0.0053665, // Motor 1
    0.005, // Motor 2
    0.0048855, // Motor 3
};

double ki1k[] = {
    0.0007900, // Motor 0
    0.000709, // Motor 1
    0.000711, // Motor 2
    0.00076439, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kiri (joystick)----------------------
double kp1ki[] = {
    0.00551654, // Motor 0
    0.0056265, // Motor 1
    0.005362, // Motor 2
    0.0049155, // Motor 3
};

double ki1ki[] = {
    0.0007480, // Motor 0
    0.000795, // Motor 1
    0.000666, // Motor 2
    0.00077939, // Motor 3
};
//------------------------------------------------------------------

//------------kp, ki untuk mode kecepatan lambat--------------------
double kp2[] = {
    0.0058015, // Motor 0
    0.00564050, // Motor 1
    0.00563830, // Motor 2
    0.00584845, // Motor 3
};

double ki2[] = {
    0.000922, // Motor 0
    0.000929, // Motor 1
    0.000851, // Motor 2
    0.000991, // Motor 3
};
//------------------------------------------------------------------

double kd[] = {
    0.00, // Motor 0
    0.00, // Motor 1
    0.00, // Motor 2
    0.00, // Motor 3
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

PID pidsdt(0.085705,0.0008524,0,1000);  // PID sudut

encoderKRAI encMotor[]={
    encoderKRAI(PC_11, PC_10, 330),       // Motor 0
    encoderKRAI(PA_7, PA_6, 330),        // Motor 1
    encoderKRAI(PC_3, PC_2, 330),     // Motor 2
    encoderKRAI(PB_1, PB_12, 330)      // Motor 3
};

Driver motor[]={
    Driver(PA_14, PA_13, PA_15),       // Motor 0
    Driver(PC_8, PC_6, PC_9),      // Motor 1
    Driver(PC_13, PC_12, PB_7),    // Motor 2
    Driver(PB_14, PB_13, PB_15)    // Motor 3
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
// CMPS12
CMPS12_KRAI cmps(PB_9, PB_8, 0xc0);

//Objek serial servo
AX12A servo(PA_0, PA_1, PA_8, 1000000);

//----------------------------------- Global Variabel-----------------------------------
uint16_t controlSig = 0; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
//Speed variables
int pulse[4] = {0};
double omega[4] = {0};
double pwmMotor[4] = {0};
    
float vx = 0, vy = 0, beta = 0; //vx,vy, dan beta untuk eksekusi dan perhitungan
int xymode, spdmode = 1; //Mode kecepatan, default = 40 cm/s

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
//--------------------------------------Variable--------------------------------------
unsigned long spdChangeFlag = 0;

float vxMode = 33, vyMode = 33.4, betaMode = 2; //Setpoint vx, vy, dan kecepatan putar
float vxtable[] = {27, 33}; //Dalam cm/s
float vytable[] = {27, 33.4}; //Dalam cm/s
float betatable[] = {0.9, 2}; //Dalam derajat/s atau rad/s ??? Lupa

//Servo variables
float Naik_Turuntable[] = {900, 360}; //naik, turun
float CW_CCWtable[] = {204, 510, 817}; //segitiga, circle, cross
float Jepit_Lepastable[] = {778, 650}; //jepit, lepas
bool up = true, lepas = false, circle = true, cross = false;
//------------------------------------------------------------------------------------

unsigned long computeSpeedFlag;

int a = 0, aset = 0, a_prev = 0;
bool odom1 = false, odom2 = false;

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
        }
        
        //Tentukan aksi yang dilakukan
        decideCase();

        if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
            computeSpeedFlag = us_ticker_read()/1000;
            taskComputeSpeed();
            //printf("%d %d %d %d\n", spd[0], spd[1], spd[2], spd[3]);
        }

    }
}

// Functions
void taskComputeSpeed(){    
    for (int i = 0; i<4; i++){
        spd[i] = encMotor[i].getPulses()/2;
    }
    for (int i = 0; i<4; i++){     
        encMotor[i].reset();
    }
    
    omega[0] = ((vy-vx-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640
    omega[1] = ((vy+vx+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
    omega[2] = ((vy+vx-(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //1050
    omega[3] = ((vy-vx+(beta*(midX+midY)))/rWheel)/6.28*330*0.035; //640

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
            if (xymode==0){ // pid maju mundur
                pwmMotor[i] = pid1m[i].createpwm(omega[i], spd[i]);
            }
            else if (xymode==1) { //pid ke kanan dan ke kiri
                pwmMotor[i] = pid1k[i].createpwm(omega[i], spd[i]);
            }
            else if (xymode==2) {
                pwmMotor[i] = pid1mu[i].createpwm(omega[i], spd[i]);
            }
            else {
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
    }

    if ((controlSig & 8192) == 8192){ //Left
        vx = vxMode;
        xymode = 1;
    }
    else if ((controlSig & 4096) == 4096){ //Right
        vx = -vxMode;
        xymode = 3;
    }
    else{
        vx = 0;
    }

    if ((controlSig & 2048) == 2048){ //R1
        if (controlSig < 4095){
            vx = -(vxMode); 
            beta = -(vxMode+10)/(10.0+12.5);
        }
        else{
            a = cmps.getAngle()/10;
            if (xymode == 2 && !odom1 && !odom2){
                aset = a+139; a_prev = a;
                if (aset>360){
                    odom2 = true;
                    aset = aset%360;
                }
                else {
                    odom1 = true;
                    aset = aset%360;
                }
                vx = -(vxMode); vy = 0;
                beta = -(vxMode+10)/(10.0+12.5);
            }
            else if (odom1){
                if(a>=aset || a<a_prev){
                    beta = 0;
                    vy = vyMode; vx = 0;
                }
                else{
                    vx = -(vxMode); vy = 0;
                    beta = -(vxMode+10)/(10.0+12.5);
                }
            }
            else if (odom2){
                if(a>=aset && a<a_prev){
                    beta = 0;
                    vy = vyMode; vx = 0;
                }
                else{
                    vx = -(vxMode); vy = 0;
                    beta = -(vxMode+10)/(10.0+12.5);
                }
            }
            else {
                beta = -betaMode;
            }
            
        }
        xymode = 0;
    }
    else if ((controlSig & 1024) == 1024){ //L1
        if (controlSig < 4095){
            vx = (vxMode);
            beta = (vxMode + 10)/(10.0+12.5);
        }
        else{
           a = cmps.getAngle()/10;
           if (xymode == 2 && !odom1 && !odom2){
               aset = (a-139)%360; a_prev = a;
               if (aset<0){
                   aset = 360 + aset;
                   odom2 = true;
               }
               else {
                   odom1 = true;
               }
               vx = (vxMode); vy = 0;
               beta = (vxMode + 10)/(10.0+12.5); 
           }
           else if (odom1){
               if(a<=aset || a>a_prev){
                   beta = 0;
                   vy = vyMode; vx = 0;
               }
               else{
                   vx = (vxMode); vy = 0;
                   beta = (vxMode + 10)/(10.0+12.5);
               }
           }
           else if (odom2){
               if(a<=aset && a>a_prev){
                   beta = 0;
                   vy = vyMode; vx = 0;
               }
               else{
                   vx = (vxMode); vy = 0;
                   beta = (vxMode + 10)/(10.0+12.5);
               }
           }
           else {
               beta = betaMode;
           }
           
       }
        xymode = 0;
    }
    else{
        beta = 0; odom1 = false; odom2 = false; aset = a = a_prev;
    }
    
    if ((controlSig & 512) == 512){ //Start
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
            spdChangeFlag = us_ticker_read()/1000;
            //Cycle kecepatan +
            if (spdmode < 1){
                spdmode++;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
        }
    }
    else if ((controlSig & 256) == 256){ //Select
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
            spdChangeFlag = us_ticker_read()/1000;
            //Cycle kecepatan -
            if (spdmode > 0){
                spdmode--;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
        }
    }

    if ((controlSig & 32) == 32){ //R2
        if ((us_ticker_read()/1000)-spdChangeFlag >= 320){
            spdChangeFlag = us_ticker_read()/1000;
            if(!lepas && (up == true || circle == false)){
                servo.move_to(Jepit_Lepastable[1], 1);
                lepas = true;
            }
            else{
                servo.move_to(Jepit_Lepastable[0], 1);
                lepas = false;
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
            else if (!circle){
                servo.move_to(Naik_Turuntable[1], 3);
                up = false;
            }
            else if (circle && !lepas){
                servo.move_to(Naik_Turuntable[1], 3);
                up = false;
            }
        }
    }

    if ((controlSig & 4) == 4){ //Circle
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && ((up == true)&&(cross == false || lepas == false))){
            spdChangeFlag = us_ticker_read()/1000;
            servo.move_to(CW_CCWtable[1], 2);
            circle = true; cross = false;
        }
    }
    else if ((controlSig & 2) == 2){ //Triangle
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && ((up == true)&&(cross == false || lepas == false))){
            spdChangeFlag = us_ticker_read()/1000;
            servo.move_to(CW_CCWtable[0], 2);
            circle = false; cross = false;
        }
    }  
    else if ((controlSig & 1) == 1){ //Cross
        if ((us_ticker_read()/1000)-spdChangeFlag >= 250 && (up == true && lepas == false)){
            spdChangeFlag = us_ticker_read()/1000;
            servo.move_to(CW_CCWtable[2], 2);
            circle = false; cross = true;
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
        v=33;
    }
    return v;
}
