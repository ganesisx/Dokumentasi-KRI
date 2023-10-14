#include "mbed.h"
#include "Driver.h"
#include "PID.h"
#include "AX12A.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include <stdlib.h>

#define SDA PB_3
#define SCL PB_10
#define PI 3.14
#define PULSE_TO_CM 0.0285    // 2PI*rwheel/PPR/2
#define Ts 35
#define MAX_SPEED 90//Vresultan max (cm/s)
#define MAX_W_SPEED 90 //Vw max (cm/s)
#define SPEED 1
#define L 16.0           // lengan roda dari pusat robot (cm)


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
//Objek serial servo
AX12A servo(PA_0, PA_1, PA_8, 1000000);
// CMPS12
CMPS12_KRAI cmps(PB_9, PB_8, 0xc0);

//----------------------------------- Global Variabel-----------------------------------
uint16_t controlSig = 0; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
//Speed variables
int pulse[4] = {0};
double omega[4] = {0};
double pwmMotor[4] = {0};
float belog=0; // kecepatan rotasi analog

int analog_LeftX=127, analog_LeftY=127, 
    analog_RightX=127, analog_RightY=127; // inisialisasi analog joystick
    
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
void track(); 


float spd[4] = {0};
float d[4] = {0};
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


//------------------------------------odom--------------------------------------------
int aset = 0, a_offset = 0;
float  cx = 20, cy = 20;
int ca = 1, sum_a = 0, a_prev = 0, a = 0, a_error_prev = 0;
float sum_x = 0, sum_y = 0;
float x = 0, y = 0, x_prev = 0, y_prev = 0, vx = 0, vy = 0, vr = 0, vw = 0, ar = 0, vs = 0, w = 0;
float S_error = 0, x_error = 0, y_error = 0, a_error = 0, sum_S_error = 0, sum_x_error = 0, sum_y_error = 0, sum_a_error = 0, S_error_prev = 0, x_error_prev = 0, y_error_prev = 0;
bool linear_rotation = false, linear = false, rotation = false;
bool mup = false, mdown = false, mright = false, mleft = false, mrot_cw = false, mrot_ccw = false;

// konstanta PID untuk kendali Posisi (x y)
double Kp_s = 0.5;
double Ki_s = 0.0;
double Kd_s = 0.0002;

// konstanta PID untuk kendali arah (theta)
double Kp_w = 0.3;
double Ki_w = 0.0;
float Kd_w = 0.0;
//-------------------------------------------------------------------------------

int i = 0;  

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
    ThisThread::sleep_for(100ms); 
    servo.move_to(Naik_Turuntable[0], 3);
    ThisThread::sleep_for(100ms); 
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

        track();

        //Tentukan aksi yang dilakukan
        decideCase();

        if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
            angle_Left = getAngle(analog_LeftX, analog_LeftY);
            taskComputeSpeed();
            //printf("%f %f %f %f\n", pwmMotor[0], pwmMotor[1], pwmMotor[2], pwmMotor[3]);
            //printf("%f\t%f\t%f\t%f\n", d[0],d[1] ,d[2],d[3]);
            // printf("%f %f %f %f %f %f\n", sum_y, y ,sum_y_error, y_error, dy, y_prev);
            computeSpeedFlag = us_ticker_read()/1000;
        }
    }
}


float moving_direction( float xs, float ys, float x, float y,float theta){
    float temp = atan((ys - y)/(xs - x)) - theta;
    
    if (xs < x)    return temp + PI;
    else            return temp;    
}

void track(){
    for (int i = 0; i<4; i++){
        spd[i] = encMotor[i].getPulses();
        d[i] = PULSE_TO_CM*spd[i];
    }

    a = cmps.getAngle()/10;
    y = y_prev + (d[0]+d[1]+d[2]+d[3])/4.0;
    x = x_prev + (-d[0]+d[1]+d[2]-d[3])/4.0;
    
    
    x_prev = x;
    y_prev = y;

    for (int i = 0; i<4; i++){     
        encMotor[i].reset();
    }
}

void taskComputeSpeed(){
    x_error = sum_x - x;
    y_error = sum_y - y;
    a_error = sum_a - a;

    if(a_error >= 90 && mrot_cw) {
        a_error = a_error - 360;
    }
    else if(a_error <= -90 && mrot_ccw) {
        a_error = a_error + 360;
    }

    sum_x_error += x_error;
    sum_y_error += y_error;
    sum_a_error += a_error;

    //vs = Kp_s*S_error + Ki_s*Ts*sum_S_error + Kd_s*(S_error - S_error_prev)/Ts;
    vx = Kp_s*x_error + Ki_s*Ts*sum_x_error + Kd_s*(x_error - x_error_prev)/Ts;
    vy = Kp_s*y_error + Ki_s*Ts*sum_y_error + Kd_s*(y_error - y_error_prev)/Ts;
    w = Kp_w*a_error + Ki_w*Ts*sum_a_error + Kd_w*(a_error - a_error_prev)/Ts;

    //S_error_prev = S_error;
    x_error_prev = x_error;
    y_error_prev = y_error;
    a_error_prev = a_error;

    // vr = vs/MAX_SPEED/0.035;
    vx = (vx/MAX_SPEED)/0.035;
    vy = (vy/MAX_SPEED)/0.035;
    vw = (w*L/MAX_W_SPEED)/0.035;

    // ar = moving_direction(sum_x, sum_y, x, y, a-a_offset);

    // vx = 0;//vr*cos(ar);
    // vy = vr;//*sin(ar);

    //vw = 0;

    pwmMotor[0] = (vy-vx-vw)/(2*PI*rWheel); //640
    pwmMotor[1] = (vy+vx+vw)/(2*PI*rWheel); //1050
    pwmMotor[2] = (vy+vx-vw)/(2*PI*rWheel); //1050
    pwmMotor[3] = (vy-vx+vw)/(2*PI*rWheel); //640

    for (int i = 0; i<4; i++){
        if(pwmMotor[i]>0.1){
            pwmMotor[i] = 0.1;
        }
        else if(pwmMotor[i]<-0.1){
            pwmMotor[i] = -0.1;
        }
        motor[i].Motion(pwmMotor[i]);
    }

}
 
void decideCase(){
    if ((controlSig & 32768) == 32768){ //Up (controlSig & 32768) == 32768
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mup){
                //sum_y = sum_y + cy; //cum setting
            }
            else {
                mup = true; linear = true;
                /*sum_y = cy;*/ sum_y = 60; y = y_prev = 0; sum_x = x = x_prev = 0; sum_a = a = cmps.getAngle()/10;
            }
        }
    }
    else if ((controlSig & 16384) == 16384){ //Down (controlSig & 16384) == 16384
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mdown){
                //sum_y = sum_y - cy;
            }
            else {
                mdown = true; linear = true;
                /*sum_y = -cy*/ sum_y = -60; y = y_prev = 0; sum_x = x = x_prev = 0; sum_a = a = cmps.getAngle()/10;
            }
        }
    }
    else {
        sum_y_error = y = y_prev = sum_y = 0; mup = false; mdown = false;
    }

    if ((controlSig & 8192) == 8192){ //Left (controlSig & 8192) == 8192
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mleft){
                //sum_x = sum_x + cx;
            }
            else {
                mleft = true; linear = true;
                /*sum_x = cx;*/ sum_x = 60; x = x_prev =  0; sum_y = y = y_prev = 0; sum_a = a = cmps.getAngle()/10;
            }
        }
    }
    else if ((controlSig & 4096) == 4096){ //Right (controlSig & 4096) == 4096
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mright){
                sum_x = sum_x - cx;
            }
            else {
                mright = true; linear = true;
                sum_x = -cx; x = x_prev =  0; sum_y = y = y_prev = 0; sum_a = a = cmps.getAngle()/10;
            }
        }
    }
    else{
        sum_x_error = x = x_prev = sum_x = 0; mright = false; mleft = false;
    }

    if ((controlSig & 2048) == 2048){ //R1
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mrot_cw && !linear){
                sum_a = (sum_a - ca)%360;
            }
            else {
                rotation = true; mrot_cw = true;
                sum_x = x = 0; sum_y = y = 0; a = cmps.getAngle()/10; sum_a_error = 0;
                if(!linear){
                    sum_a = (a - ca)%360;
                }
                else if (mup) {
                    sum_a = a;
                }
                else if (mdown){
                    sum_a = (a - 180)%360;
                }
                else if (mright){
                    sum_a = (a - 90)%360; 
                }
                else if (mleft) {
                    sum_a = (a + 90)%360;
                }
            }  
        }
    }  
    else if ((controlSig & 1024) == 1024){ //L1
        if ((us_ticker_read()/1000)-spdChangeFlag >= 35){
            spdChangeFlag = us_ticker_read()/1000;
            if (mrot_ccw && !linear){
                sum_a = (sum_a + ca)%360;
            }
            else {
                rotation = true; mrot_ccw = true;
                sum_x = x = 0; sum_y = y = 0; a = cmps.getAngle()/10; sum_a_error = 0;
                if(!linear){
                    sum_a = (a + ca)%360;
                }
                else if (mup) {
                    sum_a = a;
                }
                else if (mdown){
                    sum_a = (a + 180)%360;
                }
                else if (mright){
                    sum_a = (a + 90)%360; 
                }
                else if (mleft) {
                    sum_a = (a - 90)%360;
                }
            }
        }
    }
    else{
        sum_a_error = 0; sum_a = a = cmps.getAngle()/10; mrot_cw = false; mrot_ccw = false; rotation = false;
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
