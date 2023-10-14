///Program utama robot KRTMI 2021
#include "mbed.h"
#include "Driver.h"
#include "PID.h"
#include "AX12A.h"
#include "encoderKRAI.h"

//i2c
#define SDA PB_3
#define SCL PB_10
I2CSlave slave(SDA,SCL);
const int addr = 8;
char buf[10];


//Konstanta PID, kalau salah urutan, sesuaikan aja.
//Sudah dibulatkan ke 6 desimal. Kalau kurang sesuai diubah lagi aja
//------kp, ki untuk gerak maju mundur pivot(joystick dan pc)-------
double kp1m[] = {
    0.0075505, // Motor 0
    0.0074745, // Motor 1
    0.0075104, // Motor 2
    0.0074930, // Motor 3
};

double ki1m[] = {
    0.0003524, // Motor 0
    0.0003539, // Motor 1
    0.0003481, // Motor 2
    0.0003556, // Motor 3
};
//------------------------------------------------------------------

//--------------kp, ki untuk kiri kanan (joystick)------------------
double kp1k[] = {
    0.0185705, // Motor 0
    0.0184945, // Motor 1
    0.0185004, // Motor 2
    0.0184930, // Motor 3
};

double ki1k[] = {
    0.0001024, // Motor 0
    0.0001039, // Motor 1
    0.0000981, // Motor 2
    0.0001056, // Motor 3
};
//------------------------------------------------------------------

//------------kp, ki untuk mode kecepatan lambat--------------------
double kp2[] = {
    0.0044650, // Motor 0
    0.0043545, // Motor 1
    0.0044215, // Motor 2
    0.0043430, // Motor 3
};

double ki2[] = {
    0.0000929, // Motor 0
    0.0000953, // Motor 1
    0.0000915, // Motor 2
    0.0000851, // Motor 3
};
//------------------------------------------------------------------
double kd[] = {
    0.00, // Motor 0
    0.00, // Motor 1
    0.00, // Motor 2
    0.00, // Motor 3
};

encoderKRAI encMotor[]={
    encoderKRAI(PB_12, PB_1, 330),        // Motor 0
    encoderKRAI(PC_2, PC_3, 330),      // Motor 1
    encoderKRAI(PA_6, PA_7, 330),       // Motor 2
    encoderKRAI(PC_10, PC_11, 330)     // Motor 3
};

Driver motor[]={
    Driver(PB_14, PB_13, PB_15),      // Motor 0
    Driver(PC_13, PC_12, PB_7),    // Motor 1
    Driver(PC_8, PC_6, PC_9),       // Motor 2
    Driver(PA_14, PA_13, PA_15)    // Motor 3
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

PID pid1m[] = {
    PID(kp1m[0], ki1m[0], kd[0], 1000), // Motor 0
    PID(kp1m[1], ki1m[1], kd[1], 1000), // Motor 1
    PID(kp1m[2], ki1m[2], kd[2], 1000), // Motor 2
    PID(kp1m[3], ki1m[3], kd[3], 1000)  // Motor 3
};

PID pid1k[] = {
    PID(kp1k[0], ki1k[0], kd[0], 1000), // Motor 0
    PID(kp1k[1], ki1k[1], kd[1], 1000), // Motor 1
    PID(kp1k[2], ki1k[2], kd[2], 1000), // Motor 2
    PID(kp1k[3], ki1k[3], kd[3], 1000)  // Motor 3
};

PID pid2[] = {
    PID(kp2[0], ki2[0], kd[0], 1000), // Motor 0
    PID(kp2[1], ki2[1], kd[1], 1000), // Motor 1
    PID(kp2[2], ki2[2], kd[2], 1000), // Motor 2
    PID(kp2[3], ki2[3], kd[3], 1000)  // Motor 3
};

//Robot parameters
float rWheel = 3;   //Jari jari roda
float midX = 10;    //Jarak dari tengah robot ke roda, sumbu x
float midY = 12.5;  //Jarak dari tengah robot ke roda, sumbu y

//Speed variables
int pulse[4];
long pulseTotal[4];
int spd[4];
double omega[4];
double pwmMotor[4];
float dist[4];

//Communication variables
float pivot=3.5;
uint16_t controlSig = 0; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
char MSB[1]={0}, LSB[1]={0};
bool executed; //Menandakan bahwa sinyal joystick sudah dieksekusi
float vxMode = 40, vyMode = 40, betaMode = 1.8; //Setpoint vx, vy, dan kecepatan putar
float vx, vy, beta; //vx,vy, dan beta untuk eksekusi dan perhitungan
int xymode, spdmode = 1; //Mode kecepatan, default = 40 cm/s
float vxtable[] = {26, 40}; //Dalam cm/s
float vytable[] = {13, 38}; //Dalam cm/s
float betatable[] = {0.5, 1.8}; //Dalam derajat/s atau rad/s ??? Lupa
unsigned long spdChangeFlag, tuningChangeFlag, //Untuk debounce
              computeSpeedFlag;
float x,y,teta;
int highlow, mode=0;

//Servo variables
int Naik_Turunmode = 0; //default posisi di Naik_Turuntable[0] 
int CW_CCWmode = 0; //default posisi di CW_CCWtable[1]
int Jepit_Lepasmode = 0;
float Naik_Turuntable[] = {10, 420}; //dalam bit (0-1023)
float CW_CCWtable[] = {52, 359, 666, 973}; //dalam bit (0-1023)
float Jepit_Lepastable[] = {15, 165};

//Objek serial
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 9600);
//Kalau joystick bermasalah, bisa diubah jadi unbuffered serial dengan beberapa penyesuaian
BufferedSerial joystick(PA_9, PA_10, 9600);
//Serial ESP32
//BufferedSerial esp(PB_10, PC_5, 1000000);
//Objek serial servo
AX12A servo(PA_0, PA_1, PA_8, 1000000);


void i2cReceive();          //Komunikasi i2c
void taskComputeSpeed();    //Menghitung PID
void getJoyIn();            //Mendapatkan input joystick
void computeOmega();        //Menghitung omega setiap motor
void decideCase();          //Ubah kondisi sesuai kasus input joystick
void distance();            //Menghitung jarak tempuh
void getESPIn();

int main(){
    //inisial setting servo
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 1);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 1);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 2);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 2);
    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 3);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 3);
    servo.setSpeed(600, 2);
    servo.setSpeed(300, 3);
    servo.setSpeed(600, 1);

    //i2c
    slave.address(addr);

    while (1){
        //Get joystick input
        if (joystick.readable()){
            joystick.read(MSB,1);
            joystick.read(LSB,1);
            controlSig = ((int)*MSB<<8|(int)*LSB);
        }
        
//        printf("%d\n", mode);
        //getESPIn();
//        
//        if (!mode){
            //Tentukan aksi yang dilakukan
        decideCase();
        //}
//        //getJoyIn();
//        else{
//            beta = (int(buf[1])-18)/10;
//            vy = int(buf[0]);
//            vx = 0;
//            printf("%d %d\n", int(buf[0]), int(buf[1]) );
//        }
        
        //Read pulse, bisa ditaro dalam fungsi aja
        for (int i = 0; i<4; i++){
            pulse[i] = encMotor[i].getPulses();
        }

        if ((us_ticker_read()/1000)-computeSpeedFlag >= 35){
            computeSpeedFlag = us_ticker_read()/1000;
            taskComputeSpeed();
        }

        //Run Motor, bisa ditaro dalam fungsi aja.
        //Kalau decide case bilang vx = vy = beta = 0, maka motor pasti tidak akan jalan

        for (int i = 0; i<4; i++){
            motor[i].Motion(pwmMotor[i]);
        }
        
    }
}

//void getESPIn(){
//    if (esp.readable()){
//        esp.read(buf,2);
////        printf("From ESP : %s\n", buf);
//    }
//}


// Functions
void taskComputeSpeed(){
    //Ambil semua kecepatan
    for (int i = 0; i<4; i++){
        spd[i] = pulse[i]/2;
    }

    //Ubah kecepatan target ke omega
    computeOmega();

    //Hitung semua pwm dengan PID
    for (int i = 0; i<4; i++){
//        if (!mode){
            if (spdmode == 1){ // mode gerak cepat
                if (xymode==0){ // pid maju mundur
                    pwmMotor[i] = pid1m[i].createpwm(omega[i], spd[i]);
                }
                else { //pid ke kanan dan ke kiri
                    pwmMotor[i] = pid1k[i].createpwm(omega[i], spd[i]);
                }
            }
            else { // mode gerak lambat
                pwmMotor[i] = pid2[i].createpwm(omega[i], spd[i]);
            }
//        }
        //    
//        else{ // mode otomatis
//            pwmMotor[i] = pid1m[i].createpwm(omega[i], spd[i]);
//        }
    }
    
    //Reset semua bacaan
    for (int i = 0; i<4; i++){     
        encMotor[i].reset();
    }
    
 }

//i2c
void i2cReceive(){
    int i = slave.receive();
//    printf("Data received : %d\n", i);
    switch (i) {
        case I2CSlave::WriteAddressed:
            slave.read(buf, 5);
//            printf("Read A: %s\n", buf);
//            printf("X1 : %d\n", int(buf[2]));
//            printf("X2 : %d\n", int(buf[3]));
            break;
    }
}
 
 void decideCase(){
     if ((controlSig & 32768) == 32768){
         vy = vyMode;
         xymode = 0;
         mode = 0;
         }
     else if ((controlSig & 16384) == 16384){
         vy = -vyMode;
         xymode = 0;
         mode = 0;
         }
     else{
         vy = 0;
         xymode = 0;
         mode = 0;
         }
     if ((controlSig & 8192) == 8192){
         vx = vxMode;
         xymode = 1;
         mode = 0;
         }
     else if ((controlSig & 4096) == 4096){
         vx = -vxMode;
         xymode = 1;
         mode = 0;
         }
     else{
         vx = 0;
         xymode = 1;
         mode = 0;
         }
     if ((controlSig & 2048) == 2048){
         beta = -betaMode;
         xymode = 0;
         mode = 0;
         }
     else if ((controlSig & 1024) == 1024){
         beta = betaMode;
         xymode = 0;
         mode = 0;
         }
     else{
         beta = 0;
         xymode = 0;
         mode = 0;
         }
     if ((controlSig & 512) == 512){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                //Cycle kecepatan +
                if (spdmode < 1) spdmode++;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
         mode = 0;
         }
     else if ((controlSig & 256) == 256){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                //Cycle kecepatan -
                if (spdmode > 0) spdmode--;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
         mode = 0;
         }
       //  
     if ((controlSig & 128) == 128){
          // mode joystick
          beta = -pivot;
          mode = 1;}
     else if ((controlSig & 64) == 64){
          // mode pc
          mode = 1;
          beta = pivot;
          }
             
     if ((controlSig & 32) == 32){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 120){
                spdChangeFlag = us_ticker_read()/1000;
                if (Jepit_Lepasmode<1) {
                    Jepit_Lepasmode++;
                    servo.move_to(Jepit_Lepastable[Jepit_Lepasmode], 3);
                }
            }
         mode = 0;
         }
     else if ((controlSig & 16) == 16){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 120){
                spdChangeFlag = us_ticker_read()/1000;
                if (Jepit_Lepasmode>0) {
                    Jepit_Lepasmode--;
                    servo.move_to(Jepit_Lepastable[Jepit_Lepasmode], 3);
                }
            }
         mode = 0;
         }
     if ((controlSig & 8) == 8){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (CW_CCWmode > 0) {
                    CW_CCWmode--;
                    servo.move_to(CW_CCWtable[CW_CCWmode], 2);
                }
            }
         mode = 0;
         }
     else if ((controlSig & 4) == 4){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (CW_CCWmode < 3) {
                    CW_CCWmode++;
                    servo.move_to(CW_CCWtable[CW_CCWmode], 2);
                }
            }
         mode = 0;
         }
     if ((controlSig & 2) == 2){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (Naik_Turunmode <1) {
                    Naik_Turunmode++;
                    servo.move_to(Naik_Turuntable[Naik_Turunmode], 1);
                }
            }
         mode = 0;
         }
     else if ((controlSig & 1) == 1){
         if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000; 
                if (Naik_Turunmode>0) {
                    Naik_Turunmode--;
                    servo.move_to(Naik_Turuntable[Naik_Turunmode], 1);
                }
            }
         mode = 0;
         }
}

 void computeOmega(){
    //Yang di laporan, acuan sumbu x dan y nya terbalik.
    //Logika sederhana, kalau robot mau maju nanti omega 2 dan 3 nya akan melawan (kalau ikutin rumus laporan).
    omega[0] = ((vy-vx-(beta*(midX+midY)))/rWheel)/6.28*330*7/200;
    omega[1] = ((vy+vx+(beta*(midX+midY)))/rWheel)/6.28*330*7/200;
    omega[2] = ((vy+vx-(beta*(midX+midY)))/rWheel)/6.28*330*7/200;
    omega[3] = ((vy-vx+(beta*(midX+midY)))/rWheel)/6.28*330*7/200;
    for (int i=0; i<4; i++){
        if (mode == 0){
            if (omega[i]>=22){
                omega[i]=22;
            }
            else if(omega[i]<=-22){
                omega[i]=-22;
            }
        }
    }
 }