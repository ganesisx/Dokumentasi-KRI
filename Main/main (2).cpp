#include "mbed.h"
#include "AX12A.h"

BufferedSerial pc(USBTX, USBRX, 57600);
BufferedSerial joystick(PA_9, PA_10, 115200);
AX12A servo(PA_0, PA_1, PA_8, 1000000);

//Robot parameters
float rWheel = 3;   //Jari jari roda
float midX = 10;    //Jarak dari tengah robot ke roda, sumbu x
float midY = 12.5;  //Jarak dari tengah robot ke roda, sumbu y

//Speed variables
int pulse[4];
int spd[4];
int omega[4];
double pwmMotor[4];

//Communication variables
char controlSig[1] = {0}; //Buffer untuk menyimpan input dari joystick yang dikirim arduino
bool executed; //Menandakan bahwa sinyal joystick sudah dieksekusi
float vxMode, vyMode, betaMode; //Setpoint vx, vy, dan kecepatan putar
float vx, vy, beta; //vx,vy, dan beta untuk eksekusi dan perhitungan
int spdmode = 2; //Mode kecepatan, default = 40 cm/s
float vxtable[] = {13, 26, 40}; //Dalam cm/s
float vytable[] = {13, 26, 40}; //Dalam cm/s
float betatable[] = {13, 26, 40}; //Dalam derajat/s atau rad/s ??? Lupa
unsigned long spdChangeFlag; //Untuk debounce

//servo variables
int jepit = 0;
int Naik_Turunmode = 0; //default posisi di 0
int CW_CCWmode = 1; //default sudut di 0
float Naik_Turuntable[] = {0, 130}; //dalam cm
float CW_CCWtable[] = {82, 392, 699, 1003}; //dalam deg


//RTOS
Thread thread;

uint16_t rotate(float jarak);
uint16_t rotasi(float sudut);

void getJoyIn();            //Mendapatkan input joystick

void decideCase();          //Ubah kondisi sesuai kasus input joystick

int main()
{
    char status[1] = {0};

    //initial setting servo
    // servo.setAngleLimit(rotasi(0), ADDRESS_CW_ANGLE_LIMIT, 1);
    // servo.setAngleLimit(rotasi(195), ADDRESS_CCW_ANGLE_LIMIT, 1);  //195 karena gigi terdikit ada 13

    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 1);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 1);

    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 2);
    servo.setAngleLimit(1023, ADDRESS_CCW_ANGLE_LIMIT, 2);
    servo.setSpeed(512, 2);
    servo.move_to(392, 2);


    servo.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 3);
    servo.setAngleLimit(230, ADDRESS_CCW_ANGLE_LIMIT, 3);

    while(1){
        // printf("%d\n", CW_CCWmode);
        // servo.setSpeed(600, 2);
        // servo.move_to(rotate(CW_CCWtable[3]), 2);
        //motor2.toggleLED(1);
        //pc.write(status, 1);
        //ThisThread::sleep_for(200ms);
        //motor2.toggleLED(0);
        //ThisThread::sleep_for(200ms);
        //printf("1.%s\n", status);
        // motor2.setAngleLimit(0, ADDRESS_CW_ANGLE_LIMIT, 2);
        // motor2.setAngleLimit(0, ADDRESS_CCW_ANGLE_LIMIT, 2);
        // motor2.setSpeed(300, 2);
        // motor2.move_to(0);
        // ThisThread::sleep_for(2000ms);
        // motor2.setSpeed(1500);
        // ThisThread::sleep_for(4000ms);
        // motor2.setSpeed(0);
        // motor2.setID(3);
         //Get joystick input
        getJoyIn();

        //Tentukan aksi yang dilakukan
        decideCase();
    }
}

//Bisa dihilangin kalau misalnya error baca joystick
FileHandle *mbed::mbed_override_console(int fd)
{
    return &pc;
}

void getJoyIn(){
    if (joystick.readable()){
        joystick.read(controlSig,1);
        //Menandakan bahwa controlSig sudah berubah, namun belum dieksekusi.
        //Diperlukan agar kalau joystick idle, nilai controlSig yang sama
        //tidak akan dieksekusi.
        executed = 0;
    }
    else {
        servo.setSpeed(0, 254); //broadcast id
    }
}
 
void decideCase(){
    switch (*controlSig) {
        case '1':
            //Gripper naik
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (Naik_Turunmode <1) Naik_Turunmode++;
                servo.setSpeed(600, 1);
                servo.move_to(Naik_Turuntable[Naik_Turunmode], 1);
            }
        break;

        case '3':
            //Gripper turun
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000; 
                if (Naik_Turunmode>0) Naik_Turunmode--;
                servo.setSpeed(600, 1);
                servo.move_to(Naik_Turuntable[Naik_Turunmode], 1);
            }
        break;

        case '2':
            //Gripper putar CW
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (CW_CCWmode < 3) CW_CCWmode++;
                servo.setSpeed(600, 2);
                servo.move_to(CW_CCWtable[CW_CCWmode], 2);
            }
        break;

        case '4':
            //Gripper putar CCW
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read()/1000;
                if (CW_CCWmode > 0) CW_CCWmode--;
                servo.setSpeed(600, 2);
                servo.move_to(CW_CCWtable[CW_CCWmode], 2);
            }
        break;
        
        case '8': //Cycle kecepatan +
            //Debounce
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read();
                //Cycle kecepatan +
                if (spdmode < 3) spdmode++;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
        break;

        case 'h': //Cycle kecepatan -
            //Debounce
            if ((us_ticker_read()/1000)-spdChangeFlag >= 250){
                spdChangeFlag = us_ticker_read();
                //Cycle kecepatan +
                if (spdmode > 0) spdmode--;
                vxMode = vxtable[spdmode];
                vyMode = vytable[spdmode];
                betaMode = betatable[spdmode];
            }
        break;

        case 'a':
            //Translasi kiri
            vx = -vxMode;
            vy = 0;
            beta = 0;
        break;

        case 'b':
            //Translasi kanan
            vx = vxMode;
            vy = 0;
            beta = 0;
        break;

        case 'c':
            //Maju
            vx = 0;
            vy = vyMode;
            beta = 0;
        break;

        case 'd':
            //Mundur
            vx = 0;
            vy = -vyMode;
            beta = 0;
        break;

        case '5':
            //Putar robot CW
            vx = 0;
            vy = 0;
            beta = betaMode;
        break;

        case 'e':
            //Putar robot CCW
            vx = 0;
            vy = 0;
            beta = -betaMode;
        break;

        case '6':
            //Gripper jepit
            if ((us_ticker_read()/1000)-spdChangeFlag >= 120){
                spdChangeFlag = us_ticker_read()/1000;
                if (jepit <= 230) {jepit = jepit + 10;
                servo.setSpeed(1324, 3);
                servo.move_to(jepit, 3);}
            }
        break;
        
        case 'f':
            //Gripper lepas
            if ((us_ticker_read()/1000)-spdChangeFlag >= 120){
                spdChangeFlag = us_ticker_read()/1000;
                if (jepit >= 0) {jepit = jepit - 10;
                servo.setSpeed(300, 3);
                servo.move_to(jepit, 3);}
            }
        break;

        case '7':
            //Masih kosong

        break;

        case 'g':
            //Masih kosong

        break;

        default: //Selain kasus di atas, jangan gerakkan apa apa
            vx = 0;
            vy = 0;
            beta = 0;
            servo.setSpeed(0, 254);
    }
    *controlSig = '0';
 }

uint16_t rotate(float jarak)
{
    return round(jarak*1023/(5.23*rWheel));
}

uint16_t rotasi(float sudut)
{
    return round(sudut*1023/(300));
}