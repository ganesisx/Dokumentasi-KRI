#include "mbed.h"
#include "SharpIR.h"

SharpIR sensor1(PB_0, 430); // Inisialisasi sensor

int main() {
    while(1) {
        float distance1 = sensor1.distance(); // Membaca nilai jarak dari sensor
        printf("Distance: %d cm\n", (int)distance1); // Menampilkan nilai jarak pada serial monitor
        ThisThread::sleep_for(100ms); // Delay selama 100ms 
    }
}