#ifndef SharpIR_h
#define SharpIR_h

#include "mbed.h"

class SharpIR {
    public:
        SharpIR(PinName _pin, int _model); // Konstruktor
        float distance(); // Fungsi untuk membaca nilai jarak dari sensor
    private:
        AnalogIn irPin; // Objek AnalogIn untuk membaca nilai analog dari sensor
        int model; // Tipe/model sensor Sharp IR
};

#endif