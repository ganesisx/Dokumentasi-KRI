#include "SharpIR.h"

SharpIR::SharpIR(PinName _pin, int _model) : irPin(_pin)
{
    model = _model;
}

float SharpIR::distance(void)
{
    float volts = irPin.read()*3.3; // Membaca nilai tegangan dari sensor
    float distance = 0;
    if (model == 430) {
        distance = (12.08 * pow(volts, -1.058))-0.42;
        if (distance < 4) { // Batasan jarak minimal
            distance = 4;
        } 
        else if (distance > 30) { // Batasan jarak maksimal
            distance = 30;
        }
    } else if (model == 1080) { 
        distance = (29.988 * pow(volts, -1.173))-0.54;
        if (distance < 4) { // Batasan jarak minimal
            distance = 10;
        } 
        else if (distance > 80) { // Batasan jarak maksimal
            distance = 80;
        }
    }
    return distance;
}