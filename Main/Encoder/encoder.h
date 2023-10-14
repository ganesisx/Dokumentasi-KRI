//Library Encoder Ganesis - KRTMI 2021
//Referensi : Library encoder KRAI

#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"

#define PREV_MASK 0x1 //Konstanta untuk mengetahui previous direction
#define CURR_MASK 0x2 //Konstanta untuk mengetahui current direction
#define INVALID   0x3 //XORing two states where both bits have changed

class encoder{
    public :
        //Copy constructor
        encoder(const encoder &enc);
        
        //Constructor utama
        encoder(PinName channelA, PinName channelB, int pulsesPerREv);
        
        //Reset bacaan encoder
        void reset();
        
        //Dapatkan pembacaan pulsa
        int getPulses();
        
        //Dapatkan pembacaan rpm
        int getRevolutions(int samplingTimeMs);
    
    private :
        //Deteksi interrupt dan menentukan pembacaan pulsa
        void encode();
        
        //Deklarasi pin interrupt
        InterruptIn channelA_;
        InterruptIn channelB_;
        
        //Untuk copy constructor
        PinName pinA;
        PinName pinB;
        
        //Variabel untuk pengolahan data
        int pulsesPerRev_;
        int prevState_;
        int currState_;
        
        volatile int pulses_;
        volatile int revolutions_;
};

#endif