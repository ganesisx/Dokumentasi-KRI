// encoder.h

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include "PinChangeInterrupt.h"

class Encoder {
public:
    // Copy constructor
    Encoder(const Encoder &enc);

    // Main constructor
    Encoder(int channelA, int channelB, int pulsesPerRev);

    // Reset encoder readings
    void reset();

    // Get pulse count
    int getPulses();

    // Get revolutions based on a given sampling time
    int getRevolutions(int samplingTimeMs);

    void encode();

    // Callback function for interrupt
    static void handleInterrupt();

    void poll();

    float getSpeed(int samplingTimeMs);

private:
    static Encoder* globalEncoderInstance;
    // Member variables
    int channelA_;
    int channelB_;
    int pulsesPerRev_;
    int prevState_;
    int currState_;
    int pulses_;
    int revolutions_;
    int prevRevolutions_;
};

#endif
