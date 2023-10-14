#include "mbed.h"
#include "encoder.h"

encoder::encoder(const encoder &enc) : channelA_(enc.pinA), channelB_(enc.pinB){
    
    pulsesPerRev_ = enc.pulsesPerRev_;
    prevState_ = enc.prevState_;
    currState_ = enc.currState_;

    pulses_ = enc.pulses_;
    revolutions_ = enc.revolutions_;
}


encoder::encoder(PinName channelA, PinName channelB, int pulsesPerRev):
    channelA_(channelA), channelB_(channelB){

    pinA = channelA;
    pinB = channelB;
    pulses_       = 0;
    revolutions_  = 0;
    pulsesPerRev_ = pulsesPerRev;

    //Workout what the current state is.
    int chanA = channelA_.read();
    int chanB = channelB_.read();

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);
    prevState_ = currState_;

    //X2 encoding uses interrupts on only channel A.
    channelA_.rise(callback(this, &encoder::encode));
}


void encoder::reset(){
    pulses_      = 0;
    revolutions_ = 0;
}


int encoder::getPulses(){
    return pulses_;
}


int encoder::getRevolutions(int samplingTimeMs){
    revolutions_ = (pulses_*60*1000/samplingTimeMs)/pulsesPerRev_;
    return revolutions_;
}


void encoder::encode(){
    int change = 0;
    int chanA  = channelA_.read();
    int chanB  = channelB_.read();
    printf("Triggered\n");

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);

    //11->00->11->00 is counter clockwise rotation or "forward".
    if ((prevState_ == 0x3 && currState_ == 0x0) || (prevState_ == 0x0 && currState_ == 0x3)) {
        pulses_++;
    }
    //10->01->10->01 is clockwise rotation or "backward".
    else if ((prevState_ == 0x2 && currState_ == 0x1) || (prevState_ == 0x1 && currState_ == 0x2)) {
        pulses_--;
    }
    
    prevState_ = currState_;
}