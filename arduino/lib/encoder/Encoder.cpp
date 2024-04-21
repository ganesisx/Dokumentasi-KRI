#include "Encoder.h"

// Static pointer to an instance of the Encoder class
Encoder* Encoder::globalEncoderInstance = nullptr;

// Constructor
Encoder::Encoder(int channelA, int channelB, int pulsesPerRev)
  : channelA_(channelA), channelB_(channelB) {
  pulses_ = 0;
  revolutions_ = 0;
  pulsesPerRev_ = pulsesPerRev;

  // Determine the current state
  int chanA = digitalRead(channelA_);
  int chanB = digitalRead(channelB_);
  currState_ = (chanA << 1) | chanB;
  prevState_ = currState_;

  // Set up interrupts for X2 encoding on only channel A
  attachPCINT(digitalPinToPCINT(channelA_), handleInterrupt, RISING);

  // Set the global instance to this object
  globalEncoderInstance = this;
}

// Copy constructor
Encoder::Encoder(const Encoder& enc)
  : channelA_(enc.channelA_), channelB_(enc.channelB_) {
  pulsesPerRev_ = enc.pulsesPerRev_;
  prevState_ = enc.prevState_;
  currState_ = enc.currState_;
  pulses_ = enc.pulses_;
  revolutions_ = enc.revolutions_;
}

// Reset function
void Encoder::reset() {
  pulses_ = 0;
  revolutions_ = 0;
}

// Get pulse count function
int Encoder::getPulses() {
  return pulses_;
}

// Get revolutions function
int Encoder::getRevolutions(int samplingTimeMs) {
  revolutions_ = (pulses_ * 60 * 1000 / samplingTimeMs) / pulsesPerRev_;
  // Serial.println(pulses_);
  return revolutions_;
}

float Encoder::getSpeed(int samplingTimeMs) {
  int deltaRevolutions = getRevolutions(samplingTimeMs) - prevRevolutions_;
  int temp = getRevolutions(samplingTimeMs);
  float speed = static_cast<float>(deltaRevolutions) / (abs(samplingTimeMs) / 1000.0);
  prevRevolutions_ = getRevolutions(samplingTimeMs);
  return speed;
}

// Static callback function for interrupt
void Encoder::handleInterrupt() {
  // Call the non-static member function using the global instance
  if (globalEncoderInstance) {
    globalEncoderInstance->encode();
  }
}

// Encode function
void Encoder::encode() {
  int chanA = digitalRead(channelA_);
  int chanB = digitalRead(channelB_);
  // Serial.println(chanA);
  // Serial.println("A/B");
  // Serial.println(chanB);
  // 2-bit state.
  currState_ = (chanA << 1) | chanB;
  // Serial.println(currState_);
  // Serial.println("A/B");
  // Serial.println(prevState_);
  // 11->00->11->00 is counter clockwise rotation or "forward".
  if ((prevState_ == 0x3 && currState_ == 0x3) || (prevState_ == 0x3 && currState_ == 0x3)) {
    pulses_++;
  }
  // 10->01->10->01 is clockwise rotation or "backward".
  else if ((prevState_ == 0x2 && currState_ == 0x2) || (prevState_ == 0x2 && currState_ == 0x2)) {
    pulses_--;
  }
  // else{
  //     pulses_++;
  // }

  prevState_ = currState_;
}
