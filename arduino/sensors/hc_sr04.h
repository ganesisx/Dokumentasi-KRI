#include <Aruino.h>
#include <HCSR04.h>

//Defining the trigger pin 
#define TRIGGER_PIN_RIGHT 28
#define TRIGGER_PIN_LEFT 47
#define ECHO_PIN_LEFT 45
#define ECHO_PIN_RIGHT 24

//Threshold
#define THRESHOLD_DISTANCE 4.0

class Sensor{
  private:
    uint8_t triggerPin;
    uint8_t echoPin;
    volatile unsigned long pulseStart;
    volatile unsigned long pulseEnd;
    volatile bool pulseFinished;
    double thresholdDistance;

  public:
    //Initialize the sensors
    Sensor(uint8_t trigPin, uint8_t echoPin, double threshold) : triggerPin(trigPin), echoPin(echoPin), thresholdDistance(threshold), pulseStart(0), pulseEnd(0), pulseFinished(false){
      pinMode(triggerPin, OUTPUT)
      pinMode (echoPin, INPUT)
      attachInterupt(digitalPinToInterrupt(echoPin), [this]() { this->echoReceived(); }, CHANGE);
    }
    //Fungsi buat ngukur
    void triggerPulse() {
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(2);
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
    }
    //Interupting service
    void echoReceived(){
      unsigned long currentTime = micros();
      if (digitalRead(echoPin) == HIGH){
        pulseStart = currentTime;
      }else if (digitalRead(echoPin) == LOW):
        pulseEnd = currentTime;
        pulseFinished = true;
    }
    //Near the table
    bool isNearTable(){
        if (pulseFinished) {
            double distance = (pulseEnd - pulseStart) * 0.0343 / 2.0;
            Serial.print("Distance: ");
            Serial.println(distance);
            pulseFinished = false;  // Reset for next measurement
            if (distance <= thresholdDistance && distance >= 0) {
                Serial.println("The object is near.");
                return true;
            }else{
              Serial.println("Objeknya gak deket")
            }     
          }
          return false;
    }
    // Call this often to ensure sensor responsiveness
    void update() {
        if (!pulseFinished) {
            triggerPulse();
        }
    }
};
// Mendefinisikan sensor buat kanan dan kiri
Sensor sensorRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, THRESHOLD_DISTANCE);
Sensor sensorLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, THRESHOLD_DISTANCE);

void setup(){
  Serial.begin(9600);
}

void loop() {
    // Updating terus bacaan sensor
    sensorRight.update();
    sensorLeft.update();

    // Memeriksa apakah sensor masuk threshold kanan
    if (sensorRight.isNearTable()) {
        Serial.println("Object is near the right sensor.");
    } else {
        Serial.println("Object is not near the right sensor.");
    }

    // Memeriksa apakah sensor masuk threshold kiri
    if (sensorLeft.isNearTable()) {
        Serial.println("Object is near the left sensor.");
    } else {
        Serial.println("Object is not near the left sensor.");
    }

    delay(100); // Reduced delay to increase responsiveness
}
