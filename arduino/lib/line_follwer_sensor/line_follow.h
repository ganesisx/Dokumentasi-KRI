//Censor buat Robot

int motor1 = 5;
int motor2 = 6;
int motor3 = 7;
int motor4 = 8;

int SENSORPins[5] = {A0, A1, A2, A3, A4};

#define FORWARD_SPEED 100
#define TURN_SPEED 50
#define ADJUST_SPEED 75

void setup() {
    // Setup motor
    pinMode(motor1, OUTPUT);
    pinMode(motor2, OUTPUT);
    pinMode(motor3, OUTPUT);
    pinMode(motor4, OUTPUT);

    //Setup sensor
    for (int i = 0; i < 5; i++){
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    //Buat baca sensor!!
    bool sensorStates[5]; //Pake bool karena HIGH and LOW
    for (int i = 0; i < 5; i++){
        sensorStates[i] = digitalRead(SENSORPins[i]);
    }

    //Sensor States!!
    if (sensorStates[2]){ //Pokoke tengah
        moveForward();
    } else if (sensorStates[1] || sensorStates[3]){
        if (sensorStates[1]){
            adjustLeft();
        }else {
            adjustRight();
        }
    }else if (sensorStates[0] || sensorStates[4]){
        if (sensprStates[4]){
            sharpRight();
        }else {
            sharpLeft();
        }
    }else {
        noMovement();
    }
}


void moveForward(){
    // Contoh menggunakan analogWrite() untuk kecepatan tertentu, ganti dengan nilai kecepatan sesuai kebutuhan
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, FORWARD_SPEED);
}

void adjustLeft(){
    analogWrite(motor1, ADJUST_SPEED);
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, ADJUST_SPEED);
    analogWrite(motor4, FORWARD_SPEED);
}

void adjustRight(){
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, ADJUST_SPEED);
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, ADJUST_SPEED);
}

void sharpLeft(){
    analogWrite(motor1, TURN_SPEED); // Misal mundur
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, TURN_SPEED); // Misal mundur
    analogWrite(motor4, FORWARD_SPEED);
}

void sharpRight(){
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, TURN_SPEED); // Misal mundur
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, TURN_SPEED); // Misal mundur
}

void noMovement(){
    // Menghentikan semua motor
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    digitalWrite(motor3, LOW);
    digitalWrite(motor4, LOW);
}
