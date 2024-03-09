#define STOP_SPEED 0 // Speed to stop
#define FORWARD_SPEED 255 // Max speed for maju
#define BACKWARD_SPEED 255 // Max speed for mundur (same as forward but reverse direction)
#define TURN_SPEED 128    // Speed for belok
#define ADJUST_SPEED 192  // Speed for slight adjustments

int motor1 = 5; // Front-Left Wheel
int motor2 = 6; // Front-Right Wheel
int motor3 = 7; // Back-Right Wheel
int motor4 = 8; // Back-Left Wheel

int sensorPins[5] = {A0, A1, A2, A3, A4};

void setup() {
    pinMode(motor1, OUTPUT);
    pinMode(motor2, OUTPUT);
    pinMode(motor3, OUTPUT);
    pinMode(motor4, OUTPUT);

    for (int i = 0; i < 5; i++){
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    bool sensorStates[5];
    for (int i = 0; i < 5; i++){
        sensorStates[i] = digitalRead(sensorPins[i]);
    }

    // Example logic for movement based on a single sensor input
    if (sensorStates[2]){ // Forward
        moveForward();
    } else if (sensorStates[1]){ // Rotate Left
        rotateLeft();
    } else if (sensorStates[3]){ // Rotate Right
        rotateRight();
    } else if (sensorStates[0]){ // Move Left
        moveLeft();
    } else if (sensorStates[4]){ // Move Right
        moveRight();
    } else {
        noMovement();
    }
}

void moveForward(){
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, FORWARD_SPEED);
}

void moveLeft(){
    // For lateral left movement, reverse the rollers on one side and forward on the other
    analogWrite(motor1, BACKWARD_SPEED);
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, BACKWARD_SPEED);
    analogWrite(motor4, FORWARD_SPEED);
}

void moveRight(){
    // For lateral right movement, reverse the above logic
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, BACKWARD_SPEED);
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, BACKWARD_SPEED);
}

void rotateLeft(){
    // To rotate left, the left wheels go backward, the right wheels go forward
    analogWrite(motor1, BACKWARD_SPEED);
    analogWrite(motor2, FORWARD_SPEED);
    analogWrite(motor3, FORWARD_SPEED);
    analogWrite(motor4, BACKWARD_SPEED);
}

void rotateRight(){
    // To rotate right, reverse the left rotation logic
    analogWrite(motor1, FORWARD_SPEED);
    analogWrite(motor2, BACKWARD_SPEED);
    analogWrite(motor3, BACKWARD_SPEED);
    analogWrite(motor4, FORWARD_SPEED);
}

void noMovement(){
    analogWrite(motor1, STOP_SPEED);
    analogWrite(motor2, STOP_SPEED);
    analogWrite(motor3, STOP_SPEED);
    analogWrite(motor4, STOP_SPEED);
}
