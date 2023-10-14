#include "AX12A.h"
#include "mbed.h"

AX12A::AX12A(PinName tx, PinName rx, PinName txEnable, int baudrate) :  serial_servo(tx, rx), s_txEnable(txEnable), s_baudrate(baudrate){
    serial_servo.set_baud(s_baudrate);
}

uint8_t AX12A::ping(uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH]={0};
    int elements = 6;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x02;         // Length
        instructionBuffer[4] = PING;         // Instruction
        instructionBuffer[5] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}

uint8_t AX12A::toggleLED(uint8_t ledState, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 8;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_LED;  // Parameter 1: Starting address
        instructionBuffer[6] = ledState; 
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}

uint8_t  AX12A::move_to(uint16_t position, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 9;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x05;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_GOAL_POSITION;  // Parameter 1: Starting address
        instructionBuffer[6] = position & 0xff; 
        instructionBuffer[7] = position >> 8;
        instructionBuffer[8] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}


uint8_t AX12A:: setAngleLimit(uint16_t mode, uint8_t address, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 9;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x05;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = address;  // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) mode & 0xff;    
        instructionBuffer[7] = (uint8_t) (mode >> 8);
        instructionBuffer[8] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}

uint8_t  AX12A::setSpeed(uint16_t speed, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 9;
    char16_t checkSum = {0};
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x05;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_MOVING_SPEED;  // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) speed & 0xff;     
        instructionBuffer[7] = (uint8_t) (speed >> 8);
        instructionBuffer[8] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}

uint8_t AX12A::setID(uint8_t newID, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 8;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_ID;  // Parameter 1: Starting address
        instructionBuffer[6] = newID;     
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;
}

uint8_t AX12A::setMaxTorque(uint16_t MaxTorque, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 9;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x05;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_MAX_TORQUE_L;  // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) MaxTorque & 0xff;     
        instructionBuffer[7] = (uint8_t) (MaxTorque >> 8);
        instructionBuffer[8] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;

}

uint8_t AX12A::setTorque_Limit(uint16_t MaxTorque, uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 9;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x05;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_TORQUE_LIMIT;  // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) MaxTorque & 0xff;     
        instructionBuffer[7] = (uint8_t) (MaxTorque >> 8);
        instructionBuffer[8] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]) & 0xFF;   // Check sum
        return(write_instruction_packet(instructionBuffer, elements));
    }
    return  128;      
}

uint16_t AX12A::getPosition(uint8_t s_servoID){
    char instructionBuffer[STATUS_BUFFER_LENGTH];
    int elements = 8;
    if (serial_servo.writable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = s_servoID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = READ_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_PRESENT_POSITION;  // Parameter 1: Starting address
        instructionBuffer[6] = 0x02;   
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum
        return(write_instruction_packet_read_data(instructionBuffer, elements));
    }
    return  128;
}

uint8_t AX12A::write_instruction_packet(char *instructionBuffer, int elements){
    //Write instruction packet
    s_txEnable = 1;  
    serial_servo.write(instructionBuffer, elements);
    serial_servo.sync();
    s_txEnable = 0;
    
    //Read Status Packet
    char statusBuffer[6] = {0};
    if ( s_servoID != BROADCAST_ID ) {
        if (serial_servo.readable()){
            serial_servo.read(statusBuffer, 6);
        }
        return statusBuffer[4]; //return error
    }
    return 128;
}

uint16_t AX12A::write_instruction_packet_read_data(char *instructionBuffer, int elements){
    //Write instruction packet
    s_txEnable = 1;  
    serial_servo.write(instructionBuffer, elements);
    serial_servo.sync();
    s_txEnable = 0;
    
    //Read data 2 byte
    char statusBuffer[8] = {0};
    if ( s_servoID != BROADCAST_ID ) {
        if (serial_servo.readable()){
            serial_servo.read(statusBuffer, 8);
        }
        uint16_t data;
        data = statusBuffer[6] << 8;
        data = data + statusBuffer[5];
        return data;
    }
    return 128;
}


