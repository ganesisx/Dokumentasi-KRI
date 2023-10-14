#ifndef DEF_AX12A
#define DEF_AX12A

#include "mbed.h"

const unsigned char BROADCAST_ID = 0xFE;
const unsigned char STATUS_BUFFER_LENGTH = 0x10;

// Instruction set from https://emanual.robotis.com/docs/en/dxl/protocol1/#
const unsigned char PING = 0x01;
const unsigned char READ_DATA = 0x02;
const unsigned char WRITE_DATA = 0x03;
const unsigned char REG_WRITE = 0x04;
const unsigned char ACTION = 0x05;
const unsigned char FACTORY_RESET = 0x06;
const unsigned char REBOOT = 0x08;
const unsigned char SYNC_WRITE = 0x83;
const unsigned char BULK_READ = 0x92;

// Control table from https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
const unsigned char ADDRESS_ID = 0x03;
const unsigned char ADDRESS_BAUD_RATE = 0x04;
const unsigned char ADDRESS_RETURN_DELAY_TIME = 0x05;
const unsigned char ADDRESS_CW_ANGLE_LIMIT = 0x06;
const unsigned char ADDRESS_CCW_ANGLE_LIMIT = 0x08;
const unsigned char ADDRESS_MAX_TORQUE_L = 0x0E;
const unsigned char ADDRESS_TORQUE_ENABLE = 0x18;
const unsigned char ADDRESS_LED = 0x19;
const unsigned char ADDRESS_GOAL_POSITION = 0x1E;
const unsigned char ADDRESS_MOVING_SPEED = 0x20;
const unsigned char ADDRESS_TORQUE_LIMIT = 0x22;
const unsigned char ADDRESS_PRESENT_POSITION = 0x24;
const unsigned char ADDRESS_PRESENT_SPEED = 0x26;
const unsigned char ADDRESS_PRESENT_LOAD = 0x28;
const unsigned char ADDRESS_PRESENT_VOLTAGE = 0x2A;
const unsigned char ADDRESS_PRESENT_TEMPERATURE = 0x2B;
const unsigned char ADDRESS_MOVING = 0x2E;

class AX12A
{
public:
    AX12A(PinName tx, PinName rx, PinName txEnable, int baudrate);
    uint8_t ping(uint8_t s_servoID);
    uint8_t toggleLED(uint8_t ledState, uint8_t s_servoID);
    uint8_t move_to(uint16_t position, uint8_t s_servoID);
    uint8_t setAngleLimit(uint16_t mode, uint8_t address, uint8_t s_servoID);
    uint8_t setSpeed(uint16_t speed, uint8_t s_servoID);
    uint8_t turn(uint16_t Speed, bool SIDE, uint8_t s_servoID)
    uint8_t setID(uint8_t newID, uint8_t s_servoID);
    uint8_t setMaxTorque(uint16_t MaxTorque, uint8_t s_servoID);
    uint8_t setTorque_Limit(uint16_t MaxTorque, uint8_t s_servoID);
    uint16_t getPosition(uint8_t s_servoID);
    uint8_t write_instruction_packet(char *instructionBuffer, int elements);
    uint16_t write_instruction_packet_read_data(char *instructionBuffer, int elements);
    
private:
    BufferedSerial serial_servo;
    DigitalOut s_txEnable;
    uint8_t s_servoID; //default 1
    const int s_baudrate; //default 1Mbps
};
#endif

