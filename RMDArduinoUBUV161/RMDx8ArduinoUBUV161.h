#ifndef RMDx8ArduinoUBUV161_h
#define RMDx8ArduinoUBUV161_h

#include "Arduino.h"
#include <mcp_can.h>

class RMDx8ArduinoUBUV161 {
public:
    unsigned char len;
    unsigned char tmp_buf[8], cmd_buf[8], reply_buf[8], pos_buf[8];
    //int8_t 
    int8_t temperature; 
    uint8_t anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi, error_state, command, spinDirection;
    int16_t iq, speed, iA, iB, iC, maxSpeed, iqControl, iqControl_1, iqControl_2, iqControl_3, iqControl_4;
    uint16_t MOTOR_ADDRESS, encoder, encoderRaw, encoderOffset, voltage, circleAngle, angleControlST, CAN_MOTOR_ADDRESS; 
    int32_t Accel, angleControl, speedControl, motorAngle;
    //int64_t motorAngle; // Como la biblioteca mcp_can.h tiene algún problema con este tipo de dato se usa pos_u32t con una serie de operaciones para ajustar los datos.

    RMDx8ArduinoUBUV161(MCP_CAN &CAN, const uint16_t motor_addr); // Maneja el controlador si tiene el mismo nombre que la clase

    // Commands
    void canSetup();
    void readPID(); // 0x30 // Read PID parameter command
    void writePID(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi); // 0x31 // Write PID to RAM parameter command 
    void writePIDROM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi); // 0x32 // Write PID to ROM parameter command 
    void readAccel(); // 0x33 // Read acceleration data command 
    void writeAccel(uint32_t Accel); // 0x34 // Write acceleration data command 
    void readEncoder(); // 0x90 // The host sends the command to read the current position of the encoder
    void writeEncoderOffset(uint16_t encoderOffset); // 0x91 // Write encoder offset command
    void writePositionROMMotorZero(); // 0x19 // Write current position to ROM as motor zero position command
    void readMotorAngle(); // 0x92 // Read multi turns angle command
    void readSingleCircleAngle(); // 0x94 // The host sends command to read the single circle angle of the motor.
    void readStatus1(); // 0x9A // This command reads the motor's error status and voltage, temperature and other information.
    void clearErrorFlag(); // 0x9B // This command clears the error status of the current motor.
    void readStatus2(); // 0x9C // This command reads motor temperature, voltage, speed, encoder position
    void readStatus3(); // 0x9D // This command reads the phase current status data of the motor.
    void clearState(); // 0x80 // Turn off motor, while clearing the motor operating status and previously received control commands
    void stopMotor(); // 0x81 // Stop motor, but do not clear the motor operating state and previously received control commands
    void resumeStopMotor(); // 0x88 // Resume motor operation from motor stop command (Recovery control mode before stop motor)
    void writeCurrent(int16_t iqControl); // 0xA1 // Torque current control command
    void writeSpeed(int32_t speedControl); // 0xA2 // Speed control command 
    void writePosition3(int32_t angleControl); // 0xA3 // Position control command 1 
    void writePosition4(int32_t angleControl, uint16_t maxSpeed); // 0xA4 // Position control command 2, multi turns 
    void writePosition5(uint16_t angleControlST, uint8_t spinDirection); // 0xA5 // Position control command 3, single turn
    void writePosition6(uint16_t angleControlST, uint16_t maxSpeed, uint8_t spinDirection); // 0xA6 // Position control command 4, single turn
    void multiMotorControl(int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4); // 0x280 // Multiple motor torque closed loop control commands

    // General function
    void serialWriteTerminator();

private:
    MCP_CAN _CAN;
    uint32_t pos_u32t;

    void readBuf(unsigned char *buf);
    void writeCmd(unsigned char *buf);
};

#endif
