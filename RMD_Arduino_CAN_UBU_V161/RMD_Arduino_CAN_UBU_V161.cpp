// HAY QUE REVISAR 0X92
// DEBE SER MULTI-VUELTA Y POR TANTO CON 64 BITS.
// SI SE CAMBIA DE TIPO DE DATOS, HAY QUE MIRAR ESA RESTA DERIVADA DEL TIPO DE DATO 
// PARA QUE SEA CORRECTA (VER https://clickhouse.tech/docs/es/sql-reference/data-types/int-uint/).
// SU USO DEPENDE DEL ENCODER (DE 12bits A 18bits)
// -----------------------------
// HAY QUE REVISAR LOS TIPOS DE DATOS  int8_t,  uint8_t, int16_t,  uint16_t, uint32_t, int32_t, int64_t.
// -----------------------------
// -----------------------------


#include "Arduino.h"
#include "RMDArduinoUBUV161.h"
#include <mcp_can.h>

// constructor
RMDArduinoUBUV161::RMDArduinoUBUV161(MCP_CAN &CAN, const uint16_t motor_addr) 
    :_CAN(CAN){
        MOTOR_ADDRESS = motor_addr;
    }

void RMDArduinoUBUV161::canSetup() {
    while (CAN_OK != _CAN.begin(CAN_1000KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

/** 
 * Single motor Command description
 * Read PID parameter command 
 * The host sends the command to read the PID parameters. 
*/
void RMDArduinoUBUV161::readPID() {
    cmd_buf[0] = 0x30;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    
    // Send message
    writeCmd(cmd_buf);
    delay(100);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    anglePidKp = reply_buf[2];
    anglePidKi = reply_buf[3];
    speedPidKp = reply_buf[4];
    speedPidKi = reply_buf[5];
    iqPidKp = reply_buf[6];
    iqPidKi = reply_buf[7];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Write PID to RAM parameter command 
 * The host sends the command to write the PID parameters to the RAM. Parameters are invalid when power
 * turned off.
*/
void RMDArduinoUBUV161::writePID(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
    cmd_buf[0] = 0x31;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = anglePidKp;
    cmd_buf[3] = anglePidKi;
    cmd_buf[4] = speedPidKp;
    cmd_buf[5] = speedPidKi;
    cmd_buf[6] = iqPidKp;
    cmd_buf[7] = iqPidKi;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Write PID to ROM parameter command 
 * The host sends the command to write the PID parameters to the ROM. Parameter are still valid when power
 * turned off.
*/
void RMDArduinoUBUV161::writePIDROM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
    cmd_buf[0] = 0x32;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = anglePidKp;
    cmd_buf[3] = anglePidKi;
    cmd_buf[4] = speedPidKp;
    cmd_buf[5] = speedPidKi;
    cmd_buf[6] = iqPidKp;
    cmd_buf[7] = iqPidKi;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Read acceleration data command 
 * The host send the command to read motor acceleration data
 * The driver reply data include acceleration data, data type: int32_t; unit:1dps/s
*/
void RMDArduinoUBUV161::readAccel() { 
    cmd_buf[0] = 0x33;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    Accel = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Write acceleration data command 
 * The host sends the command to write the acceleration to the RAM, and the write parameters are invalid after
 * the power is turned off. Acceleration data Accel is int32_t type, unit 1dps/s
 * The motor responds to the host after receiving the command, the frame data is the same as the host sent
*/
void RMDArduinoUBUV161::writeAccel(uint32_t Accel) { 
    cmd_buf[0] = 0x34;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = Accel & 0xFF;
    cmd_buf[5] = (Accel >> 8) & 0xFF;
    cmd_buf[6] = (Accel >> 16) & 0xFF;
    cmd_buf[7] = (Accel >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * The host sends the command to read the current position of the encoder
 * The motor responds to the host after receiving the command, the frame data contains: 
 * Encoder current position (uint16_t type, 14bit encoder value range 0~16383), which is the encoder
 * original position minus the encoder zero offset value
 * Encoder original position (encoderRaw) (uint16_t type, 14bit encoder value range 0~16383)
 * EncoderOffset(uint16_t type, 14bit encoder value range 0~16383)
*/
void RMDArduinoUBUV161::readEncoder() { 
    cmd_buf[0] = 0x90;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoder = ((uint16_t)reply_buf[3] << 8) + reply_buf[2];
    encoderRaw = ((uint16_t)reply_buf[5] << 8) + reply_buf[4];
    encoderOffset = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Write encoder offset command
 * The host sends the command to set encoder offset, written the encoder offset with uint16_t type, 14bit encoder value 
 * range (0, 16383)
 * The motor responds to the host after receiving the command, the frame data contains the following parameters: 
 * 	New encoderOffset (uint16_t type, 14bit encoder value range (0, 16383)) 
*/
void RMDArduinoUBUV161::writeEncoderOffset(uint16_t encoderOffset) { 
    cmd_buf[0] = 0x91;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = encoderOffset & 0xFF;
    cmd_buf[7] = (encoderOffset >> 8) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderOffset = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Write current position to ROM as motor zero position command
 * This command needs to be powered on again to take effect.
 * This command will write the zero position to ROM. Multiple writes will affect the chip life. so not recommended to 
 * use it frequently.
 * The motor returns to the host after receiving the command, and the data is offset value.
*/
void RMDArduinoUBUV161::writePositionROMMotorZero() { 
    cmd_buf[0] = 0x19;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderOffset = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**  TO CHANGE – DEBE SER MULTI-VUELTA Y POR TANTO CON 64 BITS.
 * Read multi turns angle command 
 * The host sends command to read the multi-turn angle of the motor
 * The motor responds to the host after receiving the command, the frame data contains the following parameters: 
 * 	Motor angle, int64_t type data, positive value indicates clockwise cumulative angle, negative value indicates
 * 	counterclockwise cumulative angle, unit 0.01°/LSB.
*/
void RMDArduinoUBUV161::readMotorAngle() {
    cmd_buf[0] = 0x92;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    /////////////////////////////
    //motorAngle = ((int64_t)reply_buf[7] << 56) + ((int64_t)reply_buf[6] << 48) + ((int64_t)reply_buf[5] << 40) + ((int64_t)reply_buf[5] << 32) + ((int64_t)reply_buf[4] << 24) + ((int64_t)reply_buf[3] << 16) + ((int64_t)reply_buf[2] << 8) + reply_buf[1];
    ////////////////////////////////
    pos_u32t = ((uint32_t)reply_buf[4] << 24) + ((uint32_t)reply_buf[3] << 16) + ((uint32_t)reply_buf[2] << 8) + reply_buf[1];
    if (pos_u32t > 2147483648) {
        motorAngle = pos_u32t - 4294967296;
    }
    else {
        motorAngle = pos_u32t;
    }
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Read single circle angle command 
 * The host sends command to read the single circle angle of the motor.
 * The motor responds to the host after receiving the command, the frame data contains the following parameters: 
 * 	CircleAngle, uint16_t type data, starting from the encoder zero point, increased by clockwise rotation, and 
 * 	returning to zero when it reaches zero again, the unit is 0.01°/LSB.
*/
void RMDArduinoUBUV161::readSingleCircleAngle() { 
    cmd_buf[0] = 0x94;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    command = reply_buf[0];
    circleAngle = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Read motor status 1 and error flag commands 
 * This command reads the motor's error status and voltage, temperature and other information.
 * The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type,unit 1?/LSB) 
 * 	Voltage (uint16_t type, unit 0.1V/LSB) 
 * 	Error State (uint8_t type, Each bit represents a different motor state) 	
 * The control value voltage limits the maximum voltage at which the motor rotates, uint16_t type,  corresponding
 * to the actual voltage of 0.1 V/LSB.
 * The control value error_state determine the error produced.Each bit represents a different motor state.
*/
void RMDArduinoUBUV161::readStatus1() {
    cmd_buf[0] = 0x9A;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    voltage = ((uint16_t)reply_buf[4] << 8) + reply_buf[3];
    error_state = reply_buf[7];
    if (error_state == 1) Serial.println("*** Low voltage protection"); // bit 0
    if (error_state == 8) Serial.println("*** Over temperature protection"); // bit 4
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Clear motor error flag command 
 * This command clears the error status of the current motor.
 * The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type,unit 1?/LSB) 
 * 	Voltage (uint16_t type, unit 0.1V/LSB) 
 * 	Error State (uint8_t type, Each bit represents a different motor state) 	
 * The control value voltage limits the maximum voltage at which the motor rotates, uint16_t type,  corresponding
 * to the actual voltage of 0.1 V/LSB.
 * The control value error_state determine the error produced.Each bit represents a different motor state.
*/
void RMDArduinoUBUV161::clearErrorFlag() { 
    cmd_buf[0] = 0x9B;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    voltage = ((uint16_t)reply_buf[4] << 8) + reply_buf[3];
    error_state = reply_buf[7];
    if (error_state == 1) Serial.println("*** Low voltage protection"); // bit 0
    if (error_state == 8) Serial.print("*** Over temperature protection"); // bit 4
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Read motor status 2 
 * This command reads motor temperature, voltage, speed, encoder position
 * 	Motor temperature(int8_t type,unit 1?/LSB)
 * 	Motor torque current (Iq) is int16_t type, the value range: (-2048, 2048), corresponding to the actual torque 
 * 	current range (-33A, 33A) (the bus current and the actual torque of motor vary with different motors)
 * 	Motor speed is int16_t type, which corresponds to the actual speed of 1dps/LSB.
 * 	Encoder position value is uint16_t type, 14bit encoder value range (0, 16383)
*/
void RMDArduinoUBUV161::readStatus2() {
    cmd_buf[0] = 0x9C;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Read motor status 3
 * This command reads the phase current status data of the motor.
 * The motor returns to the host after receiving the command. The frame data contains three-phase current data, the
 * data type is int16_t type, corresponding to the actual phase current is 1A/64LSB.
*/
void RMDArduinoUBUV161::readStatus3() {
    cmd_buf[0] = 0x9D;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    iA = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    iB = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    iC = ((int16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Motor off command
 * Turn off motor, while clearing the motor operating status and previously received control commands
 * The motor responds to the host after receiving the command, the frame data is the same as the host sent
*/
void RMDArduinoUBUV161::clearState() { 
    cmd_buf[0] = 0x80;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Motor stop command 
 * Stop motor, but do not clear the motor operating state and previously received control commands
 * The motor responds to the host after receiving the command, the frame data is the same as the host sent
*/
void RMDArduinoUBUV161::stopMotor() {
    cmd_buf[0] = 0x81;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Motor running command 
 * Resume motor operation from motor stop command (Recovery control mode before stop motor)
 * The motor responds to the host after receiving the command, the frame data is the same as the host sent
*/
void RMDArduinoUBUV161::resumeStopMotor() {
    cmd_buf[0] = 0x88;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**
 * Torque current control command 
 * The host sends the command to control torque current output of the motor. Iq Control is int16_t type, the value range: 
 * (-2000, 2000), corresponding to the actual torque current range (-32A, 32A).
 * The bus current and the actual torque of motor vary with different motors.
 * Iq Control in this command is not limited by the Max Torque Current value in the host computer.
 * The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writeCurrent(int16_t iqControl) { 
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = iqControl & 0xFF;
    cmd_buf[5] = (iqControl) >> 8 & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**
 * Speed control command 
 * The host sends this command to control the speed of the motor. Speed Control is int32_t type, which corresponds to 
 * the actual speed of 0.01dps/LSB.
 * The maximum torque current under this command is limited by the Max Torque Current value in the host computer.
 * In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host 
 * computer.
 * The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writeSpeed(int32_t speedControl) { 
    cmd_buf[0] = 0xA2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = speedControl & 0xFF; 
    cmd_buf[5] = (speedControl >> 8) & 0xFF;
    cmd_buf[6] = (speedControl >> 16) & 0xFF;
    cmd_buf[7] = (speedControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**
 * Position control command 1 
 * The host sends this command to control the position of the motor (multi-turn angle).Angle Control is int32_t type, 
 * and the actual position is 0.01degree/LSB, 36000 represents 360°. The motor rotation direction is determined by the 
 * difference between the target position and the current position.
 * Angle Control under this command is limited by the Max Angle value in the host computer.
 * The maximum speed under this command is limited by the Max Speed value in the host computer.
 * In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host 
 * computer.
 * In this control mode, the maximum torque current of the motor is limited by the Max Torque Current value in the 
 * host computer.
* The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writePosition3(int32_t angleControl) { 
    cmd_buf[0] = 0xA3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**
 * Position control command 2, multi turns 
 * The host sends this command to control the position of the motor (multi-turn angle). Angle Control is int32_t type, 
 * and the actual position is 0.01degree/LSB, 36000 represents 360°. The motor rotation direction is determined by the 
 * difference between the target position and the current position.
 * The control value maxSpeed limits the maximum speed at which the motor rotates, uint16_t type, corresponding to 
 * the actual speed of 1dps/LSB.
 * Angle Control under this command is limited by the Max Angle value in the host computer.
 * In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host 
 * computer.
 * In this control mode, the maximum torque current of the motor is limited by the Max Torque Current value in the 
 * host computer.
* The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writePosition4(int32_t angleControl, uint16_t maxSpeed) { 
    cmd_buf[0] = 0xA4;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = maxSpeed & 0xFF;
    cmd_buf[3] = (maxSpeed >> 8) & 0xFF;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/**
 * Position control command 3, single turn 
 * The host sends this command to control the position of the motor (single-turn angle). Angle Control is uint16_t type, 
 * the value range is (0, 35999), and the actual position is 0.01 degree/LSB, the actual angle range is (0°, 359.99°).
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for 
 * clockwise (CW) and 0x01 for counterclockwise (CCW).
 * The maximum speed under this command is limited by the Max Speed value in the host computer.
 * In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host 
 * computer.
 * In this control mode, the maximum torque current of the motor is limited by the Max Torque Current value in the 
 * host computer.
* The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writePosition5(uint16_t angleControlST, uint8_t spinDirection) { 
    cmd_buf[0] = 0xA5;
    cmd_buf[1] = spinDirection;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = angleControlST & 0xFF; 		// 16 bits
    cmd_buf[5] = (angleControlST >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

/** 
 * Position control command 4, single turn 
 * The host sends this command to control the position of the motor (single-turn angle)
 * Angle Control is uint16_t type, the value range is (0, 35999), and the actual position is 0.01 degree/LSB, the actual 
 * angle range is (0°, 359.99°).
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for 
 * clockwise (CW) and 0x01 for counterclockwise (CCW).
 * Max Speed limits the maximum speed of motor rotation, which is uint16_t type, corresponding to the
 * actual speed of 1dps/LSB.
 * In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host 
 * computer.
 * In this control mode, the maximum torque current of the motor is limited by the Max Torque Current value in the 
 * host computer.
* The motor responds to the host after receiving the command, the frame data contains the following parameters:
 * 	Motor temperature (int8_t type, unit 1?/LSB)
 * 	Motor torque current (Iq) (int16_t type, Range: (-2048, 2048), real torque current range: (-33A, 33A)
 * 	Motor speed (int16_t type, 1dps/LSB)
 * 	Encoder position value (uint16_t type, 14bit encoder value range (0, 16383)
 */
void RMDArduinoUBUV161::writePosition6(uint16_t angleControlST, uint16_t maxSpeed, uint8_t spinDirection) { 
    cmd_buf[0] = 0xA6;
    cmd_buf[1] = spinDirection;
    cmd_buf[2] = maxSpeed & 0xFF;
    cmd_buf[3] = (maxSpeed >> 8) & 0xFF;
    cmd_buf[4] = angleControlST & 0xFF; 		// 16 bits
    cmd_buf[5] = (angleControlST >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    //CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se estrae la ID del motor que responde 
}

//////////////////////////////////////
// A diferencia del resto de comandos, el identificador 0x280 se envía como MOTOR_ADDRESS y no como cmd_buf[0] del dato.
//////////////////////////////////////
/** 
 * Multiple motor torque closed loop control commands
 * The format of the message used to send commands to multiple motors at the same time, as followed:
 * Identifier 0x280 Frame format DATA
 * Frame type standard frame DLC 8byte
 * The host simultaneously send this command to control the torque current output up to 4 motors. The
 * control value iqControl is int16_t type, the value range is -2000 to 2000, corresponding to the actual torque current
 * range -32A to 32A (The bus current and the actual torque of the motor vary from motor to motor).

 * The message format of each motor reply command is as follows...
 * Identifier 0x140+ID(1-4) Frame format DATA
 * Frame type standard frame DLC 8byte
 * Each motor reply according to the ID from small to large, and the reply data of each motor is the same as the
 * single motor torque closed-loop control command reply data (0xA1).
 * Note - We have differ encoder type from 12bit to 18bit , so the reply encoder value range vary from motor to motor. It
 * depands on which type you purchased.
 */
void RMDArduinoUBUV161::multiMotorControl(int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4) { 
    cmd_buf[0] = iqControl_1 & 0xFF;
    cmd_buf[1] = (iqControl_1) >> 8 & 0xFF;
    cmd_buf[0] = iqControl_2 & 0xFF;
    cmd_buf[1] = (iqControl_2) >> 8 & 0xFF;
    cmd_buf[0] = iqControl_3 & 0xFF;
    cmd_buf[1] = (iqControl_3) >> 8 & 0xFF;
    cmd_buf[0] = iqControl_4 & 0xFF;
    cmd_buf[1] = (iqControl_4) >> 8 & 0xFF;
    MOTOR_ADDRESS = 0x280;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    //readBuf(cmd_buf);
    // Receive message 
    // Se identifica a cada motor que responde por la dirección MOTOR_ADDRESS = 0x241, 0x242, 0z243, ... (Direcciones de cada motor que responda + COMMAND enviado)
    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se extrae la ID del motor que responda en primer lugar. Para el resto se deben plantear interrupciones. 
}

// General function
void RMDArduinoUBUV161::serialWriteTerminator() {
    Serial.write(13);
    Serial.write(10);
}

// Private
void RMDArduinoUBUV161::readBuf(unsigned char *buf) {
    delayMicroseconds(10000);    // 600us
    if (CAN_MSGAVAIL == _CAN.checkReceive()) {
        _CAN.readMsgBuf(&len, tmp_buf);
        CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se extrae la ID del motor que responda en primer lugar. Para el resto se deben plantear interrupciones. 
        if (tmp_buf[0] == buf[0]) {
            reply_buf[0] = tmp_buf[0];
            reply_buf[1] = tmp_buf[1];
            reply_buf[2] = tmp_buf[2];
            reply_buf[3] = tmp_buf[3];
            reply_buf[4] = tmp_buf[4];
            reply_buf[5] = tmp_buf[5];
            reply_buf[6] = tmp_buf[6];
            reply_buf[7] = tmp_buf[7];
        }
    }
}

void RMDArduinoUBUV161::writeCmd(unsigned char *buf) {
    unsigned char sendState = _CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, buf);
    if (sendState != CAN_OK) { //  //
        Serial.println("Error Sending Message (RMDArduinoUBUV161.h)...");
        Serial.print("sendState= "); Serial.println(sendState);
        Serial.print(" - CAN_FAILTX= ");Serial.print(CAN_FAILTX);
        Serial.print(" - CAN_OK= ");Serial.println(CAN_OK);
   }
}
