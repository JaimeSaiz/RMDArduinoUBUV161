// "CAN-Prog01-RMD-X8-PRO-03.ino"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Licencia: 
// Creative Commons: Reconocimiento - No Comercial - Sin Obra Derivada (by-nc-nd)
// Esta licencia no permite la generación de obras derivadas ni hacer un uso comercial de la obra original, es decir, sólo son posibles los usos y finalidades que no tengan carácter comercial. Esta es la licencia Creative Commons más restrictiva.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Formato sacado de "bump5236/RMDx8Arduino"
// Con RMD_Arduino_CAN_UBU_V161.h, cuyos elementos se ajustan a los nombres reales del fabricante 
// y añade todas las funciones que le faltaban al original. 
// Como estas bibliotecas están cambiando con cierta frecuencia, la biblioteca de Seeed “mcp2515_can.h/cpp” (https://github.com/autowp/arduino-mcp2515) 
// también podrá ser utilizada en el futuro.
// En esta prueba se van a ejecutar los comandos relacionados con la lectura de datos, para probar que funcionan correctamente. 

// Funciones (Se han cambiado todas las variables de la biblitoeca para que ajusten a las definiciones del fabricante):
//    canSetup(); // Ejecuta "CAN_OK != CAN.begin(CAN_1000KBPS)"
//    readPID(); 
//    writePID(anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi); // Write PID to RAM parameter command (one frame).
//    writePIDROM(anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi); // PUEDE DAÑAR EL CHIP SI SE REPITE MUCHAS VECES) Función mía. Write PID to ROM parameter command (one frame).
//    readAccel(); // Función mía. Read acceleration data command (one frame)
//    writeAccel(Accel); // Función mía. Write acceleration data command (one frame)
//    readEncoder(); // Función mía. The host sends the command to read the current position of the encoder
//    writeEncoderOffset(encoderOffset); // The host sends the command to set encoder offset
//    writePositionROMMotorZero(); // PUEDE DAÑAR EL CHIP SI SE REPITE MUCHAS VECES) Función mía. Write current position to ROM as motor zero position command(one frame)
//    readMotorAngle(); // 0X92
//    readSingleCircleAngle(); // Función mía. The host sends command to read the single circle angle of the motor.
//    clearState(); // Función mía. Turn off motor, while clearing the motor operating status and previously received control commands
//    stopMotor(); // Función mía. Stop motor, but do not clear the motor operating state and previously received control commands
//    resumeStopMotor(); // Función mía. Resume motor operation from motor stop command (Recovery control mode before stop motor) 
//    readStatus1(); // Función mía. Da el voltage y el Status.
//    clearErrorFlag(); // Función mía. This command clears the error status of the current motor. Da el voltage y el Status. The error flag cannot be cleared when the motor status does not return to normal.
//    readStatus2(); // Función mía. Da la corriente de cada fase del motor.
//    readStatus3(); // Función mía. Da la temperatura, voltage, velocidad y posición del encoder.
//    writeCurrent(iqControl);
//    writeSpeed(speedControl); // 
//    writePosition3(angleControlMT); // Se ha cambiado el número de la función y el tipo de dato.
//    writePosition4(angleControlMT, maxSpeed); // Se ha cambiado el número de la función y el tipo de dato. 
//    writePosition5(angleControlST, spinDirection); // Se ha cambiado el número de la función y el tipo de dato
//    writePosition6(angleControlST, maxSpeed, spinDirection); // Se ha cambiado el número de la función y el tipo de dato
//    void multiMotorControl(int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4); // Función mía. Hace el control del par de 4 motores en un único comando.

#include <mcp_can.h>
#include <SPI.h>
#include <RMD_Arduino_CAN_UBU_V161.h> // Obliga a utilizar "mcp_can.h"
#define BAUDRATE 115200

//const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)
int MOTOR_ADDRESS1 = 322; //0x140 + ID(1~32) = 320 + 2
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

RMD_Arduino_CAN_UBU_V161 rmd1(CAN, MOTOR_ADDRESS1);
float pos01;

void setup() {
  Serial.begin(BAUDRATE);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Init OK!");
  Serial.print("****************** CAN-Prog01-RMD-X8-PRO-03.ino ***************************"); 

  //SPI.begin();  
  //SPI.begin();   

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Se inician las pruebas de lectura o aquellas funciones que no necesiten datos a enviar.
  // Se sacarán por pantala sólo los datos que devuelven los motores.
  /////////////////////////////////////////////////////////////////////////////////////////////////
  rmd1.clearState(); // Turn off motor, while clearing the motor operating status and previously received control commands
  rmd1.canSetup(); // Ejecuta "CAN_OK != CAN.begin(CAN_1000KBPS)"
  delay(300);
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.println(F("***************** Datos del Motor *******************")); 
  Serial.print(F("MOTOR_ADDRESS1: ")); Serial.print(rmd1.MOTOR_ADDRESS); 
  Serial.print(F("HEX: ")); Serial.println(rmd1.MOTOR_ADDRESS,HEX);
  delay(100);

  rmd1.readPID(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readPID() - 0x30 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - anglePidKp: ")); Serial.print(rmd1.anglePidKp);  
  Serial.print(F(" - anglePidKi: ")); Serial.print(rmd1.anglePidKi);   
  Serial.print(F(" - speedPidKp: ")); Serial.print(rmd1.speedPidKp); 
  Serial.print(F(" - speedPidKi: ")); Serial.print(rmd1.speedPidKi);  
  Serial.print(F(" - iqPidKp: ")); Serial.print(rmd1.iqPidKp); 
  Serial.print(F(" - iqPidKi: ")); Serial.println(rmd1.iqPidKi);  
  Serial.println(F("********************************************************************************************")); 
  delay(100);

  rmd1.readAccel(); // Read acceleration data
  delay(300);
  Serial.println(F("***************** Datos del comando readAccel() - 0x33 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - Accel: ")); Serial.println(rmd1.Accel); 
  delay(100);

  rmd1.readEncoder(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readEncoder() - 0x90 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - encoder: ")); Serial.print(rmd1.encoder); 
  Serial.print(F(" - encoderRaw: ")); Serial.print(rmd1.encoderRaw);  
  Serial.print(F(" - encoderOffset: ")); Serial.println(rmd1.encoderOffset); 
  delay(100);

  rmd1.readMotorAngle(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readMotorAngle() - 0x92 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - motorAngle: ")); Serial.println(rmd1.motorAngle); 
  delay(100);

  rmd1.readSingleCircleAngle(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readSingleCircleAngle() - 0x94 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - circleAngle: ")); Serial.println(rmd1.circleAngle); 
  delay(100);

  rmd1.readStatus1(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readStatus1() - 0x9A *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - temperature: ")); Serial.print(rmd1.temperature); 
  Serial.print(F(" - voltage: ")); Serial.print(rmd1.voltage); 
  Serial.print(F(" - error_state: ")); Serial.println(rmd1.error_state); 
  delay(100);

  rmd1.readStatus2(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readStatus2() - 0x9C *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command); 
  Serial.print(F(" - temperature: ")); Serial.print(rmd1.temperature); 
  Serial.print(F(" - iq: ")); Serial.print(rmd1.iq); 
  Serial.print(F(" - speed: ")); Serial.print(rmd1.speed); 
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 
  delay(100);

  rmd1.readStatus3(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando readStatus3() - 0x9D *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - iA: ")); Serial.print(rmd1.iA); 
  Serial.print(F(" - iB: ")); Serial.print(rmd1.iB); 
  Serial.print(F(" - iC: ")); Serial.println(rmd1.iC); 
  delay(100);

// Otros comandos sin lectura de datos
  rmd1.clearErrorFlag(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando clearErrorFlag() - 0x9B *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - temperature: ")); Serial.print(rmd1.temperature); 
  Serial.print(F(" - voltage: ")); Serial.print(rmd1.voltage); 
  Serial.print(F(" - error_state: ")); Serial.println(rmd1.error_state); 
  delay(100);

  rmd1.clearState(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando clearState() - 0x80 *******************")); 
  //Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  

  rmd1.stopMotor(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando stopMotor() - 0x81 *******************")); 
  //Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  

  rmd1.resumeStopMotor(); // Read PID parameter
  delay(300);
  Serial.println(F("***************** Datos del comando resumeStopMotor() - 0x88 *******************")); 
  //Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Se inician las pruebas de lectura o aquellas funciones que no necesiten datos a enviar.
  // Se sacarán por pantala sólo los datos que devuelven los motores.
  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Envío de control de torque
  double DatoanglePidKp = 1; double DatoanglePidKi = 1; double DatospeedPidKp = 1; double DatospeedPidKi = 1; double DatoiqPidKp = 1; double DatoiqPidKi = 1; 
  int DatoAccel = 1; 
  int DatoiqControl = 1; 
  double DatoencoderOffset = 1; 
  int DatoiqControl_1 = 1; int DatoiqControl_2 = 1; int DatoiqControl_3 = 1; int DatoiqControl_4 = 1;
  double DatospeedControl = 1; 
  double DatoangleControl = 1; 
  double DatoangleControlST = 1; 
  double DatomaxSpeed = 1; 
  double DatospinDirection = 1; 

  rmd1.writePID(DatoanglePidKp, DatoanglePidKi, DatospeedPidKp, DatospeedPidKi, DatoiqPidKp, DatoiqPidKi); //  0x31 // Write PID to RAM parameter
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - anglePidKp: ")); Serial.print(rmd1.anglePidKp); 
  Serial.print(F(" - anglePidKi: ")); Serial.print(rmd1.anglePidKi); 
  Serial.print(F(" - speedPidKp: ")); Serial.print(rmd1.speedPidKp); 
  Serial.print(F(" - speedPidKi: ")); Serial.print(rmd1.speedPidKi); 
  Serial.print(F(" - iqPidKp: ")); Serial.print(rmd1.iqPidKp); 
  Serial.print(F(" - iqPidKi: ")); Serial.println(rmd1.iqPidKi); 
  
  // No se hará con la ROM por si se cometen errores al probarlo en el futuro, pero sí se deja definido.
  /*
  rmd1.writePIDROM(DatoanglePidKp, DatoanglePidKi, DatospeedPidKp, DatospeedPidKi, DatoiqPidKp, DatoiqPidKi); // 0x32 // Write PID to ROM parameter
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - anglePidKp: ")); Serial.print(rmd1.anglePidKp); 
  Serial.print(F(" - anglePidKi: ")); Serial.print(rmd1.anglePidKi); 
  Serial.print(F(" - speedPidKp: ")); Serial.print(rmd1.speedPidKp); 
  Serial.print(F(" - speedPidKi: ")); Serial.print(rmd1.speedPidKi); 
  Serial.print(F(" - iqPidKp: ")); Serial.print(rmd1.iqPidKp); 
  Serial.print(F(" - iqPidKi: ")); Serial.println(rmd1.iqPidKi); 
  */
  
  rmd1.writeAccel(DatoAccel); // 0x34 // Write acceleration data
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - Accel: ")); Serial.println(rmd1.Accel); 

  rmd1.writeEncoderOffset(DatoencoderOffset); // 0x91 // Write encoder offset
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - encoderOffset: ")); Serial.println(rmd1.encoderOffset); 

  rmd1.writePositionROMMotorZero(); // 0x19 // Write current position to ROM as motor zero position
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - Accel: ")); Serial.println(rmd1.Accel); 

  rmd1.writeCurrent(DatoiqControl); // 0xA1 // Torque current control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - iqControl: ")); Serial.println(rmd1.iq); 

  rmd1.writeSpeed(DatospeedControl); // 0xA2 // Speed control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F("temperature: ")); Serial.print(rmd1.temperature);  
  Serial.print(F("iq: ")); Serial.print(rmd1.iq);  
  Serial.print(F("speed: ")); Serial.print(rmd1.speed);  
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 

  rmd1.writePosition3(DatoangleControl); // 0xA3 // Position control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() - 0xA3 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F("temperature: ")); Serial.print(rmd1.temperature);  
  Serial.print(F("iq: ")); Serial.print(rmd1.iq);  
  Serial.print(F("speed: ")); Serial.print(rmd1.speed);  
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 

  rmd1.writePosition4(DatoangleControl, DatomaxSpeed); // 0xA4 // Position control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() - 0xA4 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F("temperature: ")); Serial.print(rmd1.temperature);  
  Serial.print(F("iq: ")); Serial.print(rmd1.iq);  
  Serial.print(F("speed: ")); Serial.print(rmd1.speed);  
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 

  rmd1.writePosition5(DatoangleControlST, DatospinDirection); // 0xA5 // Position control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() - 0xA5 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F("temperature: ")); Serial.print(rmd1.temperature);  
  Serial.print(F("iq: ")); Serial.print(rmd1.iq);  
  Serial.print(F("speed: ")); Serial.print(rmd1.speed);  
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 

  rmd1.writePosition6(DatoangleControlST, DatomaxSpeed, DatospinDirection); // 0xA6 // Position control
  delay(300);
  Serial.println(F("***************** Datos del comando writeCurrent() - 0xA6 *******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.println(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F("command: ")); Serial.print(rmd1.command);  
  Serial.print(F("temperature: ")); Serial.print(rmd1.temperature);  
  Serial.print(F("iq: ")); Serial.print(rmd1.iq);  
  Serial.print(F("speed: ")); Serial.print(rmd1.speed);  
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder); 

  //MOTOR_ADDRESS1 = 0x280;   
  rmd1.multiMotorControl(DatoiqControl_1, DatoiqControl_2, DatoiqControl_3, DatoiqControl_4); // 0x280 // Multiple motor torque // Envía el mismo comando a todas las máquinas.
  delay(300);
  Serial.println(F("***************** Datos del comando multiMotorControl() - 0x280*******************")); 
  Serial.print(F("MOTOR_ADDRESS_Received: ")); Serial.print(rmd1.CAN_MOTOR_ADDRESS, HEX);  
  Serial.print(F(" - command: ")); Serial.print(rmd1.command);  
  Serial.print(F(" - temperature: ")); Serial.print(rmd1.temperature); 
  Serial.print(F(" - iq: ")); Serial.print(rmd1.iq); 
  Serial.print(F(" - speed: ")); Serial.print(rmd1.speed); 
  Serial.print(F(" - encoder: ")); Serial.println(rmd1.encoder);  
}

void loop() {
}
