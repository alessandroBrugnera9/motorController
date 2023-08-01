#include <Arduino.h>
#include <mcp_can.h>
#include <MotorHandler.h>
#include <EasyCAT.h>

// MOTOR CONTROL PARAMTERS
const int baseKp = 300;
const int baseKd = 4;

// CONTROL VARIABLES
enum motorMode
{
  position,
  torque
};

struct motorCommand
{
  uint8_t mode;
  uint8_t motorCommandPackage[8];
};

motorCommand hipCommand;
motorCommand kneeCommand;
uint8_t hipResponse[6];
uint8_t kneeResponse[6];

// CONTROL INSTANCES
// setting can bus handler though SPI pins
const byte spiCSPin = 10;
MCP_CAN canHandler(spiCSPin);

// motor object instantiation
MotorHandler hipMotor(canHandler, 0x01, baseKp, baseKd);
MotorHandler kneeMotor(canHandler, 0x02, baseKp, baseKd);

// LOOP MEASUREMENT
unsigned int counter = 0;
const int nLoops = 100;
unsigned long microsStart;

// Ethercat COMMUNICATION
EasyCAT EASYCAT(9); // EasyCAT SPI chip select. Standard is pin 9

enum ethercatBytes
{
  mode,
  posHigh,
  posLow,
  velHigh,
  velLowKpHigh,
  kpLow,
  kdhigh,
  kdLowTorHigh,
  torLow
};

void readEthercat()
{
  // TODO: decide if this is ISR or not
  // TODO: read from XPC

  if (EASYCAT.BufferOut.Byte[0] != 1)
  {
    // NEED: check if knee or hip comes first
    // reading hip comands
    hipCommand.mode = EASYCAT.BufferOut.Byte[mode + 8];
    hipCommand.motorCommandPackage[0] = EASYCAT.BufferOut.Byte[posHigh + 8];
    hipCommand.motorCommandPackage[1] = EASYCAT.BufferOut.Byte[posLow + 8];
    hipCommand.motorCommandPackage[2] = EASYCAT.BufferOut.Byte[velHigh + 8];
    hipCommand.motorCommandPackage[3] = EASYCAT.BufferOut.Byte[velLowKpHigh + 8];
    hipCommand.motorCommandPackage[4] = EASYCAT.BufferOut.Byte[kpLow + 8];
    hipCommand.motorCommandPackage[5] = EASYCAT.BufferOut.Byte[kdhigh + 8];
    hipCommand.motorCommandPackage[6] = EASYCAT.BufferOut.Byte[kdLowTorHigh + 8];
    hipCommand.motorCommandPackage[7] = EASYCAT.BufferOut.Byte[torLow + 8];
    
    // reading knee comands
    kneeCommand.mode = EASYCAT.BufferOut.Byte[mode];
    kneeCommand.motorCommandPackage[0] = EASYCAT.BufferOut.Byte[posHigh];
    kneeCommand.motorCommandPackage[1] = EASYCAT.BufferOut.Byte[posLow];
    kneeCommand.motorCommandPackage[2] = EASYCAT.BufferOut.Byte[velHigh];
    kneeCommand.motorCommandPackage[3] = EASYCAT.BufferOut.Byte[velLowKpHigh];
    kneeCommand.motorCommandPackage[4] = EASYCAT.BufferOut.Byte[kpLow];
    kneeCommand.motorCommandPackage[5] = EASYCAT.BufferOut.Byte[kdhigh];
    kneeCommand.motorCommandPackage[6] = EASYCAT.BufferOut.Byte[kdLowTorHigh];
    kneeCommand.motorCommandPackage[7] = EASYCAT.BufferOut.Byte[torLow];
  }
  else
  {
    Serial.println("Ethercat is not sending commands");
  }
}

void sendEthercat()
{
  // TODO: handler variables to send to XPC
  // TODO: send to XPC

  // sending responses
  for (byte i = 0; i < 6; i++)
  {
    // NEED: check if knee or hip comes first
    EASYCAT.BufferIn.Byte[i] = kneeResponse[i];
    EASYCAT.BufferIn.Byte[i+6] = hipResponse[i];
  }
  
}



// AUXILIARY FUNCTIONS
void initializeCanBus()
{
  while (CAN_OK != canHandler.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ))
  {
    Serial.println("CAN Bus initialization failed");
    Serial.println("Initializing CAN Bus again");
    delay(100);
  }
  Serial.println("CAN Bus init OK");
}

void getMotorsResponses()
{
  const unsigned char* hipPointer = hipMotor.getRawMotorResponse();
  std::memcpy(hipResponse, hipPointer, 6);
  const unsigned char* kneePointer = kneeMotor.getRawMotorResponse();
  std::memcpy(kneeResponse, kneePointer, 6);
}

void sendMotorCommand(motorCommand &command, MotorHandler &motor)
{
  switch (command.mode)
  {
  case position:
    motor.sendRawCommand(command.motorCommandPackage);
    break;
  case torque:
    motor.sendRawCommand(command.motorCommandPackage);
    break;
  default:
    break;
  }
}



void setup()
{
  Serial.begin(9600); // Begin Serial port to talk to computer and receive commands

  // Begin the CAN Bus and set frequency to 8 MHz and baudrate of 1000kb/s  and the masks and filters disabled.
  initializeCanBus();
  canHandler.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted and received

  // Preparing motor to listen to commands
  hipMotor.exitMotorMode();
  hipMotor.zeroPosition();
  kneeMotor.exitMotorMode();
  kneeMotor.zeroPosition();

  // Ethercat
  if (EASYCAT.Init() == true) // initilization succesfully completed
  {
    Serial.println("EtherCAT initialization completed");
    EASYCAT.MainTask(); // execute the EasyCAT task

    hipMotor.enterMotorMode();
    kneeMotor.enterMotorMode();
  }

  else // initialization failed
  {
    Serial.println("EtherCAT initialization failed");
  }

  counter = 0;
  Serial.println("Setup finished.");
}

void loop()
{
  // measure the time for 100 loops
  if (counter == 0)
  {
    microsStart = micros();
  }
  else if (counter == nLoops)
  {
    Serial.print("Time for 100 loops: ");
    Serial.println(micros() - microsStart);
    counter = 0;
  }

  readEthercat();

  // send motor commands
  sendMotorCommand(hipCommand, hipMotor);
  sendMotorCommand(kneeCommand, kneeMotor);

  getMotorsResponses();

  // send to XPC
  sendEthercat();
  counter++;
}