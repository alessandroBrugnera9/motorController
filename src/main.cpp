#include <Arduino.h>
#include <mcp_can.h>
#include <MotorHandler.h>
#include <EasyCAT.h>
#include <cstring>

// DEFINITIONS
const unsigned long motorResponseTimeout = 1650; // 1.6ms
// CONTROL VARIABLES
// Communication
enum motorCANIds
{
  hipId = 0x01,
  kneeId = 0x02,
};

enum SPIPins
{
  ethercatPin = 9,
  canBusPin = 10,
};

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

// -
// Motor Control paramters
const int baseKp = 300;
const int baseKd = 4;

unsigned char disableControllerBuf[8] = {
    0, 0, 0, 0, 0, 0, 0x07, 0xFF};

enum motorMode
{
  turnOff,
  position,
  torque,
  zeroPosition,
};

struct motorInfo
{
  uint8_t mode;
  boolean systemOn;
  boolean systemAtZero;
  uint8_t motorCommandPackage[8];
  uint8_t response[5];
};

// --
// CONTROL INSTANCES
// Communication
EasyCAT EASYCAT(ethercatPin);
MCP_CAN canHandler(canBusPin);

// -
// motor command objects
motorInfo hipInfo;
motorInfo kneeInfo;

// Motor Object Instantiation
MotorHandler hipMotor(canHandler, hipId, baseKp, baseKd);
MotorHandler kneeMotor(canHandler, kneeId, baseKp, baseKd);

// --
// FUNCTIONS
// Communication
void getXPCCommand(unsigned char *motorCommandPackage, int startIndex)
{
  motorCommandPackage[0] = EASYCAT.BufferOut.Byte[posHigh + startIndex];
  motorCommandPackage[1] = EASYCAT.BufferOut.Byte[posLow + startIndex];
  motorCommandPackage[2] = EASYCAT.BufferOut.Byte[velHigh + startIndex];
  motorCommandPackage[3] = EASYCAT.BufferOut.Byte[velLowKpHigh + startIndex];
  motorCommandPackage[4] = EASYCAT.BufferOut.Byte[kpLow + startIndex];
  motorCommandPackage[5] = EASYCAT.BufferOut.Byte[kdhigh + startIndex];
  motorCommandPackage[6] = EASYCAT.BufferOut.Byte[kdLowTorHigh + startIndex];
  motorCommandPackage[7] = EASYCAT.BufferOut.Byte[torLow + startIndex];
}

boolean isValidMotorCommand(unsigned char *motorCommandPackage)
{
  // loop through the array and check if any value is different than 0
  for (byte i = 0; i < 8; i++)
  {
    if (motorCommandPackage[i] != 0)
    {
      return true;
    }
  }
  return false;
}

void readEthercat()
{
  // reading hip commands
  hipInfo.mode = EASYCAT.BufferOut.Byte[mode];
  unsigned char motorCommandPackage[8];
  getXPCCommand(motorCommandPackage, 0);

  // Check if the received motor command is valid before storing it.
  if (isValidMotorCommand(motorCommandPackage))
  {
    memcpy(hipInfo.motorCommandPackage, motorCommandPackage, 8);
  }
  kneeInfo.mode = EASYCAT.BufferOut.Byte[mode + 9];
  getXPCCommand(motorCommandPackage, 9);

  // Check if the received motor command is valid before storing it.
  if (isValidMotorCommand(motorCommandPackage))
  {
    memcpy(kneeInfo.motorCommandPackage, motorCommandPackage, 8);
  }
}
void sendEthercat()
{
  // sending responses
  for (byte i = 0; i < 5; i++)
  {
    EASYCAT.BufferIn.Byte[i] = hipInfo.response[i];
    EASYCAT.BufferIn.Byte[i + 5] = kneeInfo.response[i];
  }
}

void initializeCanBus()
{
  while (CAN_OK != canHandler.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ))
  {
    delay(100);
  }
}

// -
// Motor
void getMotorResponse(uint8_t desiredId, motorInfo &motorInfo)
{
  //  Receiving data//
  unsigned char len = 0;
  long unsigned int rxId;
  unsigned char rawResponse[6];
  unsigned char messageSenderId;

  unsigned long startTime = micros();
  // while response is not received and timeout of 5 ms is not reached
  while (micros() - startTime < motorResponseTimeout)
  {
    if (canHandler.checkReceive() == CAN_MSGAVAIL)
    {
      canHandler.readMsgBuf(&rxId, &len, rawResponse); // CAN BUS reading
      messageSenderId = rawResponse[0];
      if (messageSenderId == desiredId)
      {
        memcpy(motorInfo.response, rawResponse + 1, 5);
        return;
      }
    }
  }
  return;
}

void sendMotorCommand(motorInfo &command, MotorHandler &motor)
{
  switch (command.mode)
  {
  case turnOff:
    motor.sendRawCommand(disableControllerBuf);
    motor.exitMotorMode();
    if (command.systemOn)
    {
      command.systemOn = false;
    }
    if (command.systemAtZero)
    {
      command.systemAtZero = false;
    }
    break;
  case position:
    if (!(command.systemOn))
    {
      motor.enterMotorMode();
      command.systemOn = true;
    }
    if (command.systemAtZero)
    {
      command.systemAtZero = false;
    }

    motor.sendRawCommand(command.motorCommandPackage);
    break;
  case torque:
    if (!(command.systemOn))
    {
      motor.enterMotorMode();
      command.systemOn = true;
    }
    if (command.systemAtZero)
    {
      command.systemAtZero = false;
    }
    motor.sendRawCommand(command.motorCommandPackage);
    break;
  case zeroPosition:
    if (!command.systemAtZero)
    {
      // NEED: think in a better solution for this
      motor.sendRawCommand(disableControllerBuf);
      motor.zeroPosition();
      command.systemAtZero = true;
    }
    else
    {
      motor.exitMotorMode();
    }

    break;
  default:
    break;
  }
}


void setup()
{
  // Begin the CAN Bus and set frequency to 8 MHz and baudrate of 1000kb/s  and the masks and filters disabled.
  initializeCanBus();
  canHandler.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted and received

  // Preparing motor to listen to commands
  hipMotor.exitMotorMode();
  hipMotor.zeroPosition();
  hipInfo.systemOn = false;
  hipInfo.systemAtZero = true;
  hipInfo.mode = turnOff;
  kneeMotor.exitMotorMode();
  kneeMotor.zeroPosition();
  kneeInfo.systemOn = false;
  kneeInfo.systemAtZero = true;
  hipInfo.mode = turnOff;

  // Ethercat
  EASYCAT.Init();
}

void loop()
{
  EASYCAT.MainTask();
  readEthercat();

  // send motor commands
  sendMotorCommand(hipInfo, hipMotor);
  getMotorResponse(hipId, hipInfo);
  sendMotorCommand(kneeInfo, kneeMotor);
  getMotorResponse(kneeId, kneeInfo);

  // send to XPC
  sendEthercat();
}