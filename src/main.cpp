#include <Arduino.h>
#include <mcp_can.h>
#include <MotorHandler.h>
#include <EasyCAT.h>

// DEFINITIONS

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
  unsigned long lastResponseTime;
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
  // TODO: decide if this is ISR or not

  // reading hip commands
  hipInfo.mode = EASYCAT.BufferOut.Byte[mode];
  unsigned char motorCommandPackage[8];
  getXPCCommand(motorCommandPackage, 0);

  // Check if the received motor command is valid before storing it.
  if (isValidMotorCommand(motorCommandPackage))
  {
    // NEED: remove this checking after testing
    boolean sameCommand = memcmp(hipInfo.motorCommandPackage, motorCommandPackage, 8) == 0;

    if (!sameCommand)
    {
      Serial.println("New hip command");
      // Store the valid motor command in hipInfo.
      memcpy(hipInfo.motorCommandPackage, motorCommandPackage, 8);
    }
  }
  else
  {
    // TODO: handle invalid command
  }

  kneeInfo.mode = EASYCAT.BufferOut.Byte[mode + 9];
  getXPCCommand(motorCommandPackage, 9);

  // Check if the received motor command is valid before storing it.
  if (isValidMotorCommand(motorCommandPackage))
  {
    // NEED: remove this checking after testing
    boolean sameCommand = memcmp(kneeInfo.motorCommandPackage, motorCommandPackage, 8) == 0;

    if (!sameCommand)
    {
      Serial.println("New knee command");
      // Store the valid motor command in hipInfo.
      memcpy(kneeInfo.motorCommandPackage, motorCommandPackage, 8);
    }
  }
  else
  {
    // TODO: handle invalid command
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
    Serial.println("CAN Bus initialization failed");
    Serial.println("Initializing CAN Bus again");
    delay(100);
  }
  Serial.println("CAN Bus init OK");
}

// -
// Motor
void getMotorResponse(uint8_t desiredId)
{
  //  Receiving data//
  // NEED: check what this rxid is
  unsigned char len = 0;
  long unsigned int rxId;
  unsigned char rawResponse[6];
  unsigned char messageSenderId;

  if (canHandler.checkReceive() == CAN_MSGAVAIL)
  {
    canHandler.readMsgBuf(&rxId, &len, rawResponse); // CAN BUS reading
    messageSenderId = rawResponse[0];

    if (messageSenderId == desiredId)
    {
      if (desiredId == hipId)
      {
        std::memcpy(hipInfo.response, rawResponse + 1, 5);
        hipInfo.lastResponseTime = millis();
      }
      else
      {
        std::memcpy(kneeInfo.response, rawResponse + 1, 5);
        kneeInfo.lastResponseTime = millis();
      }
    }
    else
    {
      Serial.print("Not desired ID: ");
      Serial.println(desiredId);
      for (size_t i = 0; i < 6; i++)
      {
        Serial.print(rawResponse[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  else
  {
        Serial.print(" CAN BUS EMPTY");
  }
  
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
      Serial.println("Turning motor off");
      command.systemOn = false;
      // TODO: check if response was successful
    }
    if (command.systemAtZero)
    {
      command.systemAtZero = false;
    }
    break;
  case position:
    if (!(command.systemOn))
    {
      Serial.println("Turning motor on");
      motor.enterMotorMode();
      command.systemOn = true;
      // TODO: check if response was successful
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
      Serial.println("Turning motor on");
      motor.enterMotorMode();
      command.systemOn = true;
      // TODO: check if response was successful
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
      Serial.println("Zeroing motor");
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

// --
// DEBUGGING
// NEED: remove this after testing
unsigned long milisMeasured;
// Loop Measerement
unsigned int counter = 0;
const int nLoops = 10000;
unsigned long microsStart;
void printHipStatus()
{
  Serial.print("Knee status: ");
  Serial.println(kneeInfo.mode);
  // print motor command package without string and avoiding memory leaks
  for (int i = 0; i < 8; i++)
  {
    Serial.print(kneeInfo.motorCommandPackage[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  milisMeasured = millis();
}

void setup()
{
  Serial.begin(115200); // Begin Serial port to talk to computer and receive commands

  // Begin the CAN Bus and set frequency to 8 MHz and baudrate of 1000kb/s  and the masks and filters disabled.
  initializeCanBus();
  canHandler.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted and received

  // Preparing motor to listen to commands
  hipMotor.exitMotorMode();
  hipMotor.zeroPosition();
  hipInfo.systemOn = false;
  hipInfo.systemAtZero = true;
  hipInfo.lastResponseTime = millis();
  kneeMotor.exitMotorMode();
  kneeMotor.zeroPosition();
  kneeInfo.systemOn = false;
  kneeInfo.systemAtZero = true;
  kneeInfo.lastResponseTime = millis();

  // Ethercat
  if (EASYCAT.Init() == true) // initilization succesfully completed
  {
    Serial.println("EtherCAT initialization completed");
  }

  else // initialization failed
  {
    Serial.println("EtherCAT initialization failed");
  }

  counter = 0;
  milisMeasured = millis();
  Serial.println("Setup finished.");
}

void loop()
{
  // NEED: test loop time after successful implementation
  // measure the time for 10000 loops
  if (counter == 0)
  {
    microsStart = micros();
  }
  else if (counter == nLoops)
  {
    Serial.print("Time for 10000 loops: ");
    Serial.println(micros() - microsStart);
    counter = 0;
  }
  // Check for characters received from the Serial Monitor

  // NEED: remove this after testing
  if (Serial.available() > 0)
  {
    char receivedChar = Serial.read();

    // Perform different actions based on the received character
    if (receivedChar == 's')
    {
      // Action for character 's'
      printHipStatus();
    }
    else if (receivedChar == 'a')
    {
      Serial.println("Hip response: ");
      // print motor command package without string and avoiding memory leaks
      for (int i = 0; i < 5; i++)
      {
        Serial.print(hipInfo.response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      Serial.println("Knee response: ");
      // print motor command package without string and avoiding memory leaks
      for (int i = 0; i < 5; i++)
      {
        Serial.print(kneeInfo.response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    else if (receivedChar == 'z')
    {
      hipMotor.normalSet(0, 0, 0);
      // Add your code here for what you want to do when 'z' is received
    }
    else if (receivedChar == 'p')
    {
      kneeMotor.printPrettyResponse(kneeMotor.getMotorResponse(false));
    }
    else if (receivedChar == 'q')
    {
      Serial.print("Hip last resonse: ");
      Serial.println(millis() - hipInfo.lastResponseTime);
      Serial.print("Knee last resonse: ");
      Serial.println(millis() - kneeInfo.lastResponseTime);
    }
    else
    {
      // Invalid character received
      Serial.println("Invalid character received");
    }
  }

  // every 1second send the status of the motor and the package
  // if (millis() - milisMeasured > 10000)
  // {
  //   printHipStatus();
  //   milisMeasured = millis();
  // }

  EASYCAT.MainTask();
  readEthercat();

  // send motor commands
  sendMotorCommand(hipInfo, hipMotor);
  getMotorResponse(hipId);
  canHandler.clearMsg();
  sendMotorCommand(kneeInfo, kneeMotor);
  getMotorResponse(kneeId);
  canHandler.clearMsg();



  // send to XPC
  sendEthercat();
  counter++;

}