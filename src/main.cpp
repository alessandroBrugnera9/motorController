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
  turnOff,
  position,
  torque,
  zeroPosition,
};

struct motorCommand
{
  uint8_t mode;
  boolean systemOn;
  uint8_t motorCommandPackage[8];
};

motorCommand hipCommand;
motorCommand kneeCommand;
uint8_t hipResponse[5];
uint8_t kneeResponse[5];

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

void getMotorCommand(unsigned char *motorCommandPackage, int startIndex)
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
  // TODO: read from XPC

  // reading hip commands
  hipCommand.mode = EASYCAT.BufferOut.Byte[mode];
  unsigned char motorCommandPackage[8];
  getMotorCommand(motorCommandPackage, 0);

  // Check if the received motor command is valid before storing it.
  if (isValidMotorCommand(motorCommandPackage))
  {
    boolean sameCommand = memcmp(hipCommand.motorCommandPackage, motorCommandPackage, 8) == 0;

    if (!sameCommand)
    {
      Serial.println("New hip command");
      // Store the valid motor command in hipCommand.
      memcpy(hipCommand.motorCommandPackage, motorCommandPackage, 8);
    }
  }
  else
  {
    // TODO: handle invalid command
  }

  // reading knee comands
  // kneeCommand.mode = EASYCAT.BufferOut.Byte[mode + 9];
  // unsigned char *kneePointer = getMotorCommand(8);
  // if (isValidMotorCommand(kneePointer))
  // {
  //   // Serial.print("Knee mode:");
  //   // Serial.println(kneeCommand.mode);

  //   std::memcpy(kneeCommand.motorCommandPackage, kneePointer, 8);
  // }
}

void sendEthercat()
{
  // TODO: handler variables to send to XPC
  // TODO: send to XPC

  // sending responses
  for (byte i = 0; i < 5; i++)
  {
    EASYCAT.BufferIn.Byte[i] = hipResponse[i];
    EASYCAT.BufferIn.Byte[i + 5] = kneeResponse[i];
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
  unsigned char hipRawResponse[6];
  if (hipMotor.getRawMotorResponse(hipRawResponse))
  {
    std::memcpy(hipResponse, hipRawResponse+1, 5);
  }
  unsigned char kneeRawResponse[6];
  if (kneeMotor.getRawMotorResponse(kneeRawResponse))
  {
    std::memcpy(kneeResponse, kneeRawResponse+1, 5);
  }
}

void sendMotorCommand(motorCommand &command, MotorHandler &motor)
{
  switch (command.mode)
  {
  case turnOff:
    if (command.systemOn)
    {
      Serial.println("Turning motor off");
      motor.exitMotorMode();
      command.systemOn = false;
      // TODO: check if response was successful
    }
    break;
  case position:
    if (!(command.systemOn))
    {
      Serial.println("Turning motor on");
      hipMotor.enterMotorMode();
      command.systemOn = true;
      // TODO: check if response was successful
    }

    motor.sendRawCommand(command.motorCommandPackage);
    break;
  case torque:
    // TODO: implement on on torque mode
    motor.sendRawCommand(command.motorCommandPackage);
    break;
  case zeroPosition:
    // motor.zeroPosition();
    Serial.println("Zeroing not implemented yet");
    break;
  default:
    break;
  }
}

unsigned long milisMeasured;

// DEBUGGING
void printHipStatus()
{
  Serial.print("Hip status: ");
  Serial.println(hipCommand.mode);
  // print motor command package without string and avoiding memory leaks
  for (int i = 0; i < 8; i++)
  {
    Serial.print(hipCommand.motorCommandPackage[i], HEX);
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
  hipCommand.systemOn = false;
  kneeMotor.exitMotorMode();
  kneeMotor.zeroPosition();
  kneeCommand.systemOn = false;

  // Ethercat
  if (EASYCAT.Init() == true) // initilization succesfully completed
  {
    Serial.println("EtherCAT initialization completed");
    EASYCAT.MainTask(); // execute the EasyCAT task
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
  // measure the time for 100 loops
  // if (counter == 0)
  // {
  //   microsStart = micros();
  // }
  // else if (counter == nLoops)
  // {
  //   Serial.print("Time for 100 loops: ");
  //   Serial.println(micros() - microsStart);
  //   counter = 0;
  // }
  // Check for characters received from the Serial Monitor
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
        Serial.print(hipResponse[i], HEX);
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
      hipMotor.printPrettyResponse(hipMotor.getMotorResponse(false));
    }
    else
    {
      // Invalid character received
      Serial.println("Invalid character received");
    }
  }

  // every 1second send the status of the motor and the package
  if (millis() - milisMeasured > 10000)
  {
    printHipStatus();
    milisMeasured = millis();
  }

  EASYCAT.MainTask();
  readEthercat();

  // send motor commands
  sendMotorCommand(hipCommand, hipMotor);
  sendMotorCommand(kneeCommand, kneeMotor);

  getMotorsResponses();

  // send to XPC
  sendEthercat();
  // counter++;
}