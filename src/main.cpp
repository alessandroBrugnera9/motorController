#include <Arduino.h>
#include <mcp_can.h>
#include <MotorHandler.h>

// MOTOR CONTROL PARAMTERS
const int baseKp = 300;
const int baseKd = 4;

// CONTROL VARIABLES
struct motorCommand
{
  motorMode mode;
  int target;
};

enum motorMode
{
  position,
  torque
};

motorCommand hipCommand;
motorCommand kneeCommand;
motorResponse hipResponse;
motorResponse kneeResponse;

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

void readXPC()
{
  // TODO: decide if this is ISR or not
  // TODO: read from XPC
}

void sendXPC()
{

  // TODO: handler variables to send to XPC
  // TODO: send to XPC
}

void getMotorsResponses()
{
  hipResponse = hipMotor.getMotorResponse(false);
  kneeResponse = kneeMotor.getMotorResponse(false);
}

void sendMotorCommand(motorCommand &command, MotorHandler motor)
{
  switch (command.mode)
  {
  case position:
    motor.normalSet(command.target, 0, 0);
    break;
  case torque:
    motor.normalSet(0, 0, command.target);
    break;
  default:
    break;
  }
}

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

  readXPC();

  // send motor commands
  sendMotorCommand(hipCommand, hipMotor);
  sendMotorCommand(kneeCommand, kneeMotor);

  getMotorsResponses();

  // send to XPC
  sendXPC();
  counter++;
}