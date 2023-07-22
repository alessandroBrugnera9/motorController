#include <Arduino.h>
#include <mcp_can.h>
#include <MotorHandler.h>

const int baseKp = 300;
const int baseKd = 4;


// setting can bus handler though SPI pins
const byte spiCSPin = 10;
MCP_CAN canHandler(spiCSPin);

// motor object instantiation
MotorHandler hipMotor(canHandler, 0x01, baseKp, baseKd);
MotorHandler kneeMotor(canHandler, 0x02,  baseKp, baseKd);

void setup() {
  Serial.begin(9600); // Begin Serial port to talk to computer and receive commands

  // Begin the CAN Bus and set frequency to 8 MHz and baudrate of 1000kb/s  and the masks and filters disabled.
  initializeCanBus();
  canHandler.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted and received

  // Preparing motor to listen to commands
  motor.exitMotorMode();
  motor.zeroPosition();

  lastOutputMillis = millis();
  Serial.println("Setup finished.");
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}