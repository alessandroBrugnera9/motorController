#include "MotorHandler.h"
//TODO: organize the funcitons in this file

float *MotorHandler::buildPositionPackage(float tarPos)
{
  float *posPackage = new float[2];

  // Position package building code
  unsigned int pos_b;
  if (tarPos > pos_max)
  {
    tarPos = pos_max;
  }
  else if (tarPos < pos_min)
  {
    tarPos = pos_min;
  }
  // TODO: make this math better
  tarPos = tarPos + 95.5;
  pos_b = 65535 * tarPos / 191;
  posPackage[0] = pos_b / 256;
  posPackage[1] = pos_b % 256;

  return posPackage;
}

float *MotorHandler::buildVelocityPackage(float tarVel)
{
  float *velPackage = new float[2];

  // Velocity package building code
  unsigned int vel_b;
  if (tarVel > vel_max)
  {
    tarVel = vel_max;
  }
  else if (tarVel < vel_min)
  {
    tarVel = vel_min;
  }
  // TODO: make this math better
  tarVel = tarVel + 45;
  vel_b = 4095 * tarVel / 90;
  velPackage[0] = vel_b / 16;
  velPackage[1] = vel_b % 16;

  return velPackage;
}

float *MotorHandler::buildTorquePackage(float tarTor)
{
  float *torPackage = new float[2];

  // Torque package building code
  unsigned int tor_b;
  if (tarTor > tor_max)
  {
    tarTor = tor_max;
  }
  else if (tarTor < tor_min)
  {
    tarTor = tor_min;
  }
  tarTor = tarTor + 18;
  tor_b = 4095 * tarTor / 36;
  torPackage[0] = tor_b / 256;
  torPackage[1] = tor_b % 256;

  return torPackage;
}

MotorHandler::MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd) : canHandler(CAN)
{
  // Motor parameters setting
  id = static_cast<unsigned char>(canId);
  setKp(_kp);
  setKd(_kd);

  //NEED: set commandBuffer to postion zero
  
}

void MotorHandler::setKp(float _kp)
{
  if (_kp > kp_max)
  {
    _kp = kp_max;
  }
  else if (_kp < kp_min)
  {
    _kp = kp_min;
  }
  // TODO: make this math better
  unsigned int kp_b = static_cast<unsigned int>(4095 * _kp / 500);
  float kp_16h = kp_b / 256;
  float kp_16l = kp_b % 256;
  kp_16h_hex = kp_16h;
  kp_16l_hex = kp_16l;
}

unsigned char MotorHandler::getId() const
{
  return id;
}

void MotorHandler::setKd(float _kd)
{
  if (_kd > kd_max)
  {
    _kd = kd_max;
  }
  else if (_kd < kd_min)
  {
    _kd = kd_min;
  }
  unsigned int kd_b = static_cast<unsigned int>(4095 * _kd / 5);
  float kd_16h = kd_b / 16;
  float kd_16l = kd_b % 16;
  kd_16h_hex = kd_16h;
  kd_16l_hex = kd_16l;
}

boolean MotorHandler::exitMotorMode()
{
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;

  byte sndStat = canHandler.sendMsgBuf(id, 0, 8, buf);

  return sndStat;
}
boolean MotorHandler::enterMotorMode()
{
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;

  byte sndStat = canHandler.sendMsgBuf(id, 0, 8, buf);

  return sndStat;
}

void MotorHandler::torqueSet(float tarTor)
{
  // Call the private functions to build the individual packages
  float *torPackage = buildTorquePackage(tarTor);

  //----------------------------------------------------------------------------//
  // Sending data//
  memset(commandBuffer, 0, 8); // setting array to 0
  commandBuffer[6] = torPackage[0];
  commandBuffer[7] = torPackage[1];

  canHandler.sendMsgBuf(getId(), 0, 8, commandBuffer);
}

boolean MotorHandler::zeroPosition()
{
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;

  byte sendStatus = canHandler.sendMsgBuf(id, 0, 8, buf);

  // check if it was sent succesfully
  if (sendStatus == CAN_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void MotorHandler::clearCANBuffer()
{
  while (canHandler.checkReceive() == CAN_MSGAVAIL)
  {
    INT32U id;
    INT8U ext;
    INT8U len;
    INT8U buf[MAX_CHAR_IN_MESSAGE];
    canHandler.readMsgBuf(&id, &ext, &len, buf);
  }
}

motorResponse MotorHandler::getMotorResponse()
{
  canHandler.sendMsgBuf(getId(), 0, 8, commandBuffer);

  // wait for response or timeout in 5 ms
  // NEED: check if timeout is too big
  unsigned long startTime = millis();
  while (canHandler.checkReceive() != CAN_MSGAVAIL && millis() - startTime < RESPONSE_TIMEOUT);

  if (canHandler.checkReceive() != CAN_MSGAVAIL)
  {
    return EMPTY_RESPONSE;
  }
  else
  {
    return (handleMotorResponse());
  }
}

void MotorHandler::printPrettyResponse(motorResponse res)
{
  char outString[64]; // Array to store serial string
  char posString[8];  // Position string
  char velString[8];  // Velocity string
  char currString[8]; // Current string

  dtostrf(res.position, 6, 3, posString);
  dtostrf(res.velocity, 6, 3, velString);
  dtostrf(res.current, 6, 3, currString);

  sprintf(outString, "Position: %s  Velocity: %s  Current: %s", posString, velString, currString);
  // Serial.println(msgString2);

  // sprintf(outString, "Position: %.3f  Velocity: %.3f  Current: %.3f", res.position, res.velocity, res.current);
  Serial.println(outString);
}

byte MotorHandler::normalSet(float tarPos, float tarVel, float tarTor)
{
  // Call the private functions to build the individual packages
  float *posPackage = buildPositionPackage(tarPos);
  float *velPackage = buildVelocityPackage(tarVel);
  float *torPackage = buildTorquePackage(tarTor);

  //----------------------------------------------------------------------------//
  // Sending data//
  // TODO: make this math better
  commandBuffer[0] = posPackage[0];
  commandBuffer[1] = posPackage[1];
  commandBuffer[2] = velPackage[0];
  commandBuffer[3] = velPackage[1] * 16 + kp_16h_hex;
  commandBuffer[4] = kp_16l_hex;
  commandBuffer[5] = kd_16h_hex;
  commandBuffer[6] = kd_16l_hex * 16 + torPackage[0];
  commandBuffer[7] = torPackage[1];

  byte sendStatus = canHandler.sendMsgBuf(id, 0, 8, commandBuffer);

  return sendStatus;
}

motorResponse MotorHandler::handleMotorResponse()
{
  //  Receiving data//
  unsigned char len = 0;
  long unsigned int rxId;
  byte buf_received[6];

  canHandler.readMsgBuf(&rxId, &len, buf_received); // CAN BUS reading

  // Collecting bits from CAN_BUS signal
  unsigned int pos_motor = ((buf_received[1] << 8) | buf_received[2]);        // Position reading
  unsigned int vel_motor = (buf_received[3] << 4) | ((buf_received[4]) >> 4); // Velocity reading
  unsigned int cur_motor = ((buf_received[4] & 0xF) << 8) | buf_received[5];  // Current reading

  // Converting to readable data//
  // TODO: make this math better
  float pos_f = (float)pos_motor;
  pos_f = (pos_f * 191 / 65535) - 95.5;
  float vel_f = (float)vel_motor;
  vel_f = (vel_f * 90 / 4095) - 45;
  float cur_f = (float)cur_motor;
  cur_f = (cur_f * 36 / 4095) - 18;

  // Create an instance of the motorResponse struct and assign the values
  motorResponse response;
  response.position = pos_f;
  response.velocity = vel_f;
  response.current = cur_f;

  // Return the MotorResponse struct
  return response;
}