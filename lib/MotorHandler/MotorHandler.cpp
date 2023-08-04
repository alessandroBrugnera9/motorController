#include "MotorHandler.h"

MotorHandler::MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd) : canHandler(CAN)
{
  // Motor parameters setting
  id = static_cast<unsigned char>(canId);
  setKp(_kp);
  setKd(_kd);
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

byte MotorHandler::sendRawCommand(unsigned char *command)
{
  // assign commandBuffer to buf
  memcpy(commandBuffer, command, sizeof(commandBuffer));
  byte sndStat = canHandler.sendMsgBuf(id, 0, 8, commandBuffer);
  return sndStat;
}