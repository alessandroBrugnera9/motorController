#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <mcp_can.h>

class MotorHandler
{
private:
  // MOTOR PARAMeter
  unsigned char id;

  const float kp_max = 500;
  const float kp_min = 0;
  unsigned char kp_16h_hex;
  unsigned char kp_16l_hex;

  const float kd_max = 5;
  const float kd_min = 0;
  unsigned char kd_16h_hex;
  unsigned char kd_16l_hex;

  const float pos_max = 6.28 * 20;
  const float pos_min = -6.28 * 20;

  const float vel_max = 45;
  const float vel_min = -45;

  const float tor_max = 18;
  const float tor_min = -18;

  // Controller/Communication Parameters
  MCP_CAN &canHandler;

  // used to store the last command sent to the motor
  unsigned char commandBuffer[8];

  // used to handle the response from the motor
  const uint32_t RESPONSE_TIMEOUT = 5;

public:
  /**
   * Constructor for MotorHandler class.
   *
   * @param CAN The MCP_CAN instance used for communication.
   * @param canId The CAN ID of the motor.
   * @param _kp The proportional gain for position control.
   * @param _kd The derivative gain for position control.
   */
  MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd);
  // Controller functions
  /**
   * Set the proportional gain (Kp).
   *
   * @param _kp The new value for Kp.
   */
  void setKp(float _kp);

  /**
   * Set the derivative gain (Kd).
   *
   * @param _kd The new value for Kd.
   */
  void setKd(float _kd);

  /**
   * Get the ID of the motor in the CAN bus.
   *
   * @return The ID value as an unsigned char.
   */
  unsigned char getId() const;

  // ---------------------------------------
  // Controller control Funcions

  /**
   * Sends a command to exit motor mode.
   */
  boolean exitMotorMode();

  /**
   * Sends a command to enter motor mode.
   */
  boolean enterMotorMode();

  /**
   * Resets the motor position to zero.
   *
   * @return True if the position reset command was sent successfully, false otherwise.
   */
  boolean zeroPosition();

  // Direct CAN Bus Functions
  /**
   * @brief Sends a CAN message to the motor.
   *
   * @param command The command to be sent to the motor, that is uint8_t array of size 8. //TODO: detail this command
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte sendRawCommand(unsigned char *command);
};
#endif
