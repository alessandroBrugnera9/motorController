#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <mcp_can.h>
#ifndef dtostrf
#include <DtoStrf.h>
#endif

struct motorResponse
{
  float position;
  float velocity;
  float current;
};

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

protected:
  /**
   * @brief Builds the position package for the GIM8115 device.
   *
   * This function takes the target position and returns an array of size 2 representing the position package.
   * @param tarPos The target position.
   * @return An array of size 2 representing the position package.
   */
  float *buildPositionPackage(float tarPos);

  /**
   * @brief Builds the velocity package for the GIM8115 device.
   *
   * This function takes the target velocity and returns an array of size 2 representing the velocity package.
   * @param tarVel The target velocity.
   * @return An array of size 2 representing the velocity package.
   */
  float *buildVelocityPackage(float tarVel);

  /**
   * @brief Builds the torque package for the GIM8115 device.
   *
   * This function takes the target torque and returns an array of size 2 representing the torque package.
   * @param tarTor The target torque.
   * @return An array of size 2 representing the torque package.
   */
  float *buildTorquePackage(float tarTor);

  /**
   * Reads the motor response data from the MCP_CAN bus and returns the values.
   *
   * @return A motorResponse struct containing the position, velocity, and current values of the last motor response.
   */
  motorResponse handleMotorResponse();

public:
  // used to handle the response from the motor
  motorResponse EMPTY_RESPONSE = {float(-1.0), float(-1.0), float(-1.0)};

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

  /**
   * @brief Sets the motor to a normal mode with target position, velocity, and torque values.
   *
   * @param tarPos The target position value.
   * @param tarVel The target velocity value.
   * @param tarTor The target torque value.
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte normalSet(float tarPos, float tarVel, float tarTor);

  /**
   * Set the motor to torque mode.
   *
   * @param tarTor The target torque value for the motor.
   */
  void torqueSet(float tarTor);

  // ---------------------------------------
  // Motor Response Handling
  /**
   * @brief Clears the receive buffer by reading and discarding all available messages.
   */
  void clearCANBuffer();

  /**
   * @brief Retrieves the motor response.
   *
   * This function clears the receive buffer, enters or exits motor mode based on the current state,
   * handles the motor response, and returns the position, velocity, and current values.
   *
   * @param clearBuffer A boolean value indicating whether or not to clear the receive buffer.
   * @return A motorResponse struct containing the position, velocity, and current values of the last motor response.
   */
  motorResponse getMotorResponse(boolean sendLastCommand = true);

  /**
   * Prints the motor response in a formatted and readable manner.
   *
   * @param res The motorResponse struct containing the position, velocity, and current values.
   */
  void printPrettyResponse(motorResponse res);
};

#endif
