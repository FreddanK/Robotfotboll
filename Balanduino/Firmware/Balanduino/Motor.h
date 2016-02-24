#ifndef _motor_h_
#define _motor_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <kalman.h>

enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

// This struct will store all the configuration values
typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
} cfg_t;

class Motor {
private:
  float iTerm;

  bool calibrateGyro();
  bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
public:
  Command lastCommand;
  cfg_t cfg;
  Kalman kalman;

  void updatePID(float restAngle, float offset, float turning, float dt);
  void moveMotor(Command motor, Command direction, float speedRaw);
  void stopMotor(Command motor);
  void setPWM(Command motor, uint16_t dutyCycle);
  void stopAndReset();
  void leftEncoder();
  void rightEncoder();
  int32_t readLeftEncoder();
  int32_t readRightEncoder();
  int32_t getWheelsPosition();
  
  void setupEncoders();
  void setupMotors();
  void setupIMU();
  void setupTiming();
  void checkmotors();
  void calibrateAndReset();

  void setupBuzzer();
  void setBuzzer();
  void clearBuzzer();
  void soundBuzzer(int delay);
}







#endif
