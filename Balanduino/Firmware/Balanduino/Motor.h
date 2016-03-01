#ifndef _motor_h_
#define _motor_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <Kalman.h>

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
  /* Counters used to count the pulses from the encoders */
  volatile int32_t leftCounter;
  volatile int32_t rightCounter;
  
  constexpr uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
  constexpr uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

  float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
  float gyroXzero;
  uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
  float accAngle, gyroAngle; // Result from raw accelerometer and gyroscope readings
  float pitch; // Result from Kalman filter
  float lastError; // Store last angle error
  float iTerm; // Store iTerm

/* Used for timing */
  uint32_t kalmanTimer; // Timer used for the Kalman filter
  uint32_t pidTimer; // Timer used for the PID loop
  uint32_t imuTimer; // This is used to set a delay between sending IMU values
  uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
  uint32_t reportTimer; // This is used to set a delay between sending report values
  uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
  uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

  bool steerStop; // Stop by default
  bool stopped; // This is used to set a new target position after braking

  bool layingDown; // Use to indicate if the robot is laying down

  float targetOffset; // Offset for going forward and backward
  float turningOffset; // Offset for turning left and right

  int32_t lastWheelPosition; // Used to calculate the wheel velocity
  int32_t wheelVelocity; // Wheel velocity based on encoder readings
  int32_t targetPosition; // The encoder position the robot should be at

  bool commandSent; // This is used so multiple controller can be used at once

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

  void checkmotors();
  void calculatePitch();
  void driveMotors();
  void updateEncoders();
  
  void setupEncoders();
  void setupMotors();
  void setupIMU();
  void setupTiming();
  void calibrateAndReset();

  void initBuzzer();
  void setBuzzer();
  void clearBuzzer();
  void soundBuzzer(int delay);

  void steer(Command command);
  float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);
}




#endif
