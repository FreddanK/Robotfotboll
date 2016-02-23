/* Copyright (C) 2013-2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

#ifndef _balanduino_h_
#define _balanduino_h_

/* Use this to enable and disable the different options */
#define ENABLE_TOOLS
//#define ENABLE_SPP
//#define ENABLE_ADK
#define ENABLE_AI

#ifdef ENABLE_ADK
#include <adk.h>
#endif

#ifdef ENABLE_SPP
#include <SPP.h>
#endif

#include <Kalman.h>
extern Kalman kalman;

#if defined(ENABLE_SPP) || defined(ENABLE_ADK)
#define ENABLE_USB
  extern USB Usb; // This will take care of all USB communication
#else
  #include <Usb.h> // Include this from the USB Host library
#endif

#ifdef ENABLE_ADK
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
  extern ADK adk;
#endif


#if defined(ENABLE_SPP)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
extern USBHub Hub; // Some dongles have a hub inside
extern BTD Btd; // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
extern SPP SerialBT; // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif

#if ARDUINO < 156 // Make sure that at least Arduino IDE version 1.5.6 is used
  #error "Please update the Arduino IDE to version 1.5.6 or newer at the following website: http://arduino.cc/en/Main/Software"
#endif

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

/* Firmware Version Information */
constexpr char *version = "1.1.0";
constexpr uint8_t eepromVersion = 3; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

extern bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

constexpr uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
constexpr uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

// Buzzer used for feedback, it can be disconnected using the jumper
#if BALANDUINO_REVISION < 13
  #define buzzer P5
#else
  #define buzzer P11 /* A4 */
#endif

#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

#define VBAT A5 // Not broken out - used for battery voltage measurement

/* Counters used to count the pulses from the encoders */
extern volatile int32_t leftCounter;
extern volatile int32_t rightCounter;

extern float batteryVoltage; // Measured battery level
extern uint8_t batteryCounter; // Counter used to check if it should check the battery level

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
  bool bindSpektrum;
} cfg_t;

extern cfg_t cfg;

/* EEPROM Address Definitions */
constexpr uint8_t initFlagsAddr = 0; // Set the first byte to the EEPROM version
constexpr uint8_t configAddr = 1; // Save the configuration starting from this location

extern float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
extern float gyroXzero;
extern uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
extern float accAngle, gyroAngle; // Result from raw accelerometer and gyroscope readings
extern float pitch; // Result from Kalman filter

extern float lastError; // Store last angle error
extern float iTerm; // Store iTerm

/* Used for timing */
extern uint32_t kalmanTimer; // Timer used for the Kalman filter
extern uint32_t pidTimer; // Timer used for the PID loop
extern uint32_t imuTimer; // This is used to set a delay between sending IMU values
extern uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
extern uint32_t reportTimer; // This is used to set a delay between sending report values
extern uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
extern uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

extern bool steerStop; // Stop by default
extern bool stopped; // This is used to set a new target position after braking

extern bool layingDown; // Use to indicate if the robot is laying down

extern float targetOffset; // Offset for going forward and backward
extern float turningOffset; // Offset for turning left and right

extern char dataInput[30]; // Incoming data buffer
extern bool bluetoothData; // True if data received is from the Bluetooth connection
extern float sppData1, sppData2; // Data send via SPP connection

extern bool commandSent; // This is used so multiple controller can be used at once

extern uint32_t receiveControlTimer;
constexpr uint16_t receiveControlTimeout = 500; // After how long time should it should prioritize the other controllers instead of the serial control

extern int32_t lastWheelPosition; // Used to calculate the wheel velocity
extern int32_t wheelVelocity; // Wheel velocity based on encoder readings
extern int32_t targetPosition; // The encoder position the robot should be at

/* Encoders */
#if BALANDUINO_REVISION < 13
  #define leftEncoder1Pin 15 // Used for attachInterrupt
  #define leftEncoder2Pin 30 // Used for pin change interrupt
  #define rightEncoder1Pin 16 // Used for attachInterrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#else
  #define leftEncoder1Pin 25 // Used for pin change interrupt
  #define leftEncoder2Pin 26 // Used for pin change interrupt
  #define rightEncoder1Pin 30 // Used for pin change interrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#endif

#define MAKE_PIN(pin) _MAKE_PIN(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define _MAKE_PIN(pin) P ## pin

#define leftEncoder1 MAKE_PIN(leftEncoder1Pin)
#define leftEncoder2 MAKE_PIN(leftEncoder2Pin)

#define rightEncoder1 MAKE_PIN(rightEncoder1Pin)
#define rightEncoder2 MAKE_PIN(rightEncoder2Pin)

// You should change these to match your pins
#if BALANDUINO_REVISION < 13
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT0_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#else
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT1_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#endif

// Encoder values
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
  constexpr uint16_t zoneA = 8000 * 2;
  constexpr uint16_t zoneB = 4000 * 2;
  constexpr uint16_t zoneC = 1000 * 2;
  constexpr float positionScaleA = 600.0f * 2.0f; // One resolution is 1856 pulses per encoder
  constexpr float positionScaleB = 800.0f * 2.0f;
  constexpr float positionScaleC = 1000.0f * 2.0f;
  constexpr float positionScaleD = 500.0f * 2.0f;
  constexpr float velocityScaleMove = 70.0f * 2.0f;
  constexpr float velocityScaleStop = 60.0f * 2.0f;
  constexpr float velocityScaleTurning = 70.0f * 2.0f;
#else
  constexpr uint16_t zoneA = 8000;
  constexpr uint16_t zoneB = 4000;
  constexpr uint16_t zoneC = 1000;
  constexpr float positionScaleA = 600.0f; // One resolution is 928 pulses per encoder
  constexpr float positionScaleB = 800.0f;
  constexpr float positionScaleC = 1000.0f;
  constexpr float positionScaleD = 500.0f;
  constexpr float velocityScaleMove = 70.0f;
  constexpr float velocityScaleStop = 60.0f;
  constexpr float velocityScaleTurning = 70.0f;
#endif


#endif
