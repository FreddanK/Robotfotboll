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

 This is the algorithm for the Balanduino balancing robot.
 It can be controlled by either an Android app or a computer application via Bluetooth.
 The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 A dedicated Windows application can be found here: https://github.com/TKJElectronics/BalanduinoWindowsApp
 It can also be controlled by a PS3, PS4, Wii or a Xbox controller.
 Furthermore it supports the Spektrum serial protocol used for RC receivers.
 For details, see: http://balanduino.net/
*/

#include "Balanduino.h"

#include <Arduino.h> // Standard Arduino header
#include <Wire.h> // Official Arduino Wire library
#include <SPI.h> // Official Arduino SPI library

#include "Controller.h"
#include "Bluetooth.h"
#include "EEPROM.h"
#include "Motor.h"
#include "I2C.h"
#include "Tools.h"


#ifdef ENABLE_ADK
#include <adk.h>
#endif

// These are all open source libraries written by Kristian Lauszus, TKJ Electronics
// The USB libraries are located at the following link: https://github.com/felis/USB_Host_Shield_2.0
//#include <Kalman.h> // Kalman filter library - see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

#ifdef ENABLE_SPP
#include <SPP.h>
#endif

// Create the Kalman library instance
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

#if defined(ENABLE_SPP) || defined(ENABLE_ADK)
#define ENABLE_USB
USB Usb; // This will take care of all USB communication
#endif

#ifdef ENABLE_ADK
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
ADK adk(&Usb, "TKJ Electronics", // Manufacturer Name
              "Balanduino", // Model Name
              "Android App for Balanduino", // Description - user visible string
              "0.6.3", // Version of the Android app
              "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
              "1234"); // Serial Number - this is not used
#endif


#if defined(ENABLE_SPP)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
USBHub Hub(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
SPP SerialBT(&Btd, "Balanduino", "0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif

bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

float batteryVoltage; // Measured battery level
uint8_t batteryCounter; // Counter used to check if it should check the battery level

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

bool steerStop = true; // Stop by default
bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

float targetOffset = 0.0f; // Offset for going forward and backward
float turningOffset = 0.0f; // Offset for turning left and right

char dataInput[30]; // Incoming data buffer
bool bluetoothData; // True if data received is from the Bluetooth connection
float sppData1, sppData2; // Data send via SPP connection

bool commandSent; // This is used so multiple controller can be used at once

uint32_t receiveControlTimer;

int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at

Motor motor;

void setup() {

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup buzzer pin */
  motor.setupBuzzer();

#ifdef ENABLE_USB
  if (Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start"));
    motor.setBuzzer();
    while (1); // Halt
  }
#endif
  
  /* Read the PID values, target angle and other saved values in the EEPROM */
  if (!checkInitializationFlags()) {
    readEEPROMValues(); // Only read the EEPROM values if they have not been restored
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    motor.soundBuzzer(1000);
    delay(100); // Wait a little after the pin is cleared
  }

  motor.setupEncoders();
  
  motor.setupMotors();

  motor.setupIMU();

#ifdef ENABLE_TOOLS
  printMenu();
#endif
  
  motor.calibrateAndReset();
  
  /* Beep to indicate that it is now ready */
  motor.soundBuzzer(100);

  motor.setupTiming();
}

void loop() {

  motor.checkMotors();

  /* Calculate pitch */
  while (i2cRead(0x3D, i2cBuffer, 8));
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    float gyroRate = ((float)gyroX - gyroXzero) / 131.0f; // Convert to deg/s
    float dt = (float)(timer - kalmanTimer) / 1000000.0f;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = timer;
  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);
  //motor.calculatePitch();

  /* Drive motors */
  timer = micros();
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } else {
    layingDown = false; // It's no longer laying down
    updatePID(cfg.targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
  }
  pidTimer = timer;

  /* Update encoders */
  timer = millis();
  if (timer - encoderTimer >= 100) { // Update encoder values every 100ms
    encoderTimer = timer;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }

    batteryCounter++;
    if (batteryCounter >= 10) { // Measure battery every 1s
      batteryCounter = 0;
      batteryVoltage = (float)analogRead(VBAT) / 63.050847458f; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
      if (batteryVoltage < 11.1 && batteryVoltage > 5) // Equal to 3.7V per cell - don't turn on if it's below 5V, this means that no battery is connected
        buzzer::Set();
      else
        buzzer::Clear();
    }
  }

  /* Read the Bluetooth dongle and send PID and IMU values */
#if defined(ENABLE_TOOLS)
  checkSerialData();
#endif
#if defined(ENABLE_USB)
  readUsb();
#endif
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
  printValues();
#endif

#if defined(ENABLE_AI)
  setControlOffset();
#endif

#ifdef ENABLE_BTD
  if (Btd.isReady()) {
    timer = millis();
    if ((Btd.watingForConnection && timer - blinkTimer > 1000) || (!Btd.watingForConnection && timer - blinkTimer > 100)) {
      blinkTimer = timer;
      LED::Toggle(); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } else
    LED::Clear(); // This will turn it off
#endif
}
